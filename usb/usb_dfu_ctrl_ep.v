module usb_dfu_ctrl_ep #(
  parameter MAX_IN_PACKET_SIZE = 32,
  parameter MAX_OUT_PACKET_SIZE = 32,
) (
  input clk,
  input reset,
  output [6:0] dev_addr,

  ////////////////////
  // out endpoint interface
  ////////////////////
  output out_ep_req,
  input out_ep_grant,
  input out_ep_data_avail,
  input out_ep_setup,
  output out_ep_data_get,
  input [7:0] out_ep_data,
  output out_ep_stall,
  input out_ep_acked,

  ////////////////////
  // in endpoint interface
  ////////////////////
  output in_ep_req,
  input in_ep_grant,
  input in_ep_data_free,
  output in_ep_data_put,
  output reg [7:0] in_ep_data = 0,
  output in_ep_data_done,
  output reg in_ep_stall,
  input in_ep_acked,

  ////////////////////
  // in endpoint interface
  ////////////////////
  output spi_csel,
  output spi_clk,
  output spi_mosi,
  input spi_miso,

  ////////////////////
  // DFU State and debug
  ////////////////////
  output [7:0] dfu_state,
  output [7:0] debug,
);

  // Get flash partition information
  `include "boardinfo.vh"

  localparam IDLE = 0;
  localparam SETUP_IN = 1;
  localparam SETUP_DONE = 2;
  localparam DATA_IN = 3;
  localparam DATA_OUT = 4;
  localparam STATUS_IN = 5;
  localparam STATUS_OUT = 6;

  // DFU constants and states.
  localparam DFU_STATUS_OK = 'h00;
  localparam DFU_STATUS_ERR_TARGET = 'h01;
  localparam DFU_STATUS_ERR_FILE = 'h02;
  localparam DFU_STATUS_ERR_WRITE = 'h03;
  localparam DFU_STATUS_ERR_ERASE = 'h04;
  localparam DFU_STATUS_ERR_CHECK_ERASED = 'h05;
  localparam DFU_STATUS_ERR_PROG = 'h06;
  localparam DFU_STATUS_ERR_VERIFY = 'h07;
  localparam DFU_STATUS_ERR_ADDRESS = 'h08;
  localparam DFU_STATUS_ERR_NOTDONE = 'h09;
  localparam DFU_STATUS_ERR_FIRMWARE = 'h0a;
  localparam DFU_STATUS_ERR_VENDOR = 'h0b;
  localparam DFU_STATUS_ERR_USBR = 'h0c;
  localparam DFU_STATUS_ERR_POR = 'h0d;
  localparam DFU_STATUS_ERR_UNKNOWN = 'h0e;
  localparam DFU_STATUS_ERR_STALLEDPKT = 'h0f;

  localparam DFU_STATE_appIDLE = 'h00;
  localparam DFU_STATE_appDETACH = 'h01;
  localparam DFU_STATE_dfuIDLE = 'h02;
  localparam DFU_STATE_dfuDNLOAD_SYNC = 'h03;
  localparam DFU_STATE_dfuDNBUSY = 'h04;
  localparam DFU_STATE_dfuDNLOAD_IDLE = 'h05;
  localparam DFU_STATE_dfuMANIFEST_SYNC = 'h06;
  localparam DFU_STATE_dfuMANIFEST = 'h07;
  localparam DFU_STATE_dfuMANIFEST_WAIT_RESET = 'h08;
  localparam DFU_STATE_dfuUPLOAD_IDLE = 'h09;
  localparam DFU_STATE_dfuERROR = 'h0a;

  localparam ALT_MODE_USERPART = 'h00;
  localparam ALT_MODE_DATAPART = 'h01;
  localparam ALT_MODE_BOOTPART = 'h02;

  reg [5:0] ctrl_xfr_state = IDLE;
  reg [5:0] ctrl_xfr_state_next;

  reg setup_stage_end = 0;
  reg data_stage_end = 0;
  reg status_stage_end = 0;
  reg send_zero_length_data_pkt = 0;

  // the default control endpoint gets assigned the device address
  reg [6:0] dev_addr_i = 0;
  assign dev_addr = dev_addr_i;

  assign out_ep_req = out_ep_data_avail;
  assign out_ep_data_get = (ctrl_xfr_state == DATA_OUT && rom_mux == ROM_FIRMWARE) ? dfu_spi_wr_data_get : out_ep_data_avail;
  reg out_ep_data_valid = 0;
  always @(posedge clk) out_ep_data_valid <= out_ep_data_avail && out_ep_grant;

  // need to record the setup data
  reg [3:0] setup_data_addr = 0;
  reg [9:0] raw_setup_data [7:0];

  wire [7:0] bmRequestType = raw_setup_data[0];
  wire [7:0] bRequest = raw_setup_data[1];
  wire [15:0] wValue = {raw_setup_data[3][7:0], raw_setup_data[2][7:0]};
  wire [15:0] wIndex = {raw_setup_data[5][7:0], raw_setup_data[4][7:0]};
  wire [15:0] wLength = {raw_setup_data[7][7:0], raw_setup_data[6][7:0]};

  // keep track of new out data start and end
  wire pkt_start;
  wire pkt_end;

  rising_edge_detector detect_pkt_start (
    .clk(clk),
    .in(out_ep_data_avail),
    .out(pkt_start)
  );

  falling_edge_detector detect_pkt_end (
    .clk(clk),
    .in(out_ep_data_avail),
    .out(pkt_end)
  );

  assign out_ep_stall = 1'b0;

  wire setup_pkt_start = pkt_start && out_ep_setup;

  wire has_data_stage = wLength != 16'b0000000000000000; // this version for some reason causes a 16b carry which is slow
  //wire has_data_stage = |wLength;

  wire out_data_stage;
  assign out_data_stage = has_data_stage && !bmRequestType[7];

  wire in_data_stage;
  assign in_data_stage = has_data_stage && bmRequestType[7];

  reg [15:0] rom_length = 0;
  reg [15:0] data_length = 0;

  wire all_data_sent = (ctrl_xfr_state == DATA_IN) && ((rom_length == 16'b0) || (data_length == 16'b0));
  wire more_data_to_send = !all_data_sent;

  wire in_data_transfer_done;
  rising_edge_detector detect_in_data_transfer_done (
    .clk(clk),
    .in(all_data_sent),
    .out(in_data_transfer_done)
  );

  assign in_ep_data_done = (in_data_transfer_done && ctrl_xfr_state == DATA_IN) || send_zero_length_data_pkt;

  assign in_ep_req = ctrl_xfr_state == DATA_IN && more_data_to_send;
  assign in_ep_data_put = (rom_mux != ROM_FIRMWARE) ? rom_data_put : dfu_spi_rd_data_put;

  localparam ROM_ENDPOINT = 0;
  localparam ROM_STRING = 1;
  localparam ROM_DFUSTATE = 2;
  localparam ROM_FIRMWARE = 3;

  reg [11:0] rom_addr = 0;
  reg [3:0] rom_mux = ROM_ENDPOINT;
  wire rom_data_put;
  assign rom_data_put = (ctrl_xfr_state == DATA_IN && more_data_to_send) && in_ep_data_free;

  // Select the flash partition based on altsetting.
  reg [15:0] dfu_part_start;
  reg [15:0] dfu_part_size;
  always @(*) begin
    case (dfu_altsetting)
      ALT_MODE_USERPART : begin
        dfu_part_start <= USERPART_START / SPI_PAGE_SIZE;
        dfu_part_size  <= USERPART_SIZE / SPI_PAGE_SIZE;
      end
      ALT_MODE_DATAPART : begin
        dfu_part_start <= DATAPART_START / SPI_PAGE_SIZE;
        dfu_part_size  <= DATAPART_SIZE / SPI_PAGE_SIZE;
      end
      ALT_MODE_BOOTPART : begin
        dfu_part_start <= BOOTPART_START / SPI_PAGE_SIZE;
        dfu_part_size  <= BOOTPART_SIZE / SPI_PAGE_SIZE;
      end
      default : begin
        dfu_part_start <= 0;
        dfu_part_size  <= 0;
      end
    endcase
  end

  assign dfu_state = dfu_mem['h004];
  reg [15:0] dfu_altsetting = 0;
  reg [15:0] dfu_block_offset = 0;
  reg [15:0] dfu_block_addr = 0;
  reg [15:0] dfu_block_end = 0;

  wire [7:0] dfu_spi_rd_data;
  wire dfu_spi_rd_data_put;
  wire [7:0] dfu_spi_wr_data;
  wire dfu_spi_wr_data_get;
  wire dfu_spi_wr_busy;
  wire dfu_spi_wr_done;
  falling_edge_detector detect_wr_busy_done (
    .clk(clk),
    .in(dfu_spi_wr_busy),
    .out(dfu_spi_wr_done)
  );

  wire [7:0] dfu_debug;
  assign debug[0] = dfu_debug[0];
  assign debug[1] = dfu_block_addr[0];
  assign debug[2] = dfu_debug[1];
  assign debug[3] = out_ep_acked;

  usb_spiflash_bridge #(
    .PAGE_SIZE(SPI_PAGE_SIZE)
  ) dfu_spiflash_bridge (
    .clk(clk),
    .reset(reset),

    .spi_csel(spi_csel),
    .spi_clk(spi_clk),
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),

    .address(dfu_block_addr),

    .rd_request(ctrl_xfr_state == DATA_IN && rom_mux == ROM_FIRMWARE),
    .rd_data_free(more_data_to_send && in_ep_data_free),
    .rd_data_put(dfu_spi_rd_data_put),
    .rd_data(dfu_spi_rd_data),

    .wr_request(ctrl_xfr_state == DATA_OUT && rom_mux == ROM_FIRMWARE),
    .wr_busy(dfu_spi_wr_busy),
    .wr_data_avail(out_ep_data_avail),
    .wr_data_get(dfu_spi_wr_data_get),
    .wr_data(out_ep_data),

    .debug(dfu_debug)
  );

  reg save_dev_addr = 0;
  reg [6:0] new_dev_addr = 0;

  ////////////////////////////////////////////////////////////////////////////////
  // control transfer state machine
  ////////////////////////////////////////////////////////////////////////////////


  always @* begin
    setup_stage_end <= 0;
    data_stage_end <= 0;
    status_stage_end <= 0;
    send_zero_length_data_pkt <= 0;

    case (ctrl_xfr_state)
      IDLE : begin
        if (setup_pkt_start) begin
          ctrl_xfr_state_next <= SETUP_IN;
        end else begin
          ctrl_xfr_state_next <= IDLE;
        end
      end

      SETUP_IN : begin
        if (pkt_end) begin
          ctrl_xfr_state_next <= SETUP_DONE;
        end else begin
          ctrl_xfr_state_next <= SETUP_IN;
        end
      end

      SETUP_DONE : begin
        setup_stage_end <= 1;
        if (in_data_stage) begin
          ctrl_xfr_state_next <= DATA_IN;

        end else if (out_data_stage) begin
          ctrl_xfr_state_next <= DATA_OUT;

        end else begin
          ctrl_xfr_state_next <= STATUS_IN;
          send_zero_length_data_pkt <= 1;
        end
      end

      DATA_IN : begin
        if (in_ep_stall) begin
          ctrl_xfr_state_next <= IDLE;
          data_stage_end <= 1;
          status_stage_end <= 1;

        end else if (in_ep_acked && all_data_sent) begin
          ctrl_xfr_state_next <= STATUS_OUT;
          data_stage_end <= 1;

        end else begin
          ctrl_xfr_state_next <= DATA_IN;
        end
      end

      DATA_OUT : begin
        if (!data_length) begin
          ctrl_xfr_state_next <= STATUS_IN;
          send_zero_length_data_pkt <= 1;
          data_stage_end <= 1;

        end else begin
          ctrl_xfr_state_next <= DATA_OUT;
        end
      end

      STATUS_IN : begin
        if (in_ep_acked) begin
          ctrl_xfr_state_next <= IDLE;
          status_stage_end <= 1;

        end else begin
          ctrl_xfr_state_next <= STATUS_IN;
        end
      end

      STATUS_OUT: begin
        if (out_ep_acked) begin
          ctrl_xfr_state_next <= IDLE;
          status_stage_end <= 1;

        end else begin
          ctrl_xfr_state_next <= STATUS_OUT;
        end
      end

      default begin
        ctrl_xfr_state_next <= IDLE;
      end
    endcase
  end

  always @(posedge clk) begin
    if (reset) begin
      ctrl_xfr_state <= IDLE;
    end else begin
      ctrl_xfr_state <= ctrl_xfr_state_next;
    end
  end

  always @(posedge clk) begin
    in_ep_stall <= 0;

    if (out_ep_setup && out_ep_data_valid) begin
      raw_setup_data[setup_data_addr] <= out_ep_data;
      setup_data_addr <= setup_data_addr + 1;
    end

    // Handle DFU state transitions.
    if (dfu_spi_wr_done) begin
      if (dfu_mem['h004] == DFU_STATE_dfuDNBUSY) dfu_mem['h004] <= DFU_STATE_dfuDNLOAD_SYNC;
    end

    if (setup_stage_end) begin
      data_length <= wLength;
      
      // Standard Requests
      case ({bmRequestType[6:5], bRequest})
        'h006 : begin
          // GET_DESCRIPTOR
          case (wValue[15:8])
            1 : begin
              // DEVICE
              rom_mux     <= ROM_ENDPOINT;
              rom_addr    <= 'h00;
              rom_length  <= ep_rom['h00]; // bLength
            end

            2 : begin
              // CONFIGURATION
              rom_mux     <= ROM_ENDPOINT;
              rom_addr    <= 'h12;
              rom_length  <= ep_rom['h12 + 2]; // wTotalLength
            end

            3 : begin
              // STRING
              if (wValue[7:0] == 0) begin
                // Language descriptors
                rom_mux     <= ROM_ENDPOINT;
                rom_addr    <= 'h3F;
                rom_length  <= ep_rom['h3F]; // bLength
              end else begin
                rom_mux     <= ROM_STRING;
                rom_addr    <= (wValue[7:0] - 1) << 5;
                rom_length  <= str_rom[(wValue[7:0] - 1) << 5];
              end
            end

            6 : begin
              // DEVICE_QUALIFIER
              in_ep_stall <= 1;
              rom_mux    <= ROM_ENDPOINT;
              rom_addr   <= 'h00;
              rom_length <= 'h00;
            end

          endcase
        end

        'h005 : begin
          // SET_ADDRESS
          rom_mux    <= ROM_ENDPOINT;
          rom_addr   <= 'h00;
          rom_length <= 'h00;

          // we need to save the address after the status stage ends
          // this is because the status stage token will still be using
          // the old device address
          save_dev_addr <= 1;
          new_dev_addr <= wValue[6:0];
        end

        'h009 : begin
          // SET_CONFIGURATION
          rom_mux    <= ROM_ENDPOINT;
          rom_addr   <= 'h00;
          rom_length <= 'h00;
        end

        'h0b : begin
          // SET_INTERFACE
          rom_mux    <= ROM_ENDPOINT;
          rom_addr   <= 'h00;
          rom_length <= 'h00;

          // Select the desired alt mode, possibly adjusting the flash region.
          dfu_altsetting <= wValue;
        end

        'h101 : begin
          // DFU_DNLOAD
          if (dfu_mem['h004] == DFU_STATE_dfuIDLE) begin
            // Starting a new download.
            rom_mux    <= ROM_FIRMWARE;
            rom_addr   <= 0;
            rom_length <= wLength;

            dfu_block_offset <= dfu_part_start - wValue;
            dfu_block_end   <= wValue + dfu_part_size;
            dfu_block_addr  <= dfu_part_start;
            dfu_mem['h004] <= DFU_STATE_dfuDNBUSY;
          end else if (dfu_mem['h004] != DFU_STATE_dfuDNLOAD_IDLE) begin
            // We are not ready to receive another block
            rom_mux    <= ROM_ENDPOINT;
            rom_addr   <= 'h00;
            rom_length <= 'h00;
            dfu_mem['h000] <= DFU_STATUS_ERR_WRITE;
            dfu_mem['h004] <= DFU_STATE_dfuERROR;
          end else if (wValue >= dfu_block_end) begin
            rom_mux    <= ROM_ENDPOINT;
            rom_addr   <= 'h00;
            rom_length <= 'h00;
            dfu_mem['h000] <= DFU_STATUS_ERR_ADDRESS;
            dfu_mem['h004] <= DFU_STATE_dfuERROR;
          end else if (wLength) begin
            // Download the next block
            rom_mux    <= ROM_FIRMWARE;
            rom_addr   <= 0;
            rom_length <= wLength;
            dfu_block_addr <= (wValue + dfu_block_offset);
            dfu_mem['h004] <= DFU_STATE_dfuDNBUSY;
          end else begin
            // Download the final (zero length) block
            rom_mux    <= ROM_ENDPOINT;
            rom_addr   <= 'h00;
            rom_length <= 'h00;
            dfu_mem['h000] <= DFU_STATUS_OK;
            dfu_mem['h004] <= DFU_STATE_dfuMANIFEST_SYNC;
          end
        end

        'h102 : begin
          // DFU_UPLOAD
          if (dfu_mem['h004] != DFU_STATE_dfuUPLOAD_IDLE) begin
            rom_mux    <= ROM_FIRMWARE;
            rom_addr   <= 0;
            rom_length <= wLength;

            // Switch to the dfuUPLOAD-IDLE state.
            dfu_block_offset <= dfu_part_start - wValue;
            dfu_block_end   <= wValue + dfu_part_size;
            dfu_block_addr  <= dfu_part_start;
            dfu_mem['h004] <= DFU_STATE_dfuUPLOAD_IDLE;
          end else if (wValue >= dfu_block_end) begin
            rom_mux    <= ROM_ENDPOINT;
            rom_addr   <= 0;
            rom_length <= 0;
          end else begin
            rom_mux    <= ROM_FIRMWARE;
            rom_addr   <= 0;
            rom_length <= wLength;
            dfu_block_addr <= (wValue + dfu_block_offset);
          end
        end

        'h103 : begin
          // DFU_GETSTATUS
          rom_mux    <= ROM_DFUSTATE;
          rom_addr   <= 'h00;
          rom_length <= 6;
          if (dfu_mem['h004] == DFU_STATE_dfuDNLOAD_SYNC) dfu_mem['h004] <= DFU_STATE_dfuDNLOAD_IDLE;
          if (dfu_mem['h004] == DFU_STATE_dfuMANIFEST_SYNC) dfu_mem['h004] <= DFU_STATE_dfuIDLE;
        end

        'h104 : begin
          // DFU_CLRSTATUS
          rom_mux    <= ROM_ENDPOINT;
          rom_addr   <= 'h00;
          rom_length <= 0;
          dfu_mem['h000] <= DFU_STATUS_OK;
          dfu_mem['h004] <= DFU_STATE_dfuIDLE;
        end

        'h105 : begin
          // DFU_GETSTATE
          rom_mux    <= ROM_DFUSTATE;
          rom_addr   <= 'h04;
          rom_length <= 1;
        end

        'h106 : begin
          // DFU_ABORT
          rom_mux    <= ROM_DFUSTATE;
          rom_addr   <= 'h00;
          rom_length <= 0;

          // Return to the dfuIDLE state.
          dfu_mem['h004] <= DFU_STATE_dfuIDLE;
        end 

        default begin
          rom_mux    <= ROM_ENDPOINT;
          rom_addr   <= 'h00;
          rom_length <= 'h00;
        end
      endcase
    end

    if (in_ep_grant && in_ep_data_put) begin
      rom_addr <= rom_addr + 1;
      rom_length <= rom_length - 1;
      data_length <= data_length - 1;
    end
    if (!out_ep_setup && out_ep_grant && out_ep_data_get) begin
      data_length <= data_length - 1;
    end

    if (status_stage_end) begin
      setup_data_addr <= 0;
      rom_addr <= 0;
      rom_length <= 0;

      if (save_dev_addr) begin
        save_dev_addr <= 0;
        dev_addr_i <= new_dev_addr;
      end
    end

    if (reset) begin
      dev_addr_i <= 0;
      setup_data_addr <= 0;
      save_dev_addr <= 0;
    end
  end

  reg [7:0] ep_rom[255:0];
  reg [7:0] str_rom[255:0];
  reg [7:0] dfu_mem[5:0];

  // Mux the data being read
  always @* begin
    case (rom_mux)
      ROM_ENDPOINT : in_ep_data <= ep_rom[rom_addr];
      ROM_STRING   : in_ep_data <= str_rom[rom_addr];
      ROM_DFUSTATE : in_ep_data <= dfu_mem[rom_addr];
      ROM_FIRMWARE : in_ep_data <= dfu_spi_rd_data;
      default      : in_ep_data <= 8'b0;
    endcase
  end

  initial begin
      // device descriptor
      ep_rom['h000] <= 18; // bLength
      ep_rom['h001] <= 1; // bDescriptorType
      ep_rom['h002] <= 'h00; // bcdUSB[0]
      ep_rom['h003] <= 'h01; // bcdUSB[1]
      ep_rom['h004] <= 'h00; // bDeviceClass
      ep_rom['h005] <= 'h00; // bDeviceSubClass
      ep_rom['h006] <= 'h00; // bDeviceProtocol
      ep_rom['h007] <= MAX_OUT_PACKET_SIZE; // bMaxPacketSize0

      ep_rom['h008] <= BOARD_VID >> 0; // idVendor[0]
      ep_rom['h009] <= BOARD_VID >> 8; // idVendor[1]
      ep_rom['h00A] <= BOARD_PID >> 0; // idProduct[0]
      ep_rom['h00B] <= BOARD_PID >> 8; // idProduct[1]

      ep_rom['h00C] <= 0; // bcdDevice[0]
      ep_rom['h00D] <= 0; // bcdDevice[1]
      ep_rom['h00E] <= 1; // iManufacturer
      ep_rom['h00F] <= 2; // iProduct
      ep_rom['h010] <= 3; // iSerialNumber
      ep_rom['h011] <= 1; // bNumConfigurations

      // configuration descriptor
      ep_rom['h012] <= 9; // bLength
      ep_rom['h013] <= 2; // bDescriptorType
      ep_rom['h014] <= (9+9+9+9+9); // wTotalLength[0] FIXME!!!
      ep_rom['h015] <= 0; // wTotalLength[1]
      ep_rom['h016] <= 1; // bNumInterfaces
      ep_rom['h017] <= 1; // bConfigurationValue
      ep_rom['h018] <= 0; // iConfiguration
      ep_rom['h019] <= 'hC0; // bmAttributes
      ep_rom['h01A] <= 50; // bMaxPower
      
      // interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
      // bootloader partition
      ep_rom['h01B] <= 9; // bLength
      ep_rom['h01C] <= 4; // bDescriptorType
      ep_rom['h01D] <= 0; // bInterfaceNumber
      ep_rom['h01E] <= ALT_MODE_USERPART; // bAlternateSetting
      ep_rom['h01F] <= 0; // bNumEndpoints
      ep_rom['h020] <= 'hFE; // bInterfaceClass (Application Specific Class Code)
      ep_rom['h021] <= 1; // bInterfaceSubClass (Device Firmware Upgrade Code)
      ep_rom['h022] <= 2; // bInterfaceProtocol (DFU mode protocol)
      ep_rom['h023] <= 4; // iInterface
      
      // interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
      // user image partition
      ep_rom['h024] <= 9; // bLength
      ep_rom['h025] <= 4; // bDescriptorType
      ep_rom['h026] <= 0; // bInterfaceNumber
      ep_rom['h027] <= ALT_MODE_DATAPART; // bAlternateSetting
      ep_rom['h028] <= 0; // bNumEndpoints
      ep_rom['h029] <= 'hFE; // bInterfaceClass (Application Specific Class Code)
      ep_rom['h02A] <= 1; // bInterfaceSubClass (Device Firmware Upgrade Code)
      ep_rom['h02B] <= 2; // bInterfaceProtocol (DFU mode protocol)
      ep_rom['h02C] <= 5; // iInterface

      // interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
      // data partition
      ep_rom['h02D] <= 9; // bLength
      ep_rom['h02E] <= 4; // bDescriptorType
      ep_rom['h02F] <= 0; // bInterfaceNumber
      ep_rom['h030] <= ALT_MODE_BOOTPART; // bAlternateSetting
      ep_rom['h031] <= 0; // bNumEndpoints
      ep_rom['h032] <= 'hFE; // bInterfaceClass (Application Specific Class Code)
      ep_rom['h033] <= 1; // bInterfaceSubClass (Device Firmware Upgrade Code)
      ep_rom['h034] <= 2; // bInterfaceProtocol (DFU mode protocol)
      ep_rom['h035] <= 6; // iInterface

      // DFU Header Functional Descriptor, DFU Spec 4.1.3, Table 4.2
      ep_rom['h036] <= 9;           // bFunctionLength
      ep_rom['h037] <= 'h21;        // bDescriptorType
      ep_rom['h038] <= 'h0f;				// bmAttributes
      ep_rom['h039] <= 255;         // wDetachTimeout[0]
      ep_rom['h03A] <= 0;           // wDetachTimeout[1]
      ep_rom['h03B] <= SPI_PAGE_SIZE >> 0; // wTransferSize[0]
      ep_rom['h03C] <= SPI_PAGE_SIZE >> 8; // wTransferSize[1]
      ep_rom['h03D] <= 'h10;        // bcdDFUVersion[0]
      ep_rom['h03E] <= 'h01;        // bcdDFUVersion[1]

      // Language string descriptor is at string index zero.
      ep_rom['h03F] <= 4;     // bLength
      ep_rom['h040] <= 3;     // bDescriptorType == STRING
      ep_rom['h041] <= 'h09;  // wLANGID[0] == US English
      ep_rom['h042] <= 'h04;  // wLANGID[1]

      // String descriptors are allocated 32B for each descriptor.
      str_rom['h000] <= 16;  // bLength
      str_rom['h001] <= 3;   // bDescriptorType == STRING
      str_rom['h002] <= "L";  str_rom['h003] <= 0;
      str_rom['h004] <= "a";  str_rom['h005] <= 0;
      str_rom['h006] <= "t";  str_rom['h007] <= 0;
      str_rom['h008] <= "t";  str_rom['h009] <= 0;
      str_rom['h00A] <= "i";  str_rom['h00B] <= 0;
      str_rom['h00C] <= "c";  str_rom['h00D] <= 0;
      str_rom['h00E] <= "e";  str_rom['h00F] <= 0;

      str_rom['h020] <= 26;  // bLength
      str_rom['h021] <= 3;   // bDescriptorType == STRING
      str_rom['h022] <= "T";  str_rom['h023] <= 0;
      str_rom['h024] <= "i";  str_rom['h025] <= 0;
      str_rom['h026] <= "n";  str_rom['h027] <= 0;
      str_rom['h028] <= "y";  str_rom['h029] <= 0;
      str_rom['h02A] <= "D";  str_rom['h02B] <= 0;
      str_rom['h02C] <= "F";  str_rom['h02D] <= 0;
      str_rom['h02E] <= "U";  str_rom['h02F] <= 0;
      str_rom['h030] <= " ";  str_rom['h031] <= 0;
      str_rom['h032] <= "B";  str_rom['h033] <= 0;
      str_rom['h034] <= "o";  str_rom['h035] <= 0;
      str_rom['h036] <= "o";  str_rom['h037] <= 0;
      str_rom['h038] <= "t";  str_rom['h039] <= 0;

      str_rom['h040] <= 14;  // bLength
      str_rom['h041] <= 3;   // bDescriptorType == STRING
      str_rom['h042] <= "1";  str_rom['h043] <= 0;
      str_rom['h044] <= "2";  str_rom['h045] <= 0;
      str_rom['h046] <= "3";  str_rom['h047] <= 0;
      str_rom['h048] <= "4";  str_rom['h049] <= 0;
      str_rom['h04A] <= "5";  str_rom['h04B] <= 0;
      str_rom['h04C] <= "6";  str_rom['h04D] <= 0;

      str_rom['h060] <= 22;  // bLength
      str_rom['h061] <= 3;   // bDescriptorType == STRING
      str_rom['h062] <= "U"; str_rom['h063] <= 0;
      str_rom['h064] <= "s"; str_rom['h065] <= 0;
      str_rom['h066] <= "e"; str_rom['h067] <= 0;
      str_rom['h068] <= "r"; str_rom['h069] <= 0;
      str_rom['h06A] <= " "; str_rom['h06B] <= 0;
      str_rom['h06C] <= "I"; str_rom['h06D] <= 0;
      str_rom['h06E] <= "m"; str_rom['h06F] <= 0;
      str_rom['h070] <= "a"; str_rom['h071] <= 0;
      str_rom['h072] <= "g"; str_rom['h073] <= 0;
      str_rom['h074] <= "e"; str_rom['h075] <= 0;

      str_rom['h080] <= 20;  // bLength
      str_rom['h081] <= 3;   // bDescriptorType == STRING
      str_rom['h082] <= "U"; str_rom['h083] <= 0;
      str_rom['h084] <= "s"; str_rom['h085] <= 0;
      str_rom['h086] <= "e"; str_rom['h087] <= 0;
      str_rom['h088] <= "r"; str_rom['h089] <= 0;
      str_rom['h08A] <= " "; str_rom['h08B] <= 0;
      str_rom['h08C] <= "D"; str_rom['h08D] <= 0;
      str_rom['h08E] <= "a"; str_rom['h08F] <= 0;
      str_rom['h090] <= "t"; str_rom['h091] <= 0;
      str_rom['h092] <= "a"; str_rom['h093] <= 0;

      str_rom['h0A0] <= 22;  // bLength
      str_rom['h0A1] <= 3;   // bDescriptorType == STRING
      str_rom['h0A2] <= "B"; str_rom['h0A3] <= 0;
      str_rom['h0A4] <= "o"; str_rom['h0A5] <= 0;
      str_rom['h0A6] <= "o"; str_rom['h0A7] <= 0;
      str_rom['h0A8] <= "t"; str_rom['h0A9] <= 0;
      str_rom['h0AA] <= "l"; str_rom['h0AB] <= 0;
      str_rom['h0AC] <= "o"; str_rom['h0AD] <= 0;
      str_rom['h0AE] <= "a"; str_rom['h0AF] <= 0;
      str_rom['h0B0] <= "d"; str_rom['h0B1] <= 0;
      str_rom['h0B2] <= "e"; str_rom['h0B3] <= 0;
      str_rom['h0B4] <= "r"; str_rom['h0B5] <= 0;

      // DFU State data
      dfu_mem['h000] <= DFU_STATUS_OK;      // bStatus
      dfu_mem['h001] <= 1;                  // bwPollTimeout[0]
      dfu_mem['h002] <= 0;                  // bwPollTimeout[1]
      dfu_mem['h003] <= 0;                  // bwPollTimeout[2]
      dfu_mem['h004] <= DFU_STATE_dfuIDLE;  // bState
      dfu_mem['h005] <= 0;                  // iString
  end

endmodule
