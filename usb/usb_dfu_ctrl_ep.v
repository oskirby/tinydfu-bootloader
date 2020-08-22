module usb_dfu_ctrl_ep #(
  parameter MAX_IN_PACKET_SIZE = 32,
  parameter MAX_OUT_PACKET_SIZE = 32
) (
  input clk,
  input clk_48mhz,
  input reset,

  input usb_reset,
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
  output reg dfu_detach = 0,
  output [7:0] dfu_state,
  output [7:0] debug
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

  localparam DFU_DETACH_TIMEOUT = 1000;

  localparam ALT_MODE_USERPART = 'h00;
  localparam ALT_MODE_DATAPART = 'h01;
  localparam ALT_MODE_BOOTPART = 'h02;
  localparam ALT_MODE_SECURITY = 'h03;
  localparam ALT_MODE_MAX = (ALT_MODE_SECURITY + SPI_SECURITY_REGISTERS);

  localparam STR_INDEX_MANUFACTURER = 1;
  localparam STR_INDEX_PRODUCT = 2;
  localparam STR_INDEX_SERIAL = 3;
  localparam STR_INDEX_PARTITIONS = 4;

  localparam USERPART_NAME = "User Image";
  localparam DATAPART_NAME = "User Data";
  localparam BOOTPART_NAME = "Bootloader";

  localparam TEST_SERIAL = BOARD_SERIAL;

  function [8:0] str_addr;
    input [7:0] index;
    str_addr = (index - 1) * 9'h020;
  endfunction

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

  //wire has_data_stage = wLength != 16'b0000000000000000; // this version for some reason causes a 16b carry which is slow
  wire has_data_stage = |wLength;

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
  assign in_ep_data_put = (rom_mux == ROM_FIRMWARE) ? dfu_spi_rd_data_put : rom_data_put;

  localparam ROM_ENDPOINT = 0;
  localparam ROM_CONFIG = 1;
  localparam ROM_STRING = 2;
  localparam ROM_DFUSTATE = 3;
  localparam ROM_FIRMWARE = 4;

  reg [8:0] rom_addr = 0;
  reg [3:0] rom_mux = ROM_ENDPOINT;
  wire rom_data_put;
  assign rom_data_put = (ctrl_xfr_state == DATA_IN && more_data_to_send) && in_ep_data_free;

  reg save_dev_addr = 0;
  reg [6:0] new_dev_addr = 0;

  // Select the flash partition based on altsetting.
  wire       dfu_part_security = (dfu_altsetting >= ALT_MODE_SECURITY);
  reg [15:0] dfu_part_start[ALT_MODE_MAX-1:0];
  reg [15:0] dfu_part_size[ALT_MODE_MAX-1:0];

  initial begin
    dfu_part_start[ALT_MODE_USERPART] <= USERPART_START / SPI_PAGE_SIZE;
    dfu_part_size[ALT_MODE_USERPART]  <= USERPART_SIZE / SPI_PAGE_SIZE;

    dfu_part_start[ALT_MODE_DATAPART] <= DATAPART_START / SPI_PAGE_SIZE;
    dfu_part_size[ALT_MODE_DATAPART]  <= DATAPART_SIZE / SPI_PAGE_SIZE;

    dfu_part_start[ALT_MODE_BOOTPART] <= BOOTPART_START / SPI_PAGE_SIZE;
    dfu_part_size[ALT_MODE_BOOTPART]  <= BOOTPART_SIZE / SPI_PAGE_SIZE;
  end
  
  genvar alt_part;
  generate
    for (alt_part = ALT_MODE_SECURITY; alt_part < ALT_MODE_MAX; alt_part = alt_part + 1) begin
      initial begin
        dfu_part_start[alt_part] <= (alt_part - ALT_MODE_SECURITY + 1) << (SPI_SECURITY_REG_SHIFT - $clog2(SPI_PAGE_SIZE));
        dfu_part_size[alt_part]  <= 1;
      end
    end
  endgenerate
  
  ////////////////////////////////////////////////////////////////////////////////
  // DFU SPI Flash Interface
  ////////////////////////////////////////////////////////////////////////////////
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
    .ERASE_SIZE(SPI_ERASE_SIZE),
    .PAGE_SIZE(SPI_PAGE_SIZE)
  ) dfu_spiflash_bridge (
    .clk(clk),
    .reset(reset),

    .spi_csel(spi_csel),
    .spi_clk(spi_clk),
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),

    .address(dfu_block_addr),
    .security(dfu_part_security),

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

  ////////////////////////////////////////////////////////////////////////////////
  // DFU Detach Timer
  ////////////////////////////////////////////////////////////////////////////////
  // Prescale the 48MHz clock down to 3Mhz.
  // Generate a 1ms clock by dividing the 48Mhz high speed clock.
  reg [14:0] dfu_msec_counter = 0;
  reg dfu_msec_clk = 0;
  always @(posedge clk_48mhz) begin
    if (dfu_msec_counter) dfu_msec_counter <= dfu_msec_counter - 1;
    else begin
      dfu_msec_clk <= ~dfu_msec_clk;
      dfu_msec_counter <= 23999; /* 0.5ms in 48MHz clocks */
    end
  end
  
  // The DFU specification suggests that a USB reset from the dfuIDLE state should
  // return the device to application run-time mode. However, as an extension we
  // have also support the DFU_DETACH command which starts the detach timer.
  reg [15:0] dfu_detach_timer = DFU_DETACH_TIMEOUT;
  always @(posedge dfu_msec_clk) begin
    if (usb_reset && dfu_mem['h004] != DFU_STATE_appIDLE) begin
      dfu_detach <= 1'b1;
    end
    if (dfu_mem['h004] == DFU_STATE_appDETACH) begin
        if (dfu_detach_timer) dfu_detach_timer <= dfu_detach_timer - 1;
        else dfu_detach <= 1'b1;
    end
  end

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
              rom_mux     <= ROM_CONFIG;
              rom_addr    <= 'h00;
              rom_length  <= cfg_rom['h00 + 2]; // wTotalLength
            end

            3 : begin
              // STRING
              rom_mux <= ROM_STRING;
              rom_addr <= 'h00;
              rom_length <= str_rom_length;
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

          // Consider DFU mode started once we have become configured.
          dfu_mem['h004] <= (wValue) ? DFU_STATE_dfuIDLE : DFU_STATE_appIDLE;
        end

        'h0b : begin
          // SET_INTERFACE
          rom_mux    <= ROM_ENDPOINT;
          rom_addr   <= 'h00;
          rom_length <= 'h00;

          // Select the desired alt mode, possibly adjusting the flash region.
          dfu_altsetting <= wValue;
        end

        'h100 : begin
          // DFU_DETACH
          rom_mux    <= ROM_FIRMWARE;
          rom_addr   <= 0;
          rom_length <= wLength;
          
          dfu_mem['h004] <= DFU_STATE_appDETACH;
        end

        'h101 : begin
          // DFU_DNLOAD
          if (dfu_mem['h004] == DFU_STATE_dfuIDLE) begin
            // Starting a new download.
            rom_mux    <= ROM_FIRMWARE;
            rom_addr   <= 0;
            rom_length <= wLength;

            dfu_block_offset <= dfu_part_start[dfu_altsetting] - wValue;
            dfu_block_end   <= wValue + dfu_part_size[dfu_altsetting];
            dfu_block_addr  <= dfu_part_start[dfu_altsetting];
            dfu_mem['h004] <= DFU_STATE_dfuDNBUSY;
          end else if (dfu_mem['h004] != DFU_STATE_dfuDNLOAD_IDLE) begin
            // We are not ready to receive another block
            rom_mux    <= ROM_ENDPOINT;
            rom_addr   <= 'h00;
            rom_length <= 'h00;
            dfu_mem['h000] <= DFU_STATUS_ERR_WRITE;
            dfu_mem['h004] <= DFU_STATE_dfuERROR;
          end else if (wLength == 0) begin
            // Download the final (zero length) block
            rom_mux    <= ROM_ENDPOINT;
            rom_addr   <= 'h00;
            rom_length <= 'h00;
            dfu_mem['h000] <= DFU_STATUS_OK;
            dfu_mem['h004] <= DFU_STATE_dfuMANIFEST_SYNC;
          end else if (wValue >= dfu_block_end) begin
            rom_mux    <= ROM_ENDPOINT;
            rom_addr   <= 'h00;
            rom_length <= 'h00;
            dfu_mem['h000] <= DFU_STATUS_ERR_ADDRESS;
            dfu_mem['h004] <= DFU_STATE_dfuERROR;
          end else begin
            // Download the next block
            rom_mux    <= ROM_FIRMWARE;
            rom_addr   <= 0;
            rom_length <= wLength;
            dfu_block_addr <= (wValue + dfu_block_offset);
            dfu_mem['h004] <= DFU_STATE_dfuDNBUSY;
          end
        end

        'h102 : begin
          // DFU_UPLOAD
          if (dfu_mem['h004] != DFU_STATE_dfuUPLOAD_IDLE) begin
            rom_mux    <= ROM_FIRMWARE;
            rom_addr   <= 0;
            rom_length <= wLength;

            // Switch to the dfuUPLOAD-IDLE state.
            dfu_block_offset <= dfu_part_start[dfu_altsetting] - wValue;
            dfu_block_end   <= wValue + dfu_part_size[dfu_altsetting];
            dfu_block_addr  <= dfu_part_start[dfu_altsetting];
            dfu_mem['h004] <= DFU_STATE_dfuUPLOAD_IDLE;
          end else if (wValue >= dfu_block_end) begin
            rom_mux    <= ROM_ENDPOINT;
            rom_addr   <= 0;
            rom_length <= 0;
            dfu_mem['h004] <= DFU_STATE_dfuIDLE;
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
  reg [7:0] cfg_rom[255:0];
  reg [7:0] dfu_mem[5:0];

  // Mux the data being read
  always @* begin
    case (rom_mux)
      ROM_ENDPOINT : in_ep_data <= ep_rom[rom_addr];
      ROM_CONFIG   : in_ep_data <= cfg_rom[rom_addr];
      ROM_STRING   : in_ep_data <= str_rom_data;
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
      ep_rom['h00E] <= STR_INDEX_MANUFACTURER;  // iManufacturer
      ep_rom['h00F] <= STR_INDEX_PRODUCT;       // iProduct
      ep_rom['h010] <= STR_INDEX_SERIAL;        // iSerialNumber
      ep_rom['h011] <= 1; // bNumConfigurations

      // Language string descriptor is at string index zero.
      ep_rom['h012] <= 4;     // bLength
      ep_rom['h013] <= 3;     // bDescriptorType == STRING
      ep_rom['h014] <= 'h09;  // wLANGID[0] == US English
      ep_rom['h015] <= 'h04;  // wLANGID[1]

      // configuration descriptor
      cfg_rom['h00] <= 9; // bLength
      cfg_rom['h01] <= 2; // bDescriptorType
      cfg_rom['h02] <= (9+9) + (ALT_MODE_MAX * 9); // wTotalLength[0]
      cfg_rom['h03] <= 0; // wTotalLength[1]
      cfg_rom['h04] <= 1; // bNumInterfaces
      cfg_rom['h05] <= 1; // bConfigurationValue
      cfg_rom['h06] <= 0; // iConfiguration
      cfg_rom['h07] <= 'hC0; // bmAttributes
      cfg_rom['h08] <= 50; // bMaxPower
      
      // DFU Header Functional Descriptor, DFU Spec 4.1.3, Table 4.2
      cfg_rom[(ALT_MODE_MAX * 9) + 'h09] <= 9;           // bFunctionLength
      cfg_rom[(ALT_MODE_MAX * 9) + 'h0A] <= 'h21;        // bDescriptorType
      cfg_rom[(ALT_MODE_MAX * 9) + 'h0B] <= 'h0f;        // bmAttributes
      cfg_rom[(ALT_MODE_MAX * 9) + 'h0C] <= DFU_DETACH_TIMEOUT >> 0; // wDetachTimeout[0]
      cfg_rom[(ALT_MODE_MAX * 9) + 'h0D] <= DFU_DETACH_TIMEOUT >> 8; // wDetachTimeout[1]
      cfg_rom[(ALT_MODE_MAX * 9) + 'h0E] <= SPI_PAGE_SIZE >> 0; // wTransferSize[0]
      cfg_rom[(ALT_MODE_MAX * 9) + 'h0F] <= SPI_PAGE_SIZE >> 8; // wTransferSize[1]
      cfg_rom[(ALT_MODE_MAX * 9) + 'h10] <= 'h10;        // bcdDFUVersion[0]
      cfg_rom[(ALT_MODE_MAX * 9) + 'h11] <= 'h01;        // bcdDFUVersion[1]

      // DFU State data
      dfu_mem['h000] <= DFU_STATUS_OK;      // bStatus
      dfu_mem['h001] <= 1;                  // bwPollTimeout[0]
      dfu_mem['h002] <= 0;                  // bwPollTimeout[1]
      dfu_mem['h003] <= 0;                  // bwPollTimeout[2]
      dfu_mem['h004] <= DFU_STATE_appIDLE;  // bState
      dfu_mem['h005] <= 0;                  // iString
  end

///////////////////////////////////////////////////////////
// Generate Configuration Descriptors for the Partitions
///////////////////////////////////////////////////////////
genvar alt_num;
generate
  for (alt_num = 0; alt_num < ALT_MODE_MAX; alt_num = alt_num + 1) begin
    initial begin
      cfg_rom[alt_num*9 + 'h09] <= 9;     // bLength
      cfg_rom[alt_num*9 + 'h0A] <= 4;     // bDescriptorType
      cfg_rom[alt_num*9 + 'h0B] <= 0;     // bInterfaceNumber
      cfg_rom[alt_num*9 + 'h0C] <= alt_num; // bAlternateSetting
      cfg_rom[alt_num*9 + 'h0D] <= 0;     // bNumEndpoints
      cfg_rom[alt_num*9 + 'h0E] <= 'hFE;  // bInterfaceClass (Application Specific Class Code)
      cfg_rom[alt_num*9 + 'h0F] <= 1;     // bInterfaceSubClass (Device Firmware Upgrade Code)
      cfg_rom[alt_num*9 + 'h10] <= 2;     // bInterfaceProtocol (DFU mode protocol)
      cfg_rom[alt_num*9 + 'h11] <= (STR_INDEX_PARTITIONS + alt_num); // iInterface
    end
  end
endgenerate

///////////////////////////////////////////////////////////
// Generate String ROMS for the Security Registers
///////////////////////////////////////////////////////////
wire [7:0] str_rom_data;
wire [7:0] str_rom_length;
usb_string_rom#(
  .STRINGS({
    BOARD_MFR_NAME, 8'h00,
    BOARD_PRODUCT_NAME, 8'h00,
    TEST_SERIAL, 8'h00,
    USERPART_NAME, 8'h00,
    DATAPART_NAME, 8'h00,
    BOOTPART_NAME, 8'h00,
    {(SPI_SECURITY_REGISTERS){"Security Reg", 8'h00}}
  })
) str_rom(
  .str_index(wValue[7:0]),
  .rom_addr(rom_addr),
  .rom_length(str_rom_length),
  .rom_data(str_rom_data)
);

endmodule
