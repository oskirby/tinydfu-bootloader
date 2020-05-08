module usb_dfu_ctrl_ep #(
  parameter MAX_IN_PACKET_SIZE = 32,
  parameter MAX_OUT_PACKET_SIZE = 32,
  parameter SPI_PAGE_SIZE = 4096,
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

  output [7:0] debug,
);


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
  localparam DFU_STATE_dfuDOWNLOAD_SYNC = 'h03;
  localparam DFU_STATE_dfuDNBUSY = 'h04;
  localparam DFU_STATE_dfuDNLOAD_IDLE = 'h05;
  localparam DFU_STATE_dfuMANIFEST_SYNC = 'h06;
  localparam DFU_STATE_dfuMANIFEST = 'h07;
  localparam DFU_STATE_dfuMANIFEST_WAIT_RESET = 'h08;
  localparam DFU_STATE_dfuUPLOAD_IDLE = 'h09;
  localparam DFU_STATE_dfuERROR = 'h0a;


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
  assign out_ep_data_get = out_ep_data_avail;
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

  // wire has_data_stage = wLength != 16'b0000000000000000; // this version for some reason causes a 16b carry which is slow
  wire has_data_stage = |wLength;

  wire out_data_stage;
  assign out_data_stage = has_data_stage && !bmRequestType[7];

  wire in_data_stage;
  assign in_data_stage = has_data_stage && bmRequestType[7];

  reg [15:0] rom_length = 0;
  reg [15:0] max_length = 0;

  wire all_data_sent = (rom_length == 16'b0) || (max_length == 16'b0);
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
  reg [7:0] rom_mux = ROM_ENDPOINT;
  wire rom_data_put;
  assign rom_data_put = (ctrl_xfr_state == DATA_IN && more_data_to_send) && in_ep_data_free;

  wire [7:0] dfu_state = dfu_mem['h004];
  reg [15:0] dfu_altsetting = 0;
  reg [15:0] dfu_block_start = 0;
  reg [15:0] dfu_page_addr = 0;
  reg [15:0] dfu_byte_addr = 0;

  wire [7:0] dfu_spi_rd_data;
  wire dfu_spi_rd_data_put;
  wire [7:0] dfu_spi_wr_data;

  usb_spiflash_bridge #(
    .SPI_PAGE_SIZE(SPI_PAGE_SIZE)
  ) dfu_spiflash_bridge (
    .clk(clk),
    .reset(reset),

    .spi_csel(spi_csel),
    .spi_clk(spi_clk),
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),

    .address(16'b0),

    .rd_request(ctrl_xfr_state == DATA_IN && rom_mux == ROM_FIRMWARE),
    .rd_data_free(more_data_to_send && in_ep_data_free),
    .rd_data_put(dfu_spi_rd_data_put),
    .rd_data(dfu_spi_rd_data),

    .debug(debug)
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
        if (out_ep_acked) begin
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

    if (setup_stage_end) begin
      max_length <= wLength;
      
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
                rom_addr    <= 'h2d;
                rom_length  <= ep_rom['h2d]; // bLength
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

        'h102 : begin
          // DFU_UPLOAD
          if (dfu_state == DFU_STATE_dfuUPLOAD_IDLE) begin
            rom_mux    <= ROM_FIRMWARE;
            rom_addr   <= 0;
            rom_length <= SPI_PAGE_SIZE;
          end else begin
            rom_mux    <= ROM_FIRMWARE;
            rom_addr   <= 0;
            rom_length <= SPI_PAGE_SIZE;

            // Switch to the dfuUPLOAD-IDLE state.
            dfu_block_start <= wValue;
            dfu_state <= DFU_STATE_dfuUPLOAD_IDLE;
          end
        end

        'h103 : begin
          // DFU_GETSTATUS
          rom_mux    <= ROM_DFUSTATE;
          rom_addr   <= 'h00;
          rom_length <= 6;
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
          dfu_state <= DFU_STATE_dfuIDLE;
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
      max_length <= max_length - 1;
    end

    if (status_stage_end) begin
      setup_data_addr <= 0;
      rom_addr <= 0;
      rom_length <= 0;
      max_length <= 0;

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
  reg [7:0] dfu_mem[15:0];

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

      ep_rom['h008] <= 'h50; // idVendor[0] http://wiki.openmoko.org/wiki/USB_Product_IDs
      ep_rom['h009] <= 'h1d; // idVendor[1]
      ep_rom['h00A] <= 'h30; // idProduct[0]
      ep_rom['h00B] <= 'h61; // idProduct[1]

      ep_rom['h00C] <= 0; // bcdDevice[0]
      ep_rom['h00D] <= 0; // bcdDevice[1]
      ep_rom['h00E] <= 1; // iManufacturer
      ep_rom['h00F] <= 2; // iProduct
      ep_rom['h010] <= 3; // iSerialNumber
      ep_rom['h011] <= 1; // bNumConfigurations

      // configuration descriptor
      ep_rom['h012] <= 9; // bLength
      ep_rom['h013] <= 2; // bDescriptorType
      ep_rom['h014] <= (9+9+9); // wTotalLength[0] FIXME!!!
      ep_rom['h015] <= 0; // wTotalLength[1]
      ep_rom['h016] <= 1; // bNumInterfaces
      ep_rom['h017] <= 1; // bConfigurationValue
      ep_rom['h018] <= 0; // iConfiguration
      ep_rom['h019] <= 'hC0; // bmAttributes
      ep_rom['h01A] <= 50; // bMaxPower
      
      // interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
      ep_rom['h01B] <= 9; // bLength
      ep_rom['h01C] <= 4; // bDescriptorType
      ep_rom['h01D] <= 0; // bInterfaceNumber
      ep_rom['h01E] <= 0; // bAlternateSetting
      ep_rom['h01F] <= 0; // bNumEndpoints
      ep_rom['h020] <= 'hFE; // bInterfaceClass (Application Specific Class Code)
      ep_rom['h021] <= 1; // bInterfaceSubClass (Device Firmware Upgrade Code)
      ep_rom['h022] <= 2; // bInterfaceProtocol (DFU mode protocol)
      ep_rom['h023] <= 4; // iInterface

      // DFU Header Functional Descriptor, DFU Spec 4.1.3, Table 4.2
      ep_rom['h024] <= 9;           // bFunctionLength
      ep_rom['h025] <= 'h21;        // bDescriptorType
      ep_rom['h026] <= 'h0b;				// bmAttributes
      ep_rom['h027] <= 255;         // wDetachTimeout[0]
      ep_rom['h028] <= 0;           // wDetachTimeout[1]
      ep_rom['h029] <= SPI_PAGE_SIZE >> 0; // wTransferSize[0]
      ep_rom['h02A] <= SPI_PAGE_SIZE >> 8; // wTransferSize[1]
      ep_rom['h02B] <= 'h1a;        // bcdDFUVersion[0]
      ep_rom['h02C] <= 'h01;        // bcdDFUVersion[1]

      // Language string descriptor is at string index zero.
      ep_rom['h02D] <= 4;     // bLength
      ep_rom['h02E] <= 3;     // bDescriptorType == STRING
      ep_rom['h02F] <= 'h09;  // wLANGID[0] == US English
      ep_rom['h030] <= 'h04;  // wLANGID[1]

      // String descriptors are located at 'h80 + ('h20 * wIndex - 1)
      str_rom['h000] <= 16;  // bLength
      str_rom['h001] <= 3;   // bDescriptorType == STRING
      str_rom['h002] <= "L";  str_rom['h003] <= 0;
      str_rom['h004] <= "a";  str_rom['h005] <= 0;
      str_rom['h006] <= "t";  str_rom['h007] <= 0;
      str_rom['h008] <= "t";  str_rom['h009] <= 0;
      str_rom['h00A] <= "i";  str_rom['h00B] <= 0;
      str_rom['h00C] <= "c";  str_rom['h00D] <= 0;
      str_rom['h00E] <= "e";  str_rom['h00F] <= 0;

      str_rom['h020] <= 18;  // bLength
      str_rom['h021] <= 3;   // bDescriptorType == STRING
      str_rom['h022] <= "T";  str_rom['h023] <= 0;
      str_rom['h024] <= "i";  str_rom['h025] <= 0;
      str_rom['h026] <= "n";  str_rom['h027] <= 0;
      str_rom['h028] <= "y";  str_rom['h029] <= 0;
      str_rom['h02A] <= "F";  str_rom['h02B] <= 0;
      str_rom['h02C] <= "P";  str_rom['h02D] <= 0;
      str_rom['h02E] <= "G";  str_rom['h02F] <= 0;
      str_rom['h030] <= "A";  str_rom['h031] <= 0;

      str_rom['h040] <= 14;  // bLength
      str_rom['h041] <= 3;   // bDescriptorType == STRING
      str_rom['h042] <= "1";  str_rom['h043] <= 0;
      str_rom['h044] <= "2";  str_rom['h045] <= 0;
      str_rom['h046] <= "3";  str_rom['h047] <= 0;
      str_rom['h048] <= "4";  str_rom['h049] <= 0;
      str_rom['h04A] <= "5";  str_rom['h04B] <= 0;
      str_rom['h04C] <= "6";  str_rom['h04D] <= 0;

      str_rom['h060] <= 12;  // bLength
      str_rom['h061] <= 3;   // bDescriptorType == STRING
      str_rom['h062] <= "F"; str_rom['h063] <= 0;
      str_rom['h064] <= "l"; str_rom['h065] <= 0;
      str_rom['h066] <= "a"; str_rom['h067] <= 0;
      str_rom['h068] <= "s"; str_rom['h069] <= 0;
      str_rom['h06a] <= "h"; str_rom['h06b] <= 0;

      // DFU State data
      dfu_mem['h000] <= DFU_STATUS_OK;      // bStatus
      dfu_mem['h001] <= 0;                  // bwPollTimeout[0]
      dfu_mem['h002] <= 0;                  // bwPollTimeout[1]
      dfu_mem['h003] <= 0;                  // bwPollTimeout[2]
      dfu_mem['h004] <= DFU_STATE_dfuIDLE;  // bState
      dfu_mem['h005] <= 0;                  // iString

  end

endmodule
