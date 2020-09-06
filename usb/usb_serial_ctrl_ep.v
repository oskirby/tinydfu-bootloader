module usb_serial_ctrl_ep #(
  parameter MAX_IN_PACKET_SIZE = 32,
  parameter MAX_OUT_PACKET_SIZE = 32,
  parameter CDC_ACM_ENDPOINT = 2,
  parameter CDC_RX_ENDPOINT = 1,
  parameter CDC_TX_ENDPOINT = 1
) (
  input clk,
  input clk_48mhz,
  input reset,

  input usb_reset,
  output [6:0] dev_addr,

  ///////////////////////////
  // OUT endpoint interface
  ///////////////////////////
  output out_ep_req,
  input out_ep_grant,
  input out_ep_data_avail,
  input out_ep_setup,
  output out_ep_data_get,
  input [7:0] out_ep_data,
  output out_ep_stall,
  input out_ep_acked,

  ///////////////////////////
  // IN endpoint interface
  ///////////////////////////
  output in_ep_req,
  input in_ep_grant,
  input in_ep_data_free,
  output in_ep_data_put,
  output reg [7:0] in_ep_data = 0,
  output in_ep_data_done,
  output reg in_ep_stall,
  input in_ep_acked,

  ///////////////////////////
  // UART Control Signals.
  ///////////////////////////
  output reg uart_rts = 0,
  output reg uart_dtr = 0,

  ///////////////////////////
  // DFU Detach Signalling
  ///////////////////////////
  output reg dfu_detach = 0,
  output [7:0] debug
);

  // Get board information
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

  localparam STR_INDEX_MANUFACTURER = 1;
  localparam STR_INDEX_PRODUCT = 2;
  localparam STR_INDEX_SERIAL = 3;
  localparam STR_INDEX_UART_MODE = 4;
  localparam STR_INDEX_DFU_MODE = 5;

  localparam DFU_DETACH_TIMEOUT = 1000;

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

  wire has_data_stage = wLength != 16'b0000000000000000; // this version for some reason causes a 16b carry which is slow
  //wire has_data_stage = |wLength;

  wire out_data_stage;
  assign out_data_stage = has_data_stage && !bmRequestType[7];

  wire in_data_stage;
  assign in_data_stage = has_data_stage && bmRequestType[7];

  reg [7:0] rom_length = 0;
  reg [7:0] data_length = 0;

  wire all_data_sent = (ctrl_xfr_state == DATA_IN) && ((rom_length == 8'b0) || (data_length == 8'b0));
  wire more_data_to_send = !all_data_sent;

  wire in_data_transfer_done;
  rising_edge_detector detect_in_data_transfer_done (
    .clk(clk),
    .in(all_data_sent),
    .out(in_data_transfer_done)
  );

  assign in_ep_data_done = (in_data_transfer_done && ctrl_xfr_state == DATA_IN) || send_zero_length_data_pkt;
  assign in_ep_req = (ctrl_xfr_state == DATA_IN) && more_data_to_send;
  assign in_ep_data_put = (ctrl_xfr_state == DATA_IN) && more_data_to_send && in_ep_data_free;

  localparam ROM_ENDPOINT = 0;
  localparam ROM_STRING   = 1;
  localparam ROM_DFUSTATE = 2;

  reg [7:0] rom_addr = 0;
  reg [1:0] rom_mux = ROM_ENDPOINT;

  reg save_dev_addr = 0;
  reg [6:0] new_dev_addr = 0;

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

    if (setup_stage_end) begin
      data_length <= (wLength[15:8]) ? 8'hff : wLength[7:0];
      
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
              rom_mux     <= ROM_STRING;
              rom_addr    <= 'h00;
              rom_length  <= str_rom_length;
            end

            6 : begin
              // DEVICE_QUALIFIER
              in_ep_stall <= 1;
              rom_mux    <= ROM_ENDPOINT;
              rom_addr   <= 'h00;
              rom_length <= 'h00;
            end

            default : begin
              // UNKNOWN
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

        'h00b : begin
          // SET_INTERFACE
          rom_mux    <= ROM_ENDPOINT;
          rom_addr   <= 'h00;
          rom_length <= 'h00;
        end

        'h100 : begin
          // DFU_DETACH
          rom_mux    <= ROM_ENDPOINT;
          rom_addr   <= 0;
          rom_length <= 'h00;
          
          dfu_mem['h004] <= DFU_STATE_appDETACH;
        end

        'h120 : begin
          // SET_LINE_CODING
          rom_mux    <= ROM_ENDPOINT;
          rom_addr   <= 'h00;
          rom_length <= 'h00;
        end

        'h121 : begin
          // GET_LINE_CODING
          rom_mux    <= ROM_ENDPOINT;
          rom_addr   <= 'h67;
          rom_length <= 'h07;
        end

        'h122 : begin
          // SET_CONTROL_LINE_STATE
          rom_mux    <= ROM_ENDPOINT;
          rom_addr   <= 'h00;
          rom_length <= 'h00;

          uart_dtr <= wValue[0];
          uart_rts <= wValue[1];
        end

        'h123 : begin
          // SEND_BREAK
          rom_mux    <= ROM_ENDPOINT;
          rom_addr   <= 'h00;
          rom_length <= 'h00;
        end

        default : begin
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

    if (reset || usb_reset) begin
      dev_addr_i <= 0;
      setup_data_addr <= 0;
      save_dev_addr <= 0;
      uart_rts <= 0;
      uart_dtr <= 0;
    end
  end

  reg [7:0] ep_rom[255:0];
  reg [7:0] dfu_mem[5:0];

  // Mux the data being read
  always @* begin
    case (rom_mux)
      ROM_ENDPOINT : in_ep_data <= ep_rom[rom_addr];
      ROM_STRING   : in_ep_data <= str_rom_data;
      ROM_DFUSTATE : in_ep_data <= dfu_mem[rom_addr];
      default      : in_ep_data <= 8'b0;
    endcase
  end

  initial begin
      // device descriptor
      ep_rom['h000] <= 18; // bLength
      ep_rom['h001] <= 1; // bDescriptorType
      ep_rom['h002] <= 'h00; // bcdUSB[0]
      ep_rom['h003] <= 'h02; // bcdUSB[1]
      ep_rom['h004] <= 'h02; // bDeviceClass (Communications Device Class)
      ep_rom['h005] <= 'h00; // bDeviceSubClass (Abstract Control Model)
      ep_rom['h006] <= 'h00; // bDeviceProtocol (No class specific protocol required)
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

      // configuration descriptor
      ep_rom['h012] <= 9; // bLength
      ep_rom['h013] <= 2; // bDescriptorType
      ep_rom['h014] <= (9+9+5+5+4+5+7+9+7+7+9+9); // wTotalLength[0]
      ep_rom['h015] <= 0; // wTotalLength[1]
      ep_rom['h016] <= 3; // bNumInterfaces
      ep_rom['h017] <= 1; // bConfigurationValue
      ep_rom['h018] <= 0; // iConfiguration
      ep_rom['h019] <= 'hC0; // bmAttributes
      ep_rom['h01A] <= 50; // bMaxPower

      // interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
      ep_rom['h01B] <= 9; // bLength
      ep_rom['h01C] <= 4; // bDescriptorType
      ep_rom['h01D] <= 0; // bInterfaceNumber
      ep_rom['h01E] <= 0; // bAlternateSetting
      ep_rom['h01F] <= 1; // bNumEndpoints
      ep_rom['h020] <= 2; // bInterfaceClass (Communications Device Class)
      ep_rom['h021] <= 2; // bInterfaceSubClass (Abstract Control Model)
      ep_rom['h022] <= 0; // bInterfaceProtocol (0 = ?, 1 = AT Commands: V.250 etc)
      ep_rom['h023] <= STR_INDEX_UART_MODE; // iInterface

      // CDC Header Functional Descriptor, CDC Spec 5.2.3.1, Table 26
      ep_rom['h024] <= 5;     // bFunctionLength
      ep_rom['h025] <= 'h24;  // bDescriptorType
      ep_rom['h026] <= 'h00;  // bDescriptorSubtype
      ep_rom['h027] <= 'h10;
      ep_rom['h028] <= 'h01;  // bcdCDC
      
      // Call Management Functional Descriptor, CDC Spec 5.2.3.2, Table 27
      ep_rom['h029] <= 5;     // bFunctionLength
      ep_rom['h02A] <= 'h24;  // bDescriptorType
      ep_rom['h02B] <= 'h01;  // bDescriptorSubtype
      ep_rom['h02C] <= 'h00;  // bmCapabilities
      ep_rom['h02D] <= 1;     // bDataInterface

      // Abstract Control Management Functional Descriptor, CDC Spec 5.2.3.3, Table 28
      ep_rom['h02E] <= 4;     // bFunctionLength
      ep_rom['h02F] <= 'h24;  // bDescriptorType
      ep_rom['h030] <= 'h02;  // bDescriptorSubtype
      ep_rom['h031] <= 'h06;  // bmCapabilities

      // Union Functional Descriptor, CDC Spec 5.2.3.8, Table 33
      ep_rom['h032] <= 5;     // bFunctionLength
      ep_rom['h033] <= 'h24;  // bDescriptorType
      ep_rom['h034] <= 'h06;  // bDescriptorSubtype
      ep_rom['h035] <= 0;     // bMasterInterface
      ep_rom['h036] <= 1;     // bSlaveInterface0

      // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
      ep_rom['h037] <= 7;     // bLength
      ep_rom['h038] <= 5;     // bDescriptorType
      ep_rom['h039] <= (CDC_ACM_ENDPOINT | 'h80); // bEndpointAddress
      ep_rom['h03A] <= 'h03;  // bmAttributes (0x03=intr)
      ep_rom['h03B] <= 8;     // wMaxPacketSize[0]
      ep_rom['h03C] <= 0;     // wMaxPacketSize[1]
      ep_rom['h03D] <= 64;    // bInterval

    	// interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
      ep_rom['h03E] <= 9;     // bLength
      ep_rom['h03F] <= 4;     // bDescriptorType
      ep_rom['h040] <= 1;     // bInterfaceNumber
      ep_rom['h041] <= 0;     // bAlternateSetting
      ep_rom['h042] <= 2;     // bNumEndpoints
      ep_rom['h043] <= 'h0A;  // bInterfaceClass
      ep_rom['h044] <= 'h00;  // bInterfaceSubClass
      ep_rom['h045] <= 'h00;  // bInterfaceProtocol
      ep_rom['h046] <= 0;     // iInterface

      // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
      ep_rom['h047] <= 7;     // bLength
      ep_rom['h048] <= 5;     // bDescriptorType
      ep_rom['h049] <= CDC_RX_ENDPOINT; // bEndpointAddress
      ep_rom['h04A] <= 'h02;  //bmAttributes (0x02=bulk)
      ep_rom['h04B] <= MAX_IN_PACKET_SIZE; // wMaxPacketSize[0]
      ep_rom['h04C] <= 0;     // wMaxPacketSize[1]
      ep_rom['h04D] <= 0;     // bInterval

      // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
      ep_rom['h04E] <= 7;     // bLength
      ep_rom['h04F] <= 5;     // bDescriptorType
      ep_rom['h050] <= (CDC_TX_ENDPOINT | 'h80); // bEndpointAddress
      ep_rom['h051] <= 'h02;  // bmAttributes (0x02=bulk)
      ep_rom['h052] <= MAX_OUT_PACKET_SIZE; // wMaxPacketSize[0]
      ep_rom['h053] <= 0;     // wMaxPacketSize[1]
      ep_rom['h054] <= 0;     // bInterval

      // DFU Interface Descriptor, DFU Spec 4.1.2, Table 4.1
      ep_rom['h055] <= 9;     // bLength
      ep_rom['h056] <= 4;     // bDescriptorType
      ep_rom['h057] <= 2;     // bInterfaceNumber
      ep_rom['h058] <= 0;     // bAlternateSetting
      ep_rom['h059] <= 0;     // bNumEndpoints
      ep_rom['h05A] <= 'hFE;  // bInterfaceClass (Application Specific Class Code)
      ep_rom['h05B] <= 1;     // bInterfaceSubClass (Device Firmware Upgrade Code)
      ep_rom['h05C] <= 1;     // bInterfaceProtocol (Runtime protocol)
      ep_rom['h05D] <= STR_INDEX_DFU_MODE; // iInterface

      // DFU Header Functional Descriptor, DFU Spec 4.1.3, Table 4.2
      ep_rom['h05E] <= 9;     // bFunctionLength
      ep_rom['h05F] <= 'h21;  // bDescriptorType
      ep_rom['h060] <= 'h0F;  // bmAttributes
      ep_rom['h061] <= DFU_DETACH_TIMEOUT >> 0; // wDetachTimeout[0]
      ep_rom['h062] <= DFU_DETACH_TIMEOUT >> 8; // wDetachTimeout[1]
      ep_rom['h063] <= MAX_OUT_PACKET_SIZE; // wTransferSize[0]
      ep_rom['h064] <= 'h00;  // wTransferSize[1]
      ep_rom['h065] <= 'h10;  // bcdDFUVersion[0]
      ep_rom['h066] <= 'h01;  // bcdDFUVersion[1]
      
      // LINE_CODING
      ep_rom['h067] <= 'h80; // dwDTERate[0]
      ep_rom['h068] <= 'h25; // dwDTERate[1]
      ep_rom['h069] <= 'h00; // dwDTERate[2]
      ep_rom['h06A] <= 'h00; // dwDTERate[3]
      ep_rom['h06B] <= 1; // bCharFormat (1 stop bit)
      ep_rom['h06C] <= 0; // bParityType (None)
      ep_rom['h06D] <= 8; // bDataBits (8 bits)

      // DFU State data
      dfu_mem['h00] <= DFU_STATUS_OK;      // bStatus
      dfu_mem['h01] <= 1;                  // bwPollTimeout[0]
      dfu_mem['h02] <= 0;                  // bwPollTimeout[1]
      dfu_mem['h03] <= 0;                  // bwPollTimeout[2]
      dfu_mem['h04] <= DFU_STATE_appIDLE;  // bState
      dfu_mem['h05] <= 0;                  // iString
  end

  ///////////////////////////////////////////////////////////
  // Generate String Descriptor ROM
  ///////////////////////////////////////////////////////////
  wire [7:0] str_rom_data;
  wire [7:0] str_rom_length;

  usb_string_rom#(
    .STRINGS({
      BOARD_MFR_NAME, 8'h00,
      BOARD_PRODUCT_NAME, 8'h00,
      BOARD_SERIAL, 8'h00,
      "Serial Interface", 8'h00,
      "DFU Interface", 8'h00
    })
  ) str_rom(
    .str_index(wValue[7:0]),
    .rom_addr(rom_addr),
    .rom_length(str_rom_length),
    .rom_data(str_rom_data)
  );

endmodule
