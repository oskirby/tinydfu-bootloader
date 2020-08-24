/*
   usb_serial_core

  Luke Valenti's USB module, as adapted by Lawrie Griffiths and Owen Kirby.

  This module was originally tinyfpga_bootloader.v in Luke's code.

  Instanciation template with separate pipe signals

  ----------------------------------------------------
  usb_serual_core dfu (
    .clk_48mhz  (clk_48mhz),
    .reset      (reset),

    // USB pins - these must be connected properly to the outside world.  See below.
    .usb_p_tx(usb_p_tx),
    .usb_n_tx(usb_n_tx),
    .usb_p_rx(usb_p_rx),
    .usb_n_rx(usb_n_rx),
    .usb_tx_en(usb_tx_en),

    // SPI pins
    .spi_csel(spi_csel),
    .spi_clk(spi_clk),
    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),

    // DFU state and debug
    .dfu_state(dfu_state),
    .debug( debug )
  );

  ----------------------------------------------------

  Then, the actual physical pins need some special handling.  Get some
  help from the device.

  ----------------------------------------------------
  assign usb_p_rx = usb_tx_en ? 1'b1 : usb_p_in;
  assign usb_n_rx = usb_tx_en ? 1'b0 : usb_n_in;

  SB_IO #(
    .PIN_TYPE(6'b 1010_01), // PIN_OUTPUT_TRISTATE - PIN_INPUT
    .PULLUP(1'b 0)
  )
  iobuf_usbp
  (
    .PACKAGE_PIN(pin_usbp),
    .OUTPUT_ENABLE(usb_tx_en),
    .D_OUT_0(usb_p_tx),
    .D_IN_0(usb_p_in)
  );

  SB_IO #(
    .PIN_TYPE(6'b 1010_01), // PIN_OUTPUT_TRISTATE - PIN_INPUT
    .PULLUP(1'b 0)
  )
  iobuf_usbn
  (
    .PACKAGE_PIN(pin_usbn),
    .OUTPUT_ENABLE(usb_tx_en),
    .D_OUT_0(usb_n_tx),
    .D_IN_0(usb_n_in)
  );
  ----------------------------------------------------

It should be noted that there are no other invocations of usb stuff other than
usb_dfu.v.   Since it's the top, I'm going to put some doc in here.

General note: USB communications happen over endpoints.  The OUT endpoints are out
with respect to the HOST, and IN endpoints are in with respect to the HOST.

Files:

usb_dfu.v       - top level module creates the control end point, (usb_dfu_ctrl_ep) and
                  the main protocol engine (usb_fs_pe - passing in the endpoint count,
                  along with the actual usb signal lines).  Also, the endpoint signals
                  are connected to the protocol engine in its invocation.  The DFU
                  control end point cluster has two endpoints each - one in and one out.

usb_serial_ctrl_ep - DFU control logic.  Two end point interfaces are (one in one
                  out) passed in with their various signal lines.  Contains all the
                  USB setup logic.  Returns the configuration etc.  Vendor 50 1D
                  Product 30 61.  Two interfaces. Descriptors.  Obviously the main
                  configuration file.

usb_spiflash_bridge.v - where the data action is for us. Is passed the SPI signals to the
                  flash device. This translates the read, erase and write state machines
                  into the USB endpoint interface.

usb_fs_pe.v     - full speed protocol engine - instanciates all the endpoints, making
                  arrays of in and out end point signals.  Also is passed in are all
                  the actual interfaces to the end points.

                  Creates the in and out arbitors (usb_fs_in_arb, usb_fs_out_arb)

                  Creates the in protocol engine (usb_fs_in_pe), and the out protocol
                  engine (usb_fs_out_pe)

                  Creates the receiver and transmitter (usb_fs_rx, usb_fs_tx)

                  Creates the tx mux and the rx mux which permit the different engines
                  to talk.

usb_fs_in_pe.v  - in protocol engine

usb_fs_out_pe.v - out protocol engine

usb_fs_rx.v     - Actual rx logic

usb_fs_tx.v     - Actual tx logic

edge_detect.v   - rising and falling edge detectors

serial.v        - width adapter (x widths to y widths)

*/

module usb_serial_core (
  input  clk_48mhz,
  input  clk,
  input  reset,

  // USB lines.  Split into input vs. output and oe control signal to maintain
  // highest level of compatibility with synthesis tools.
  output usb_p_tx,
  output usb_n_tx,
  input  usb_p_rx,
  input  usb_n_rx,
  output usb_tx_en,

  // uart pipeline in (into the module, out of the device, into the host)
  input [7:0] uart_in_data,
  input       uart_in_valid,
  output      uart_in_ready,

  // uart pipeline out (out of the host, into the device, out of the module)
  output [7:0] uart_out_data,
  output       uart_out_valid,
  input        uart_out_get,
  
  // DFU state and debug
  output dfu_detach,
  output [7:0] dfu_state,
  output [11:0] debug
);

  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////
  ////////
  //////// usb engine
  ////////
  ////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////

  wire usb_reset;
  wire [6:0] dev_addr;
  wire [7:0] out_ep_data;

  wire ctrl_out_ep_req;
  wire ctrl_out_ep_grant;
  wire ctrl_out_ep_data_avail;
  wire ctrl_out_ep_setup;
  wire ctrl_out_ep_data_get;
  wire ctrl_out_ep_stall;
  wire ctrl_out_ep_acked;

  wire ctrl_in_ep_req;
  wire ctrl_in_ep_grant;
  wire ctrl_in_ep_data_free;
  wire ctrl_in_ep_data_put;
  wire [7:0] ctrl_in_ep_data;
  wire ctrl_in_ep_data_done;
  wire ctrl_in_ep_stall;
  wire ctrl_in_ep_acked;

  wire sof_valid;
  wire [10:0] frame_index;

  reg [31:0] host_presence_timer = 0;
  reg host_presence_timeout = 0;

  usb_serial_ctrl_ep ctrl_ep_inst (
    .clk(clk),
    .clk_48mhz(clk_48mhz),
    .reset(reset),
    .usb_reset(usb_reset),
    .dev_addr(dev_addr),

    // out endpoint interface
    .out_ep_req(ctrl_out_ep_req),
    .out_ep_grant(ctrl_out_ep_grant),
    .out_ep_data_avail(ctrl_out_ep_data_avail),
    .out_ep_setup(ctrl_out_ep_setup),
    .out_ep_data_get(ctrl_out_ep_data_get),
    .out_ep_data(out_ep_data),
    .out_ep_stall(ctrl_out_ep_stall),
    .out_ep_acked(ctrl_out_ep_acked),

    // in endpoint interface
    .in_ep_req(ctrl_in_ep_req),
    .in_ep_grant(ctrl_in_ep_grant),
    .in_ep_data_free(ctrl_in_ep_data_free),
    .in_ep_data_put(ctrl_in_ep_data_put),
    .in_ep_data(ctrl_in_ep_data),
    .in_ep_data_done(ctrl_in_ep_data_done),
    .in_ep_stall(ctrl_in_ep_stall),
    .in_ep_acked(ctrl_in_ep_acked),

    // DFU state and debug
    .dfu_detach(dfu_detach)
  );

  wire uart_in_ep_req;
  wire uart_in_ep_grant;
  wire uart_in_ep_data_free;
  wire uart_in_ep_data_put;
  wire [7:0] uart_in_ep_data;
  wire uart_in_ep_data_done;
  wire uart_in_ep_stall;
  wire uart_in_ep_acked;
  usb_uart_in_ep in_ep_inst (
    .clk(clk),
    .reset(reset),

    .in_ep_req(uart_in_ep_req),
    .in_ep_grant(uart_in_ep_grant),
    .in_ep_data_free(uart_in_ep_data_free),
    .in_ep_data_put(uart_in_ep_data_put),
    .in_ep_data(uart_in_ep_data),
    .in_ep_data_done(uart_in_ep_data_done),
    .in_ep_stall(uart_in_ep_stall),
    .in_ep_acked(uart_in_ep_acked),

    .uart_in_data(uart_in_data),
    .uart_in_valid(uart_in_valid),
    .uart_in_ready(uart_in_ready)
  );

  wire uart_out_ep_req;
  wire uart_out_ep_grant;
  wire uart_out_ep_data_avail;
  wire uart_out_ep_setup;
  wire uart_out_ep_data_get;
  wire uart_out_ep_stall;
  wire uart_out_ep_acked;
  usb_uart_out_ep out_ep_inst (
    .clk(clk),
    .reset(reset),

    .out_ep_req(uart_out_ep_req),
    .out_ep_grant(uart_out_ep_grant),
    .out_ep_data_avail(uart_out_ep_data_avail),
    .out_ep_setup(uart_out_ep_setup),
    .out_ep_data_get(uart_out_ep_data_get),
    .out_ep_data(out_ep_data),
    .out_ep_stall(uart_out_ep_stall),
    .out_ep_acked(uart_out_ep_acked),

    .uart_out_data(uart_out_data),
    .uart_out_valid(uart_out_valid),
    .uart_out_get(uart_out_get)
  );

  usb_fs_pe #(
    .NUM_OUT_EPS(5'd2),
    .NUM_IN_EPS(5'd2)
  ) usb_fs_pe_inst (
    .clk_48mhz(clk_48mhz),
    .clk(clk),
    .reset(reset),

    .usb_p_tx(usb_p_tx),
    .usb_n_tx(usb_n_tx),
    .usb_p_rx(usb_p_rx),
    .usb_n_rx(usb_n_rx),
    .usb_tx_en(usb_tx_en),

    .dev_addr(dev_addr),

    // out endpoint interfaces
    .out_ep_req       ({uart_out_ep_req,        ctrl_out_ep_req}),
    .out_ep_grant     ({uart_out_ep_grant,      ctrl_out_ep_grant}),
    .out_ep_data_avail({uart_out_ep_data_avail, ctrl_out_ep_data_avail}),
    .out_ep_setup     ({uart_out_ep_setup,      ctrl_out_ep_setup}),
    .out_ep_data_get  ({uart_out_ep_data_get,   ctrl_out_ep_data_get}),
    .out_ep_data      (out_ep_data),
    .out_ep_stall     ({uart_out_ep_stall,      ctrl_out_ep_stall}),
    .out_ep_acked     ({uart_out_ep_acked,      ctrl_out_ep_acked}),

    // in endpoint interfaces
    .in_ep_req        ({uart_in_ep_req,         ctrl_in_ep_req}),
    .in_ep_grant      ({uart_in_ep_grant,       ctrl_in_ep_grant}),
    .in_ep_data_free  ({uart_in_ep_data_free,   ctrl_in_ep_data_free}),
    .in_ep_data_put   ({uart_in_ep_data_put,    ctrl_in_ep_data_put}),
    .in_ep_data       ({uart_in_ep_data[7:0],   ctrl_in_ep_data[7:0]}),
    .in_ep_data_done  ({uart_in_ep_data_done,   ctrl_in_ep_data_done}),
    .in_ep_stall      ({uart_in_ep_stall,       ctrl_in_ep_stall}),
    .in_ep_acked      ({uart_in_ep_acked,       ctrl_in_ep_acked}),

    // sof interface
    .sof_valid(sof_valid),
    .frame_index(frame_index),
    .rst_detect(usb_reset),

    // Debug
    .debug(debug[11:4])
  );

endmodule
