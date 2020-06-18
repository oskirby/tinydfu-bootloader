/*
  usb_dfu_ecp5

  Simple wrapper around the usb_dfu which incorporates the Pin driver logic
  so this doesn't clutter the top level circuit

  Make the signature generic (usb_dfu) and rely on the file inclusion process (makefile)
  to bring the correct architecture in

  ----------------------------------------------------
  usb_dfu u_u (
    .clk_48mhz  (clk_48mhz),
    .reset      (reset),

    // USB pins
    .pin_usb_p( pin_usb_p ),
    .pin_usb_n( pin_usb_n ),

    // SPI pins
    .spi_csel( spi_csel ),
    .spi_clk( spi_clk ),
    .spi_mosi( spi_mosi ),
    .spi_miso( spi_miso )
  );

*/
module usb_dfu (
  input  clk_48mhz,
  input reset,

  // USB pins
  inout  pin_usb_p,
  inout  pin_usb_n,

  // SPI pins
  output spi_csel,
  output spi_clk,
  output spi_mosi,
  input spi_miso,

  output [11:0] debug
);

  wire usb_p_tx;
  wire usb_n_tx;
  wire usb_p_rx;
  wire usb_n_rx;
  wire usb_tx_en;

  // wire [11:0] debug_dum;

  usb_dfu_core dfu (
      .clk_48mhz  (clk_48mhz),
      .reset      (reset),

      // pins - these must be connected properly to the outside world.  See below.
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

      .debug( debug )
  );

  wire usb_p_in;
  wire usb_n_in;

  assign usb_p_rx = usb_tx_en ? 1'b1 : usb_p_in;
  assign usb_n_rx = usb_tx_en ? 1'b0 : usb_n_in;

	// T = TRISTATE (not transmit)
	BB io_p( .I( usb_p_tx ), .T( !usb_tx_en ), .O( usb_p_in ), .B( pin_usb_p ) );
	BB io_n( .I( usb_n_tx ), .T( !usb_tx_en ), .O( usb_n_in ), .B( pin_usb_n ) );

endmodule
