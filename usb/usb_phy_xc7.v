/*
  usb_phy_xc7

  USB PHY for the Xilinx 7-Series FPGAs

  ----------------------------------------------------
  usb_phy_x7 u_u (
    .clk        (clk),
    .reset      (reset),

    // USB pins
    .pin_usb_p( pin_usb_p ),
    .pin_usb_n( pin_usb_n ),

    // USB signals
    input  usb_p_tx,
    input  usb_n_tx,
    output usb_p_rx,
    output usb_n_rx,
    input  usb_tx_en,
  );
*/
module usb_phy_xc7 (
  input  clk,
  output reset,

  // USB pins
  inout  pin_usb_p,
  inout  pin_usb_n,

  // USB signals
  input  usb_p_tx,
  input  usb_n_tx,
  output usb_p_rx,
  output usb_n_rx,
  input  usb_tx_en
);

  wire usb_p_in;
  wire usb_n_in;
  assign usb_p_in = pin_usb_p;
  assign usb_n_in = pin_usb_n;

  assign usb_p_rx = usb_tx_en ? 1'b1 : usb_p_in;
  assign usb_n_rx = usb_tx_en ? 1'b0 : usb_n_in;

  assign pin_usb_p = usb_tx_en ? usb_p_tx : 1'bZ;
  assign pin_usb_n = usb_tx_en ? usb_n_tx : 1'bZ;

  usb_reset_det rst_detector(
    .clk(clk),
    .reset(reset),
    .usb_p_rx(usb_p_in),
    .usb_n_rx(usb_n_in)
  );

endmodule
