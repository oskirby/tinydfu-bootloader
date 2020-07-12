// detects USB port reset signal from host
module usb_reset_det (
  input clk,
  output reset,

  input usb_p_rx,
  input usb_n_rx
);

  // USB reset is signalled by single-ended zero state for 10ms.
  // Devices may reset after SE0 is detected for 2.5us or more.
  localparam RESET_TIMEOUT = 12000;

  // reset detection
  reg [16:0] reset_timer = RESET_TIMEOUT;
  reg reset_i = 0;

  always @(posedge clk) reset_i <= (reset_timer == 0);
  assign reset = reset_i;
 
  always @(posedge clk) begin
    if (usb_p_rx || usb_n_rx) begin
      reset_timer <= RESET_TIMEOUT;
    end else if (reset_timer) begin
      reset_timer <= reset_timer - 1;
    end
  end
  
endmodule
