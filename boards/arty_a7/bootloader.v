/*
   usb_dfu_a7
   
   Used https://github.com/TomKeddie/TinyFPGA-Bootloader as reference for multiboot. 
   If boot pin is made high, the "normal" image is booted, else, USB DFU.
   Unsure how/if there is a nicer way of booting that does not use a pin.
   
   usage: flash generated file to SPI flash.
   Boot board
   Plug in USB
   Write an FW image to the user image section
   Set boot pin to 1
*/

module bootloader (
  input refclk,
  input reset,
  input boot_button,

  // USB pins
  inout  pin_usb_p,
  inout  pin_usb_n,
  inout  pin_usb_pull,

  // SPI pins
  output spi_csel,
  output spi_clk,
  output spi_mosi,
  input spi_miso,

  // LEDs and Debug
  output [3:0] led
);

    localparam c_user_start = 48'h0000_0200_0000;

    wire usb_p_tx;
    wire usb_n_tx;
    wire usb_p_rx;
    wire usb_n_rx;
    wire usb_tx_en;
    wire [7:0] dfu_state;
    wire [11:0] debug;
    
    /////////////////////////
    // Clock Generation
    ////////////////////////
    reg [1:0] clkdiv = 0;
    wire clk = clkdiv[1];
    wire clk_48mhz;
    wire clk_locked = 1'b1; // FIXME!
    always @(posedge clk_48mhz) clkdiv <= clkdiv + 1;

    usb_pll_clkwiz pll
    (
        .clk_out1(clk_48mhz),
        .clk_in1(refclk)
    );

    /////////////////////////
    // LED Patterns
    /////////////////////////
    reg [31:0] led_counter = 0;
    always @(posedge clk) begin
        led_counter <= led_counter + 1;
    end

    // Simple blink pattern when idle.
    wire [3:0] led_idle = {3'b0, led_counter[21]};

    // Cylon pattern when programming.
    reg [2:0] led_cylon_count = 0;
    reg [3:0] led_cylon = 4'b0;
    always @(posedge led_counter[20]) begin
        if (led_cylon_count) led_cylon_count <= led_cylon_count - 1;
	else led_cylon_count <= 5;

	if (led_cylon_count == 5) led_cylon <= 4'b0010;
	if (led_cylon_count == 4) led_cylon <= 4'b0100;
	if (led_cylon_count <= 3) led_cylon <= 4'b0001 << led_cylon_count;
    end

    // Selecty the LED pattern by DFU state.
    assign led = (dfu_state == 8'h00) ? led_cylon : (dfu_state == 8'h02) ? led_idle : led_cylon;

    /////////////////////////
    // Reset and Multiboot
    /////////////////////////
    reg user_bootmode = 1'b0;
    reg [15:0] reset_delay = 16'hffff;
    wire usb_reset;
    always @(posedge clk) begin
        if (clk_locked && reset_delay) reset_delay <= reset_delay;
	if (boot_button) user_bootmode <= 1'b1;
	if (usb_reset && dfu_state == 8'h01) user_bootmode <= 1'b0;
    end

    /////////////////////////
    // USB DFU Device
    /////////////////////////
    usb_dfu_core dfu (
        .clk_48mhz  (clk_48mhz),
	.clk        (clk),
        .reset      (usb_reset),

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

         // DFU state and debug
        .dfu_state(dfu_state),
        .debug( debug )
    );

    usb_phy_xc7 phy (
        .clk(clk),
	.reset(usb_reset),

	.pin_usb_p(pin_usb_p),
	.pin_usb_n(pin_usb_n),

	.usb_p_tx(usb_p_tx),
	.usb_n_tx(usb_n_tx),
	.usb_p_rx(usb_p_rx),
	.usb_n_rx(usb_n_rx),
	.usb_tx_en(usb_tx_en)
    );

    assign pin_usb_pull = 1'b1;
  
  wbicapetwo #(
    .G_START_ADDRESS(c_user_start)
    )                        
  wbicapetwo_inst (
    .clk(clk_48mhz),
    .reset(reset),
    .boot(boot)
  );

endmodule
