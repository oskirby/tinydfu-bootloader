/*
 *  TinyDFU Bootloader for the Logicbone ECP5.
 */
module logicbone_ecp5 (
    input        refclk,
    output       resetn,
    input        pwr_button,

    inout        usb_ufp_dp,
    inout        usb_ufp_dm,
    output       usb_ufp_pull,

    output [3:0] led,

    output       flash_csel,
    //output flash_sclk, // Requires a special USRMCLK block.
    output       flash_mosi,
    input        flash_miso,

    // i2c interface
    inout        i2c_scl,
    inout        i2c_sda,
                       
    output [15:0] debug
);

// Use an ecppll generated pll
wire clk_48mhz;
wire clk_locked;
reg  clk_24mhz = 0;
reg  clk = 0;
pll pll48( .clkin(refclk), .clkout0(clk_48mhz), .locked( clk_locked ) );
always @(posedge clk_48mhz) clk_24mhz <= ~clk_24mhz;
always @(posedge clk_24mhz) clk <= ~clk;

// The SPI serial clock requires a special IP block to access.
wire flash_sclk;
USRMCLK usr_sclk(.USRMCLKI(flash_sclk), .USRMCLKTS(1'b0));

//////////////////////////
// LED Patterns
//////////////////////////
reg [31:0] led_counter;
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

   if (led_cylon_count == 4) led_cylon <= 4'b0100;
   else if (led_cylon_count == 5) led_cylon <= 4'b0010;
   else led_cylon <= 4'b0001 << led_cylon_count[1:0];
end

// Select the LED pattern by DFU state.
wire [7:0] dfu_state;
assign led = (dfu_state == 'h02) ? ~led_idle : ~led_cylon;

//////////////////////////
// Reset and Multiboot
//////////////////////////
reg user_bootmode = 1'b0;
reg [15:0] reset_delay = 16'hffff;
wire usb_reset;
always @(posedge clk) begin
    if (clk_locked && reset_delay) reset_delay <= reset_delay - 1;
    if (pwr_button == 1'b0) user_bootmode <= 1'b1;
    if (usb_reset && dfu_state == 8'h01) user_bootmode <= 1'b0;
end
BB pin_resetn( .I( 1'b0 ), .T( user_bootmode || reset_delay ), .O( ), .B( resetn ) );

wire usb_p_tx;
wire usb_n_tx;
wire usb_p_rx;
wire usb_n_rx;
wire usb_tx_en;

// USB DFU - this instanciates the entire USB device.
usb_dfu_core dfu (
  .clk_48mhz  (clk_48mhz),
  .clk        (clk),
  .reset      (~user_bootmode),

  // USB signals
  .usb_p_tx( usb_p_tx ),
  .usb_n_tx( usb_n_tx ),
  .usb_p_rx( usb_p_rx ),
  .usb_n_rx( usb_n_rx ),
  .usb_tx_en( usb_tx_en ),

  // SPI
  .spi_csel( flash_csel ),
  .spi_clk( flash_sclk ),
  .spi_mosi( flash_mosi ),
  .spi_miso( flash_miso ),

  // DFU State and debug
  .dfu_state( dfu_state ),
  .debug( )
);

// USB Physical interface
usb_phy_ecp5 phy (
  .clk (clk_48mhz),
  .reset (usb_reset),

  .pin_usb_p (usb_ufp_dp),
  .pin_usb_n (usb_ufp_dm),

  .usb_p_tx( usb_p_tx ),
  .usb_n_tx( usb_n_tx ),
  .usb_p_rx( usb_p_rx ),
  .usb_n_rx( usb_n_rx ),
  .usb_tx_en( usb_tx_en ),
);

// USB Host Detect Pull Up
BB pin_usb_pull( .I( 1'b1 ), .T( ~user_bootmode ), .O( ), .B( usb_ufp_pull ) );

//////////////////////////
// I2C Board Setup
//////////////////////////
wire i2c_bringup_done;
wire i2c_bringup_fail;

wire i2c_scl_in;
wire i2c_scl_drive_n;
wire i2c_sda_in;
wire i2c_sda_drive_n;
BB pin_i2c_scl( .I( 1'b0 ), .T( i2c_scl_drive_n ), .O( i2c_scl_in ), .B( i2c_scl ) );
BB pin_i2c_sda( .I( 1'b0 ), .T( i2c_sda_drive_n ), .O( i2c_sda_in ), .B( i2c_sda ) );

i2c_bringup #(
  .CLK_FREQUENCY  ( 12000000 ),
) i2c_bringup_inst (
  .clk            ( clk ),
  .done           ( i2c_bringup_done ),
  .fail           ( i2c_bringup_fail ),

  .i2c_scl_in     ( i2c_scl_in ),
  .i2c_scl_drive_n( i2c_scl_drive_n ),
  .i2c_sda_in     ( i2c_sda_in ),
  .i2c_sda_drive_n( i2c_sda_drive_n ),

  .debug( debug )
);

endmodule
