/*
    TinyDFU Bootloader for the Logicbone ECP5.
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
pll pll48( .clkin(refclk), .clkout0(clk_48mhz), .locked( clk_locked ) );

// The SPI serial clock requires a special IP block to access.
wire flash_sclk;
USRMCLK usr_sclk(.USRMCLKI(flash_sclk), .USRMCLKTS(1'b0));

//////////////////////////
// LED Patterns
//////////////////////////
reg [31:0] led_counter;
always @(posedge clk_48mhz) begin
    led_counter <= led_counter + 1;
end

// Simple blink pattern when idle.
wire [3:0] led_idle = {3'b0, led_counter[23]};

// Cylon pattern when programming.
reg [2:0] led_cylon_count = 0;
reg [3:0] led_cylon = 4'b0;
always @(posedge led_counter[22]) begin
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
always @(posedge clk_48mhz) begin
    if (clk_locked && reset_delay) reset_delay <= reset_delay - 1;
    if (pwr_button == 1'b0) user_bootmode <= 1'b1;
end
BB pin_resetn( .I( 1'b0 ), .T( user_bootmode || reset_delay ), .O( ), .B( resetn ) );

wire usb_p_tx;
wire usb_n_tx;
wire usb_p_rx;
wire usb_n_rx;
wire usb_tx_en;

// usb DFU - this instanciates the entire USB device.
usb_dfu dfu (
    .clk_48mhz  (clk_48mhz),
    .reset      (~user_bootmode),

    // pins
    .pin_usb_p( usb_ufp_dp ),
    .pin_usb_n( usb_ufp_dm ),

    // SPI
    .spi_csel( flash_csel ),
    .spi_clk( flash_sclk ),
    .spi_mosi( flash_mosi ),
    .spi_miso( flash_miso ),  

    // DFU State and debug
    .dfu_state( dfu_state ), 
    .debug( )
);

// USB Host Detect Pull Up
BB pin_usb_pull( .I( 1'b1 ), .T( ~user_bootmode ), .O( ), .B( usb_ufp_pull ) );



// qhick 'n dirty reset for the i2c block
reg [3:0] i2c_rst_counter = 15;
  reg     i2c_rst = 1;
always @(posedge clk_48mhz) begin
  if (i2c_rst_counter) i2c_rst_counter <= i2c_rst_counter - 1;
  i2c_rst <= |i2c_rst_counter;
end
  
wire i2c_scl_in;
wire i2c_scl_drive_n;
wire i2c_sda_in;
wire i2c_sda_drive_n;

localparam MAX_TRANSFER      = 4;
localparam MAX_TRANSFER_SIZE = $clog2(MAX_TRANSFER+1);

reg [6:0]                    i2c_address = 0;
reg [MAX_TRANSFER*8-1:0]     i2c_write_data = 0;
wire [MAX_TRANSFER*8-1:0]    i2c_read_data;
reg                          i2c_start                 = 0;
reg                          next_i2c_start            = 0;
reg [MAX_TRANSFER_SIZE-1:0]  i2c_write_transfer_length = 0;
reg [MAX_TRANSFER_SIZE-1:0]  i2c_read_transfer_length  = 0;
wire                         i2c_busy;
wire                         i2c_complete;
wire                         i2c_no_response;
wire [MAX_TRANSFER_SIZE-1:0] i2c_total_written;
wire [MAX_TRANSFER_SIZE-1:0] i2c_total_read;

reg [6:0]                    next_i2c_address = 0;
reg [MAX_TRANSFER*8-1:0]     next_i2c_write_data = 0;
reg [MAX_TRANSFER_SIZE-1:0]  next_i2c_write_transfer_length = 0;
reg [MAX_TRANSFER_SIZE-1:0]  next_i2c_read_transfer_length  = 0;
  
i2c_master #(
  .MAX_TRANSFER_LENGTH (       4 ),
  .INPUT_FREQUENCY     ( 48000000 ),
  .FREQUENCY           (   100000 )
) i2c_master_inst (
  .clk                   ( clk_48mhz ),
  .rst                   ( i2c_rst ),
  .i2c_address           ( i2c_address ),
  .write_data            ( i2c_write_data ),
  .read_data             ( i2c_read_data ),
  .start                 ( i2c_start ),
  .write_transfer_length ( i2c_write_transfer_length ),
  .read_transfer_length  ( i2c_read_transfer_length ),
  .busy                  ( i2c_busy ),
  .complete              ( i2c_complete ),
  .no_response           ( i2c_no_response ),
  .total_written         ( i2c_total_written ),
  .total_read            ( i2c_total_read ),
  .i2c_scl_in            ( i2c_scl_in ),
  .i2c_scl_drive_n       ( i2c_scl_drive_n ),
  .i2c_sda_in            ( i2c_sda_in ),
  .i2c_sda_drive_n       ( i2c_sda_drive_n )
);

  
localparam I2C_SEQ_DELAY               = 14'b00000000000001;
localparam I2C_SEQ_CHECK_USB           = 14'b00000000000010;
localparam I2C_SEQ_CHECK_USB_FIN       = 14'b00000000000100;
localparam I2C_SEQ_PROG_DOWNSTREAM     = 14'b00000000001000;
localparam I2C_SEQ_PROG_DOWNSTREAM_FIN = 14'b00000000010000;
localparam I2C_SEQ_CMD_3               = 14'b00000000100000;
localparam I2C_SEQ_CMD_3_FIN           = 14'b00000001000000;
localparam I2C_SEQ_CMD_4               = 14'b00000010000000;
localparam I2C_SEQ_CMD_4_FIN           = 14'b00000100000000;
localparam I2C_SEQ_CMD_5               = 14'b00001000000000;
localparam I2C_SEQ_CMD_5_FIN           = 14'b00010000000000;
localparam I2C_SEQ_CMD_6               = 14'b00100000000000;
localparam I2C_SEQ_CMD_6_FIN           = 14'b01000000000000;
localparam I2C_SEQ_DONE                = 14'b10000000000000;
reg [13:0]                 i2c_seq_state;
reg [13:0]                 next_i2c_seq_state;

reg [31:0]                 i2c_seq_delay;
reg [31:0]                 next_i2c_seq_delay;
  
always @(*) begin
  next_i2c_seq_state        = i2c_seq_state;
  next_i2c_seq_delay        = i2c_seq_delay;
  next_i2c_start            = 0;

  i2c_address               = 0;
  i2c_write_transfer_length = 0;
  i2c_read_transfer_length  = 0;
  i2c_write_data            = 0;
      
  case (i2c_seq_state)
  I2C_SEQ_DELAY: begin
    if (i2c_seq_delay) next_i2c_seq_delay = i2c_seq_delay - 1;
    else begin
      next_i2c_seq_state = I2C_SEQ_CHECK_USB;
    end
  end
  
  I2C_SEQ_CHECK_USB: begin
    i2c_address                   = 7'h2D;
    i2c_write_transfer_length     = 1;
    i2c_read_transfer_length      = 2;
    
    i2c_write_data                = 32'h1C;

    next_i2c_start = 1;
    if (i2c_busy) next_i2c_seq_state = I2C_SEQ_CHECK_USB_FIN;
  end

  I2C_SEQ_CHECK_USB_FIN: begin
    if (!i2c_busy) next_i2c_seq_state = I2C_SEQ_PROG_DOWNSTREAM;
  end
  
  I2C_SEQ_PROG_DOWNSTREAM: begin
    next_i2c_seq_state = I2C_SEQ_PROG_DOWNSTREAM_FIN;
  end
  I2C_SEQ_PROG_DOWNSTREAM_FIN: begin
    next_i2c_seq_state = I2C_SEQ_CMD_3;
  end
  
  
  I2C_SEQ_CMD_3: begin
    next_i2c_seq_state = I2C_SEQ_CMD_3_FIN;
  end
  I2C_SEQ_CMD_3_FIN: begin
    next_i2c_seq_state = I2C_SEQ_CMD_4;
  end
  
  I2C_SEQ_CMD_4: begin
    next_i2c_seq_state = I2C_SEQ_CMD_4_FIN;
  end
  I2C_SEQ_CMD_4_FIN: begin
    next_i2c_seq_state = I2C_SEQ_CMD_5;
  end
  
  I2C_SEQ_CMD_5: begin
    next_i2c_seq_state = I2C_SEQ_CMD_5_FIN;
  end
  I2C_SEQ_CMD_5_FIN: begin
    next_i2c_seq_state = I2C_SEQ_CMD_6;
  end
  
  I2C_SEQ_CMD_6: begin
    next_i2c_seq_delay = 100000;
    next_i2c_seq_state = I2C_SEQ_CMD_6_FIN;
  end
  I2C_SEQ_CMD_6_FIN: begin
    next_i2c_seq_state = I2C_SEQ_DONE;
  end
  
  I2C_SEQ_DONE: begin
  end
      
  endcase
end

always @(posedge clk_48mhz or posedge i2c_rst) begin
  if (i2c_rst) begin
    i2c_seq_state                  <= I2C_SEQ_DELAY;
    i2c_seq_delay                  <= 500;
    i2c_start                      <= 0;
  end
  else begin
    i2c_seq_state <= next_i2c_seq_state;
    i2c_seq_delay <= next_i2c_seq_delay;
    i2c_start     <= next_i2c_start;;
  end
end
  
assign debug = { i2c_read_data[15:8], 2'd0, next_i2c_start, i2c_busy, i2c_complete, i2c_no_response, i2c_scl_in, i2c_sda_in };

BB pin_i2c_scl( .I( 1'b0 ), .T( i2c_scl_drive_n ), .O( i2c_scl_in ), .B( i2c_scl ) );
BB pin_i2c_sda( .I( 1'b0 ), .T( i2c_sda_drive_n ), .O( i2c_sda_in ), .B( i2c_sda ) );
  

endmodule
