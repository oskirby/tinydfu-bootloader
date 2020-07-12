/*
 * i2c board bringup state machine.
 *
 * This module executes a bunch of I2C transactions to configure
 * the hardware for correct operation as follows:
 *  1) Read the USB-C UFP current limit resistors to figure out
 *     how much power the board can consume.
 *  2) Program the USB-C DFP power switch and DFP port controller
 *     to permit a suitable current limit to downstream devices.
 *  3) Enable the USB-C DFP power switch and DFP port controller.
 */

module i2c_bringup #(
  parameter CLK_FREQUENCY = 12000000,
) (
  input wire clk,

  output wire done,
  output wire fail,

  input wire i2c_scl_in,
  output reg i2c_scl_drive_n,
  input wire i2c_sda_in,
  output reg i2c_sda_drive_n,

  output wire [15:0] debug
);

localparam BUS_FREQUENCY = 100000;

// MxL7704 Power Manager bus and register addresses
localparam MXL7704_BUS_ADDR = 7'h2D;
localparam MXL7704_REG_ADC1 = 8'h1C;
localparam MXL7704_REG_ADC2 = 8'h1D;

// UFP Current detection thresholds.
localparam MXL7704_UFP_THR_LOW = 8'd25;    /* 80uA current sink: Detect from 250mV to 660mV */
localparam MXL7704_UFP_THR_MED = 8'd66;    /* 180uA current sink: Detect from 660mV to 1.23V */
localparam MXL7704_UFP_THR_HIGH = 8'd123;  /* 300uA current sink: Detect at 1.23V and above */

// FUSB307 USB-C DFP controller bus and register addresses 
localparam FUSB307_BUS_ADDR = 7'h52;
localparam FUSB307_REG_ROLECTL   = 8'h1A;
localparam FUSB307_REG_PWRCTRL   = 8'h1C;
localparam FUSB307_REG_CCSTAT    = 8'h1D;
localparam FUSB307_REG_PWRSTAT   = 8'h1E;
localparam FUSB307_REG_COMMAND   = 8'h23;
localparam FUSB307_REG_GPIO1_CFG = 8'hA4;
localparam FUSB307_REG_GPIO2_CFG = 8'hA5;
localparam FUSB307_REG_RESET     = 8'hA2;

// Generate a Reset Delay
reg [3:0] i2c_rst_counter = 15;
  reg     i2c_rst = 1;
always @(posedge clk) begin
  if (i2c_rst_counter) i2c_rst_counter <= i2c_rst_counter - 1;
  i2c_rst <= |i2c_rst_counter;
end

localparam MAX_TRANSFER      = 4;
localparam MAX_TRANSFER_SIZE = $clog2(MAX_TRANSFER+1);

reg [6:0]                    i2c_address = 0;
reg [MAX_TRANSFER*8-1:0]     i2c_write_data = 0;
wire [MAX_TRANSFER*8-1:0]    i2c_read_data;
reg                          i2c_start = 0;
reg [MAX_TRANSFER_SIZE-1:0]  i2c_write_length = 0;
reg [MAX_TRANSFER_SIZE-1:0]  i2c_read_length  = 0;
wire                         i2c_busy;
wire                         i2c_complete;
wire                         i2c_no_response;
wire [MAX_TRANSFER_SIZE-1:0] i2c_total_written;
wire [MAX_TRANSFER_SIZE-1:0] i2c_total_read;

reg                          next_i2c_start = 0;
reg [6:0]                    next_i2c_address = 0;
reg [MAX_TRANSFER*8-1:0]     next_i2c_write_data = 0;
reg [MAX_TRANSFER_SIZE-1:0]  next_i2c_write_length = 0;
reg [MAX_TRANSFER_SIZE-1:0]  next_i2c_read_length  = 0;
  
i2c_master #(
  .MAX_TRANSFER_LENGTH   (        4 ),
  .INPUT_FREQUENCY       ( CLK_FREQUENCY ),
  .FREQUENCY             ( BUS_FREQUENCY )
) i2c_master_inst (
  // Clock and Reset
  .clk                   ( clk ),
  .rst                   ( i2c_rst ),

  // I2C Transfer
  .i2c_address           ( i2c_address ),
  .write_data            ( i2c_write_data ),
  .write_transfer_length ( i2c_write_length ),
  .read_data             ( i2c_read_data ),
  .read_transfer_length  ( i2c_read_length ),

  // Control Signals
  .start                 ( i2c_start ),
  .busy                  ( i2c_busy ),
  .complete              ( i2c_complete ),
  .no_response           ( i2c_no_response ),
  .total_written         ( i2c_total_written ),
  .total_read            ( i2c_total_read ),

  // I2C Physical Interface
  .i2c_scl_in            ( i2c_scl_in ),
  .i2c_scl_drive_n       ( i2c_scl_drive_n ),
  .i2c_sda_in            ( i2c_sda_in ),
  .i2c_sda_drive_n       ( i2c_sda_drive_n )
);

// Locate the busy falling edge for state changes.
wire i2c_busy_end;
falling_edge_detector detect_i2c_busy_end (
    .clk(clk),
    .in(i2c_busy),
    .out(i2c_busy_end)
);

// I2C Board Setup State Machine
localparam I2C_SEQ_DELAY                  = 19'b0000000000000000001;  // Waiting for the reset delay to elapse.
localparam I2C_SEQ_READ_UFP_ADC1          = 19'b0000000000000000010;
localparam I2C_SEQ_READ_UFP_ADC2          = 19'b0000000000000000100;
localparam I2C_SEQ_WRITE_DFP_RESET        = 19'b0000000000000001000;
localparam I2C_SEQ_POLL_DFP_TCPC_INIT     = 16'b0000000000000010000;
localparam I2C_SEQ_WRITE_DFP_GPIO1        = 19'b0000000000000100000;
localparam I2C_SEQ_WRITE_DFP_GPIO2        = 19'b0000000000001000000;
localparam I2C_SEQ_WRITE_DFP_ROLECTL      = 19'b0000000000010000000;
localparam I2C_SEQ_COMMAND_DFP_SINK       = 19'b0000000000100000000;
localparam I2C_SEQ_READ_DFP_PWRSTAT       = 19'b0000000001000000000;
localparam I2C_SEQ_DONE                   = 19'b0000000010000000000;
localparam I2C_SEQ_FAIL                   = 19'b0000000100000000000;
reg [17:0]                 i2c_seq_state;
reg [17:0]                 next_i2c_seq_state;

reg [31:0]                 i2c_seq_delay;
reg [31:0]                 next_i2c_seq_delay;

assign done = (i2c_seq_state == I2C_SEQ_DONE);
assign fail = (i2c_seq_state == I2C_SEQ_FAIL);

// Detect the UFP current limit and orientation.
reg [7:0] mxl7704_adc1 = 0;
reg [7:0] mxl7704_adc2 = 0;
wire [7:0] usb_ufp_ccvolts;
wire [2:0] usb_ufp_climit;
wire usb_ufp_orient;

assign usb_ufp_orient = (mxl7704_adc1 > mxl7704_adc2);
assign usb_ufp_ccvolts = usb_ufp_orient ? mxl7704_adc1 : mxl7704_adc2;
assign usb_ufp_climit[2] = (mxl7704_adc1 > 123) | (mxl7704_adc2 > 123);
assign usb_ufp_climit[1] = (mxl7704_adc1 > 66)  | (mxl7704_adc2 > 66);
assign usb_ufp_climit[0] = (mxl7704_adc1 > 25)  | (mxl7704_adc2 > 25); 

always @(*) begin
  next_i2c_seq_state    = i2c_seq_state;
  next_i2c_seq_delay    = i2c_seq_delay;
  next_i2c_start        = 0;

  i2c_address           = 0;
  i2c_write_length      = 0;
  i2c_read_length       = 0;
  i2c_write_data        = 0;
      
  case (i2c_seq_state)
  I2C_SEQ_DELAY: begin
    if (i2c_seq_delay) next_i2c_seq_delay = i2c_seq_delay - 1;
    else begin
      next_i2c_seq_state = I2C_SEQ_READ_UFP_ADC1;
    end
  end
  
  // Read the ADC1 from the PMIC for USB-C UFP current detection.
  I2C_SEQ_READ_UFP_ADC1: begin
    i2c_address         = MXL7704_BUS_ADDR;
    i2c_write_length    = 1;
    i2c_read_length     = 1;
    
    i2c_write_data      = {24'b0, MXL7704_REG_ADC1};

    next_i2c_start = ~i2c_busy;
    if (i2c_busy_end) next_i2c_seq_state = I2C_SEQ_READ_UFP_ADC2;
  end
  
  // Read the ADC2 from the PMIC for USB-C UFP current detection.
  I2C_SEQ_READ_UFP_ADC2: begin
    i2c_address         = MXL7704_BUS_ADDR;
    i2c_write_length    = 1;
    i2c_read_length     = 1;
    
    i2c_write_data      = {24'b0, MXL7704_REG_ADC2};

    next_i2c_start = ~i2c_busy;
    if (i2c_busy_end) next_i2c_seq_state = I2C_SEQ_WRITE_DFP_RESET;
  end

  // Issue a software reset to the DFP port controller.
  I2C_SEQ_WRITE_DFP_RESET: begin
    i2c_address         = FUSB307_BUS_ADDR;
    i2c_write_length    = 2;
    i2c_read_length     = 0;

    i2c_write_data  = {16'b0, 8'h01, FUSB307_REG_RESET};

    next_i2c_start = ~i2c_busy;
    if (i2c_busy_end) next_i2c_seq_state = I2C_SEQ_POLL_DFP_TCPC_INIT;
  end

  // Poll PWRSTAT until the soft reset is complete.
  I2C_SEQ_POLL_DFP_TCPC_INIT: begin
    i2c_address         = FUSB307_BUS_ADDR;
    i2c_write_length    = 1;
    i2c_read_length     = 1;

    i2c_write_data  = {24'b0, FUSB307_REG_PWRSTAT};

    next_i2c_start = ~i2c_busy;
    if (i2c_busy_end && ~i2c_read_data[6]) next_i2c_seq_state = I2C_SEQ_WRITE_DFP_GPIO1;
  end
  
  // Configure GPIO1 to set the DFP current limit.
  I2C_SEQ_WRITE_DFP_GPIO1: begin
    i2c_address         = FUSB307_BUS_ADDR;
    i2c_write_length    = 2;
    i2c_read_length     = 0;
    
    // Pull GPIO1 to GND to select 1.5A DFP current limit.
    if (usb_ufp_climit[2]) begin
      i2c_write_data  = {16'b0, 8'h01, FUSB307_REG_GPIO1_CFG};
    // Set GPIO2 to High-Z to select 900mA DFP current limit.
    end else begin
      i2c_write_data  = {16'b0, 8'h00, FUSB307_REG_GPIO1_CFG};
    end

    next_i2c_start = ~i2c_busy;
    if (i2c_busy_end) next_i2c_seq_state = I2C_SEQ_WRITE_DFP_GPIO2;
  end
  
  // Configure GPIO2 to set the DFP current limit.
  I2C_SEQ_WRITE_DFP_GPIO2: begin
    i2c_address         = FUSB307_BUS_ADDR;
    i2c_write_length    = 2;
    i2c_read_length     = 0;
    
    // Set GPIO2 to High-Z to disable the 3.0A DFP current limit.
    i2c_write_data      = {16'b0, 8'h00, FUSB307_REG_GPIO2_CFG};

    next_i2c_start = ~i2c_busy;
    if (i2c_busy_end) next_i2c_seq_state = I2C_SEQ_WRITE_DFP_ROLECTL;
  end
  
  // Configure the DFP port controller role.
  I2C_SEQ_WRITE_DFP_ROLECTL: begin
    i2c_address         = FUSB307_BUS_ADDR;
    i2c_write_length    = 2;
    i2c_read_length     = 0;

    // Configure for 1.5A DFP current if the UFP reports 3.0A, otherwise fallback to 900mA. 
    i2c_write_data  = {16'b0, (usb_ufp_climit[2]) ? 8'h15 : 8'h05, FUSB307_REG_ROLECTL};
    
    next_i2c_start = ~i2c_busy;
    if (i2c_busy_end) next_i2c_seq_state = I2C_SEQ_COMMAND_DFP_SINK;
  end

  // Instruct the DFP to begin sourcing power.
  I2C_SEQ_COMMAND_DFP_SINK: begin
    i2c_address         = FUSB307_BUS_ADDR;
    i2c_write_length    = 2;
    i2c_read_length     = 0;
    i2c_write_data      = {16'b0, 8'h99, FUSB307_REG_COMMAND};
    //i2c_write_data      = {16'b0, 8'h61, FUSB307_REG_PWRCTRL};

    next_i2c_start = ~i2c_busy;
    if (i2c_busy_end) next_i2c_seq_state = I2C_SEQ_READ_DFP_PWRSTAT;
  end
  
  // Debug - check if we successfully enabled the DFP port.
  I2C_SEQ_READ_DFP_PWRSTAT: begin
    i2c_address         = FUSB307_BUS_ADDR;
    i2c_write_length    = 1;
    i2c_read_length     = 3;
    i2c_write_data      = {24'b0, FUSB307_REG_CCSTAT};

    next_i2c_start = ~i2c_busy;
    if (i2c_busy_end) begin
      next_i2c_seq_delay = 100000;
      next_i2c_seq_state = I2C_SEQ_DONE;
    end
  end

  I2C_SEQ_DONE: begin
  end
      
  endcase
end

always @(posedge clk or posedge i2c_rst) begin
  if (i2c_rst) begin
    i2c_seq_state                  <= I2C_SEQ_DELAY;
    i2c_seq_delay                  <= 500;
    i2c_start                      <= 0;
  end
  else begin
    i2c_seq_state <= next_i2c_seq_state;
    i2c_seq_delay <= next_i2c_seq_delay;
    i2c_start     <= next_i2c_start;

    // Latch the read data into a register.
    if (i2c_busy) begin
        if (i2c_seq_state == I2C_SEQ_READ_UFP_ADC1) mxl7704_adc1 <= i2c_read_data[7:0];
        if (i2c_seq_state == I2C_SEQ_READ_UFP_ADC2) mxl7704_adc2 <= i2c_read_data[7:0];
    end
  end
end

assign debug = { i2c_read_data[15:8],
        (mxl7704_adc2[5]),
        (mxl7704_adc2[4]),
        next_i2c_start,
        i2c_busy,
        i2c_complete,
        i2c_no_response,
        i2c_scl_in,
        i2c_sda_in
};

endmodule
