module i2c_master_tb #() ();
  localparam MAX_TRANSFER      = 2;
  localparam MAX_TRANSFER_SIZE = $clog2(MAX_TRANSFER+1);
  
  

  reg                          clk;
  reg                          rst;
  reg [6:0]                    i2c_address;
  reg [MAX_TRANSFER*8-1:0]     write_data;
  wire [MAX_TRANSFER*8-1:0]    read_data;
  reg                          start;
  reg [MAX_TRANSFER_SIZE-1:0]  write_transfer_length;
  reg [MAX_TRANSFER_SIZE-1:0]  read_transfer_length;
  wire                         busy;
  wire                         complete;
  wire                         no_response;
  wire [MAX_TRANSFER_SIZE-1:0] total_written;
  wire [MAX_TRANSFER_SIZE-1:0] total_read;

  reg                          i2c_scl_in;
  wire                         i2c_scl_drive_n;
  reg                          i2c_sda_in;
  wire                         i2c_sda_drive_n;
  
  i2c_master #(
    .MAX_TRANSFER_LENGTH (       2 ),
    .INPUT_FREQUENCY     ( 1000000 ),
    .FREQUENCY           (  100000 )
  ) i2c_master_inst (
    .clk                   ( clk                   ),
    .rst                   ( rst                   ),
    .i2c_address           ( i2c_address           ),
    .write_data            ( write_data            ),
    .read_data             ( read_data             ),
    .start                 ( start                 ),
    .write_transfer_length ( write_transfer_length ),
    .read_transfer_length  ( read_transfer_length  ),
    .busy                  ( busy                  ),
    .complete              ( complete              ),
    .no_response           ( no_response           ),
    .total_written         ( total_written         ),
    .total_read            ( total_read            ),
    .i2c_scl_in            ( i2c_scl_in            ),
    .i2c_scl_drive_n       ( i2c_scl_drive_n       ),
    .i2c_sda_in            ( i2c_sda_in            ),
    .i2c_sda_drive_n       ( i2c_sda_drive_n       )
  );


  localparam  CLOCK_PERIOD            = 100; // Clock period in ps
  localparam  INITIAL_RESET_CYCLES    = 10;  // Number of cycles to reset when simulation starts
  // Clock signal generator
  initial clk = 1'b1;
  always begin
    #(CLOCK_PERIOD / 2);
    clk = ~clk;
  end

  // Initial reset
  initial begin
    rst = 1'b1;
    repeat(INITIAL_RESET_CYCLES) @(posedge clk);
    rst = 1'b0;
  end

  // Test cycle
  initial begin
    i2c_address           = 0;
    write_data            = 0;
    start                 = 0;
    write_transfer_length = 0;
    read_transfer_length  = 0;

    repeat(100) @(posedge clk);
    i2c_address           = 7'h2D;
    write_transfer_length = 2;
    read_transfer_length  = 2;
    write_data            = 16'hF005;
    
    
    repeat(10) @(posedge clk);
    start = 1;
    
    @(posedge clk);
    start = 0;
    
    
  end

  reg slave_sda;
  
  // i2c_slave simulator
  initial begin
    slave_sda = 1;
    @(negedge i2c_scl_in);
    
    repeat(8) @(negedge i2c_scl_in);
    slave_sda = 0;
    @(negedge i2c_scl_in);
    slave_sda = 1;

    repeat(8) @(negedge i2c_scl_in);
    slave_sda = 0;
    @(negedge i2c_scl_in);
    slave_sda = 1;

    repeat(8) @(negedge i2c_scl_in);
    slave_sda = 0;
    @(negedge i2c_scl_in);
    slave_sda = 1;

    // for repeated start
    @(negedge i2c_scl_in);
    
    repeat(8) @(negedge i2c_scl_in);
    slave_sda = 0;
    @(negedge i2c_scl_in);
    slave_sda = 1;
    
    
  end
  always @(*) begin
    i2c_scl_in = i2c_scl_drive_n;
    i2c_sda_in = i2c_sda_drive_n & slave_sda;
  end

endmodule
  
  
  
