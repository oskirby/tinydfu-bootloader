/*
 i2c master
 
 */


module i2c_master #(
  parameter MAX_TRANSFER_LENGTH = 1,
  parameter INPUT_FREQUENCY     = 12000000,
  parameter FREQUENCY           = 400000,
  // non user-editable
  parameter TRANSFER_LENGTH_SIZE = $clog2(MAX_TRANSFER_LENGTH+1),
  parameter INTERFACE_SIZE       = (MAX_TRANSFER_LENGTH * 8)
)(
  input wire                            clk,
  input wire                            rst,
    
  input wire [6:0]                      i2c_address,
    
  input wire [INTERFACE_SIZE-1:0]       write_data,
  output reg [INTERFACE_SIZE-1:0]       read_data,

  input wire                            start,
  input wire [TRANSFER_LENGTH_SIZE-1:0] write_transfer_length,
  input wire [TRANSFER_LENGTH_SIZE-1:0] read_transfer_length,

  output wire                           busy,
  output reg                            complete,
  output reg                            no_response,
  
  output reg [TRANSFER_LENGTH_SIZE-1:0] total_written,
  output reg [TRANSFER_LENGTH_SIZE-1:0] total_read,

  input wire                            i2c_scl_in,
  output reg                            i2c_scl_drive_n,
  input wire                            i2c_sda_in,
  output reg                            i2c_sda_drive_n
  );

  initial begin
    read_data       = 0;
    complete        = 0;
    no_response     = 0;
  
    total_written   = 0;
    total_read      = 0;
    i2c_scl_drive_n = 1;
    i2c_sda_drive_n = 1;
  end

  
  localparam DIV_COUNT = (INPUT_FREQUENCY / FREQUENCY);
  localparam DIV_COUNT_SIZE = $clog2(DIV_COUNT+1);

  reg [DIV_COUNT_SIZE-1:0]  div_counter;
  wire                      output_clk;
  wire                      output_clk_drive;
  wire                      output_clk_read;
  always @(posedge clk) begin
    if (div_counter) begin
      div_counter <= div_counter - 1;
    end
    else begin
      div_counter    <= DIV_COUNT;
    end
  end
  assign output_clk       = div_counter == 0;
  assign output_clk_read = div_counter == (DIV_COUNT/2);
  localparam DIV_COUNT_1_4 = (DIV_COUNT>>2);
  localparam DIV_COUNT_3_4 = ((DIV_COUNT*3) >> 2);
  assign output_clk_drive = div_counter > DIV_COUNT_1_4 && div_counter <= DIV_COUNT_3_4;

  
  localparam I2C_STATE_IDLE            = 16'b0000000000000001;
  localparam I2C_STATE_START           = 16'b0000000000000010;
  localparam I2C_STATE_WR_DEVADDR      = 16'b0000000000000100;
  localparam I2C_STATE_WR_DIR          = 16'b0000000000001000;
  localparam I2C_STATE_WR_DEVADDR_ACK  = 16'b0000000000010000;
  localparam I2C_STATE_WR_DATA         = 16'b0000000000100000;
  localparam I2C_STATE_WR_DATA_ACK     = 16'b0000000001000000;
  localparam I2C_STATE_WR_STOP         = 16'b0000000010000000;
  localparam I2C_STATE_REPEAT_START    = 16'b0000000100000000;
  localparam I2C_STATE_RD_DEVADDR      = 16'b0000001000000000;
  localparam I2C_STATE_RD_DIR          = 16'b0000010000000000;
  localparam I2C_STATE_RD_DEVADDR_ACK  = 16'b0000100000000000;
  localparam I2C_STATE_RD_DATA         = 16'b0001000000000000;
  localparam I2C_STATE_RD_DATA_ACK     = 16'b0010000000000000;
  localparam I2C_STATE_RD_DATA_NACK    = 16'b0100000000000000;
  localparam I2C_STATE_STOP            = 16'b1000000000000000;
  reg [15:0] i2c_state;
  reg [15:0] next_i2c_state;

  reg [3:0]                        bit_num;
  reg [3:0]                        next_bit_num;
  reg [TRANSFER_LENGTH_SIZE+1-1:0] byte_num;
  reg [TRANSFER_LENGTH_SIZE+1-1:0] next_byte_num;

  // latch the data in for the transfer
  reg                              in_progress                   = 0;
  reg                              next_in_progress = 0;
  reg                              last_start                    = 0;
  reg [6:0]                        latched_i2c_address           = 0;
  reg [INTERFACE_SIZE-1:0]         latched_write_data            = 0;
  reg [TRANSFER_LENGTH_SIZE-1:0]   latched_write_transfer_length = 0;
  reg [TRANSFER_LENGTH_SIZE-1:0]   latched_read_transfer_length  = 0;
  reg                              request_start;
  always @(posedge clk or posedge rst) begin
    if (rst) begin
      last_start                    <= 0;
      request_start                 <= 0;
      latched_i2c_address           <= 0;
      latched_write_data            <= 0;
      latched_write_transfer_length <= 0;
      latched_read_transfer_length  <= 0;
    end
    else begin
      last_start <= start;
      if (start & !last_start) begin
        latched_i2c_address           <= i2c_address;
        latched_write_data            <= write_data;
        latched_write_transfer_length <= write_transfer_length;
        latched_read_transfer_length  <= read_transfer_length;
        request_start                 <= 1;
      end
      else if (in_progress) begin
        request_start <= 0;
      end
    end
  end
  assign busy = start | in_progress | request_start;

  reg latched_i2c_sda_in = 1;
  always @(posedge clk) if (output_clk_read) latched_i2c_sda_in <= i2c_sda_in;

  reg start_signal         = 1;
  reg latched_start_signal = 1;
  always @(posedge clk) if (output_clk_read) latched_start_signal <= start_signal;
  
  reg next_complete;
  reg next_no_response;
  reg [TRANSFER_LENGTH_SIZE-1:0] next_total_written;
  reg [TRANSFER_LENGTH_SIZE-1:0] next_total_read;
  reg [INTERFACE_SIZE-1:0]       next_read_data = 0;

  reg [6:0]                      working_i2c_address;
  reg [6:0]                      next_working_i2c_address;
  reg [INTERFACE_SIZE-1:0]       working_write_data;
  reg [INTERFACE_SIZE-1:0]       next_working_write_data;
  reg [INTERFACE_SIZE-1:0]       working_read_data;
  reg [INTERFACE_SIZE-1:0]       next_working_read_data;

  wire [INTERFACE_SIZE-1:0]       bit_next_working_write_data;
  wire [INTERFACE_SIZE-1:0]       byte_next_working_write_data;
  wire [INTERFACE_SIZE-1:0]       bit_next_working_read_data;
  wire [INTERFACE_SIZE-1:0]       byte_next_working_read_data;
  generate
    if (MAX_TRANSFER_LENGTH > 1) begin
      assign bit_next_working_write_data  = { working_write_data[INTERFACE_SIZE-1:8], working_write_data[6:0], 1'd0 };
      assign byte_next_working_write_data = { 8'd0, working_write_data[INTERFACE_SIZE-1:8] };
      
      assign bit_next_working_read_data  = { working_read_data[INTERFACE_SIZE-1:8], latched_i2c_sda_in, working_read_data[7:1] };
      assign byte_next_working_read_data = { working_read_data[INTERFACE_SIZE-9:0], 8'd0 };
    end
    else begin
      assign bit_next_working_write_data  = { working_write_data[6:0], 1'd0 };
      assign byte_next_working_write_data = { 8'd0 };
      
      assign bit_next_working_read_data = { latched_i2c_sda_in, working_read_data[7:1] };
      assign byte_next_working_read_data = 8'd0;
    end
  endgenerate

  
  always @(*) begin
    next_i2c_state           = i2c_state;
    next_complete            = complete;
    next_no_response         = no_response;

    next_bit_num             = bit_num;
    next_byte_num            = byte_num;
    i2c_sda_drive_n          = 1;
    i2c_scl_drive_n          = 1;

    next_working_read_data   = working_read_data;
    next_working_i2c_address = working_i2c_address;
    next_working_write_data  = working_write_data;
    next_total_written       = total_written;
    next_total_read          = total_read;
    next_read_data           = read_data;

    start_signal             = 1;
    next_in_progress         = 0;
    
    case (i2c_state)
    I2C_STATE_IDLE           : begin
      next_in_progress     = 0;
      i2c_sda_drive_n = 1;
      i2c_scl_drive_n = 1;
      
      if (request_start) begin
        next_working_i2c_address = latched_i2c_address;

        next_complete      = 0;
        next_no_response   = 0;
        next_total_written = 0;
        next_total_read    = 0;
        
        next_working_write_data = latched_write_data;
        
        if (!(latched_write_transfer_length) && !(latched_read_transfer_length)) begin
          next_i2c_state = I2C_STATE_IDLE;
        end
        else begin
          if (latched_write_transfer_length) next_i2c_state = I2C_STATE_START;
          else                               next_i2c_state = I2C_STATE_REPEAT_START;
        end
      end
    end
    
    I2C_STATE_START          : begin
      next_in_progress         = 1;
      
      i2c_scl_drive_n          = latched_start_signal | output_clk_drive;
      i2c_sda_drive_n          = latched_start_signal;
      start_signal             = 0;
      
      next_working_i2c_address = latched_i2c_address;
      
      next_i2c_state           = I2C_STATE_WR_DEVADDR;
      next_bit_num             = 0;
      next_byte_num            = 0;
    end
    
    I2C_STATE_WR_DEVADDR     : begin
      next_in_progress         = 1;
      
      i2c_sda_drive_n          = working_i2c_address[6];
      next_working_i2c_address = { working_i2c_address[5:0], 1'b0 };
      i2c_scl_drive_n          = output_clk_drive;
      next_bit_num             = bit_num + 1;
      
      if (bit_num >= 6) next_i2c_state = I2C_STATE_WR_DIR;
    end
    
    I2C_STATE_WR_DIR         : begin
      next_in_progress     = 1;

      i2c_sda_drive_n = 0;
      i2c_scl_drive_n = output_clk_drive;

      next_i2c_state = I2C_STATE_WR_DEVADDR_ACK;
    end
    
    I2C_STATE_WR_DEVADDR_ACK : begin
      next_in_progress     = 1;

      i2c_sda_drive_n = 1;
      i2c_scl_drive_n = output_clk_drive;
      

      if (!latched_i2c_sda_in) begin
        next_i2c_state = I2C_STATE_WR_DATA;
        next_bit_num   = 0;
        next_byte_num  = 0;
      end
      else begin 
        next_no_response = 1;
        next_i2c_state   = I2C_STATE_STOP;
      end
    end 
    
    I2C_STATE_WR_DATA        : begin
      next_in_progress     = 1;

      i2c_sda_drive_n = working_write_data[7];
      i2c_scl_drive_n = output_clk_drive;

      next_working_write_data = bit_next_working_write_data;
      
      next_bit_num    = next_bit_num + 1;
      if (bit_num >= 7) begin
        next_i2c_state = I2C_STATE_WR_DATA_ACK;
      end
    end

    
    I2C_STATE_WR_DATA_ACK    : begin
      next_in_progress = 1;

      i2c_sda_drive_n  = 1;
      i2c_scl_drive_n  = output_clk_drive;

      next_bit_num     = 0;
      next_byte_num    = byte_num + 1;

      // if either completed or not-acknowledged
      if (byte_num >= latched_write_transfer_length-1 || latched_i2c_sda_in) begin
        start_signal = 0;
        if (latched_read_transfer_length) begin
          next_i2c_state = I2C_STATE_WR_STOP;
        end
        else begin
          next_complete  = 1;
          next_i2c_state = I2C_STATE_STOP;
        end
      end
      else begin // acknowledged with more data
        next_i2c_state = I2C_STATE_WR_DATA;
        next_working_write_data = byte_next_working_write_data;
      end
    end

    I2C_STATE_WR_STOP        : begin
      next_in_progress         = 1;

      i2c_scl_drive_n          = latched_start_signal | output_clk_drive;
      i2c_sda_drive_n          = latched_start_signal;
      start_signal             = 1;
      
      next_i2c_state           = I2C_STATE_REPEAT_START;
    end
    
    I2C_STATE_REPEAT_START   : begin
      next_in_progress         = 1;

      i2c_scl_drive_n          = latched_start_signal | output_clk_drive;
      i2c_sda_drive_n          = latched_start_signal;
      start_signal             = 0;
      
      next_working_i2c_address = latched_i2c_address;
      next_total_written       = byte_num;

      next_i2c_state           = I2C_STATE_RD_DEVADDR;
      next_bit_num             = 0;
      next_byte_num            = 0;
    end
    
    
    I2C_STATE_RD_DEVADDR     : begin
      next_in_progress              = 1;

      i2c_sda_drive_n          = working_i2c_address[6];
      next_working_i2c_address = { working_i2c_address[5:0], 1'b0 };
      i2c_scl_drive_n          = output_clk_drive;
      next_bit_num             = bit_num + 1;
      
      if (bit_num >= 6) next_i2c_state = I2C_STATE_RD_DIR;
    end
    
    I2C_STATE_RD_DIR         : begin
      next_in_progress     = 1;

      i2c_sda_drive_n = 1;
      i2c_scl_drive_n = output_clk_drive;

      next_i2c_state = I2C_STATE_RD_DEVADDR_ACK;
    end
    
    I2C_STATE_RD_DEVADDR_ACK : begin
      next_in_progress = 1;

      i2c_sda_drive_n = 1;
      i2c_scl_drive_n = output_clk_drive;
      
      if (!latched_i2c_sda_in) begin
        next_i2c_state = I2C_STATE_RD_DATA;
        next_bit_num   = 0;
        next_byte_num  = 0;
      end
      else begin 
        next_no_response = 1;
        next_i2c_state   = I2C_STATE_STOP;
      end
    end
    
    I2C_STATE_RD_DATA        : begin
      next_in_progress            = 1;

      i2c_sda_drive_n        = 1;
      i2c_scl_drive_n        = output_clk_drive;

      next_working_read_data = bit_next_working_read_data;
      
      next_bit_num           = next_bit_num + 1;
      
      if (bit_num >= 7) begin
        next_byte_num = byte_num + 1;
        
        if (byte_num >= latched_read_transfer_length-1) begin
          next_bit_num  = 0;
          next_i2c_state = I2C_STATE_RD_DATA_NACK;
        end
        else begin
          next_bit_num  = 0;
          next_i2c_state = I2C_STATE_RD_DATA_ACK;
        end
      end
    end
    
    I2C_STATE_RD_DATA_ACK    : begin
      next_in_progress     = 1;

      i2c_sda_drive_n = 0;
      i2c_scl_drive_n = output_clk_drive;

      next_working_read_data = byte_next_working_read_data;
      next_i2c_state = I2C_STATE_RD_DATA;
    end

    I2C_STATE_RD_DATA_NACK   : begin
      next_in_progress     = 1;

      i2c_sda_drive_n = 1;
      i2c_scl_drive_n = output_clk_drive;
      next_read_data  = working_read_data;
      next_total_read = byte_num;

      next_i2c_state = I2C_STATE_STOP;
    end
    
    I2C_STATE_STOP           : begin
      next_in_progress    = 1;

      i2c_scl_drive_n          = ~latched_start_signal | output_clk_drive;
      i2c_sda_drive_n          = ~latched_start_signal;
      start_signal             = 0;

      next_i2c_state = I2C_STATE_IDLE;
    end

    default: next_i2c_state = I2C_STATE_IDLE;
    
    endcase
  end

  always @(posedge clk or posedge rst) begin
    if (rst) begin
      i2c_state           <= I2C_STATE_IDLE;
      in_progress         <= 0;
      complete            <= 0;
      no_response         <= 0;

      bit_num             <= 0;
      byte_num            <= 0;

      working_read_data   <= 0;
      working_i2c_address <= 0;
      working_write_data  <= 0;

      total_written       <= 0;
      total_read          <= 0;
      read_data           <= 0;
    end
    else begin
      if (output_clk) begin
        i2c_state           <= next_i2c_state;
        in_progress         <= next_in_progress;
        complete            <= next_complete;
        no_response         <= next_no_response;
      
        bit_num             <= next_bit_num;
        byte_num            <= next_byte_num;

        working_read_data   <= next_working_read_data;
        working_i2c_address <= next_working_i2c_address;
        working_write_data  <= next_working_write_data;

        total_written       <= next_total_written;
        total_read          <= next_total_read;
        read_data           <= next_read_data;
      end
    end
  end
  
  
endmodule
