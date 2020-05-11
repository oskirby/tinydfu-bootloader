module usb_spiflash_bridge #(
  parameter SECTOR_SIZE = 4096,
  parameter PAGE_SIZE = 256
) (
  input clk,
  input reset,

  ////////////////////
  // SPI pin interface
  ////////////////////
  output reg spi_csel = 1'b1,
  output reg spi_clk  = 1'b0,
  output reg spi_mosi = 1'b0,
  input spi_miso,

  ////////////////////
  // control interface
  ////////////////////
  input [15:0] address,         // Flash page address

  ////////////////////
  // data read interfce
  ////////////////////
  input rd_request,             // High to request reading from the flash device.
  input rd_data_free,           // High whenever the upstream can accept more data.
  output rd_data_put,           // High whenever data is flowing from the flash device.
  output reg [7:0] rd_data = 0, // Output data bus when reading.

  ////////////////////
  // data write interfce
  ////////////////////
  input wr_request,             // High to request writing to the flash device.
  output wr_busy,               // High to indicate that an erase/program is in progress.
  input wr_data_avail,          // High whenever the upstream has more data to write.
  output wr_data_get,           // High whenever data is flowing into the flash device.
  input [7:0] wr_data,          // Input data bus when writing.

  output [7:0] debug
);

  // SPI flash geometry.
  localparam SECTOR_PAGE_COUNT = (SECTOR_SIZE / PAGE_SIZE);
  localparam SECTOR_BITS = $clog2(SECTOR_SIZE);
  localparam PAGE_BITS   = $clog2(PAGE_SIZE);

  // SPI flash commands
  localparam FLASH_CMD_WRITE_ENABLE         = 8'h06;
  localparam FLASH_CMD_VOLATILE_SR_WE       = 8'h50;
  localparam FLASH_CMD_WRITE_DISABLE        = 8'h04;
  localparam FLASH_CMD_RELEASE_POWER_DOWN   = 8'hAB;
  localparam FLASH_CMD_MANUFACTURER_ID      = 8'h90;
  localparam FLASH_CMD_JEDEC_ID             = 8'h9F;
  localparam FLASH_CMD_READ_UNIQUE_ID       = 8'h4B;
  localparam FLASH_CMD_READ_DATA            = 8'h03;
  localparam FLASH_CMD_FAST_READ            = 8'h0B;
  localparam FLASH_CMD_PAGE_PROGRAM         = 8'h02;
  localparam FLASH_CMD_SECTOR_ERASE         = 8'h20;
  localparam FLASH_CMD_BLOCK_ERASE_32K      = 8'h52;
  localparam FLASH_CMD_BLOCK_ERASE_64K      = 8'hD8;
  localparam FLASH_CMD_CHIP_ERASE           = 8'h60; // And also 'hC7 ??
  localparam FLASH_CMD_READ_SR1             = 8'h05;
  localparam FLASH_CMD_WRITE_SR1            = 8'h01;
  localparam FLASH_CMD_READ_SR2             = 8'h35;
  localparam FLASH_CMD_WRITE_SR2            = 8'h31;
  localparam FLASH_CMD_READ_SR3             = 8'h15;
  localparam FLASH_CMD_WRITE_SR3            = 8'h11;
  localparam FLASH_CMD_READ_SFDP            = 8'h5A;
  localparam FLASH_CMD_ERASE_SECURITY       = 8'h44;
  localparam FLASH_CMD_PROGRAM_SECURITY     = 8'h42;
  localparam FLASH_CMD_READ_SECURITY        = 8'h48;
  localparam FLASH_CMD_GLOBAL_BLOCK_LOCK    = 8'h7E;
  localparam FLASH_CMD_GLOBAL_BLOCK_UNLOCK  = 8'h98;
  localparam FLASH_CMD_READ_BLOCK_LOCK      = 8'h3D;
  localparam FLASH_CMD_SINGLE_BLOCK_LOCK    = 8'h36;
  localparam FLASH_CMD_SINGLE_BLOCK_UNLOCK  = 8'h39;
  localparam FLASH_CMD_ERASE_SUSPEND        = 8'h75;
  localparam FLASH_CMD_ERASE_RESUME         = 8'h7A;
  localparam FLASH_CMD_POWER_DOWN           = 8'hB9;
  localparam FLASH_CMD_ENABLE_RESET         = 8'h66;
  localparam FLASH_CMD_RESET_DEVICE         = 8'h99;

  // SPI flash state machine
  localparam FLASH_STATE_IDLE = 0;
  localparam FLASH_STATE_READ_COMMAND = 1;
  localparam FLASH_STATE_READ_DATA = 2;
  localparam FLASH_STATE_ERASE_ENABLE = 3;
  localparam FLASH_STATE_ERASE_COMMAND = 4;
  localparam FLASH_STATE_ERASE_BUSY = 5;
  localparam FLASH_STATE_WRITE_ENABLE = 6;
  localparam FLASH_STATE_WRITE_COMMAND = 7;
  localparam FLASH_STATE_WRITE_DATA = 8;
  localparam FLASH_STATE_WRITE_BUSY = 9;
  
  reg [3:0] flash_state = FLASH_STATE_IDLE;
  reg [3:0] flash_state_next = FLASH_STATE_IDLE;

  reg [23:0] byte_address = 0;
  wire [SECTOR_BITS-PAGE_BITS-1:0] page_num = byte_address[SECTOR_BITS-1:PAGE_BITS];
  wire byte_addr_latch = 0;
  always @(posedge clk) if (byte_addr_latch) byte_address <= (address * PAGE_SIZE);
  
  rising_edge_detector detect_address_latch (
    .clk(clk),
    .in(wr_request || rd_request),
    .out(byte_addr_latch)
  );

  assign debug[0] = (flash_state == FLASH_STATE_ERASE_BUSY);
  assign debug[1] = wr_data_avail;
  assign debug[2] = wr_data_get;

  reg [7:0] command_bits;       // Number of bits to transfer in the command stage.
  reg [3:0] command_csel;       // Number of clocks to hold CSEL high after the command (or zero to leave CSEL active).
  reg [63:0] command_buf;       // Command data to be shifted out of MOSI.
  reg command_start;            // Pulsed high to start a new command.

  reg [63:0] read_buf = 0;
  reg [63:0] write_buf = 0;
  reg [7:0] bitcount = 0;
  reg [3:0] cseldelay = 0;
  wire transfer_busy = (bitcount || cseldelay);
  wire transfer_done;
  
  assign rd_data_put = (flash_state == FLASH_STATE_READ_DATA) && transfer_done;
  assign wr_data_get = wr_data_avail && (wr_cache_write_addr < (PAGE_SIZE-1));
  assign wr_busy = (flash_state >= FLASH_STATE_ERASE_ENABLE);
  always @* rd_data <= read_buf[7:0];

  falling_edge_detector detect_transfer_done (
    .clk(clk),
    .in(transfer_busy),
    .out(transfer_done)
  );

  // Data cache, holds a page of data to be written.
  reg [7:0]   wr_cache_read_addr = 0;
  reg [7:0]   wr_cache_write_addr = 0;
  wire [15:0] wr_cache_read_data;
  SB_RAM40_4K #(
    .READ_MODE(32'd1),  // Select 512x8 mode.
    .WRITE_MODE(32'd1), // Select 512x8 mode.
  ) wr_cache_mem(
    // Read data port (cache -> SPI flash)
    .RDATA(wr_cache_read_data),
    .RADDR(wr_cache_read_addr),
    .RCLK(clk),
    .RCLKE(1'b1),
    .RE(1'b1),

    // Write data port (USB endpoint -> cache),
    .WDATA({8'b0, wr_data}),
    .WADDR(wr_cache_write_addr),
    .WCLK(clk),
    .WCLKE(wr_request),
    .WE(wr_data_get)
  );

  always @(posedge clk) begin
    if (flash_state == FLASH_STATE_WRITE_BUSY) wr_cache_write_addr <= 0;
    else if (wr_data_get) wr_cache_write_addr <= wr_cache_write_addr + 1;
  end

  always @* begin
    command_start <= 0;
    command_bits <= 0;
    command_csel <= 2;
    command_buf <= 0;

    case (flash_state)
      FLASH_STATE_IDLE : begin
        if (rd_request) begin
          flash_state_next <= FLASH_STATE_READ_COMMAND;
          command_start <= 1;
          command_bits <= 40;
          command_csel <= 0;
          command_buf  <= {24'b0, FLASH_CMD_FAST_READ, byte_address, 8'b0};
        
        end else if (wr_request) begin
          // We need to erase when starting a new sector.
          flash_state_next <= (page_num == 0) ? FLASH_STATE_ERASE_ENABLE : FLASH_STATE_WRITE_ENABLE;
          command_start <= 1;
          command_bits  <= 8;
          command_buf   <= {56'b0, FLASH_CMD_WRITE_ENABLE};

        end else begin
          flash_state_next <= FLASH_STATE_IDLE;
        end
      end

      FLASH_STATE_READ_COMMAND : begin
        if (!transfer_busy) begin
          flash_state_next <= FLASH_STATE_READ_DATA;
        end else begin
          flash_state_next <= FLASH_STATE_READ_COMMAND;
        end
      end

      FLASH_STATE_READ_DATA : begin
        if (!rd_request) begin
          flash_state_next <= FLASH_STATE_IDLE;
        end else begin
          flash_state_next <= FLASH_STATE_READ_DATA;
        end
      end

      FLASH_STATE_ERASE_ENABLE : begin
        if (!transfer_busy) begin
          flash_state_next <= FLASH_STATE_ERASE_COMMAND;
          command_start <= 1;
          command_bits  <= 32;
          command_csel  <= 4;
          command_buf   <= {32'b0, FLASH_CMD_SECTOR_ERASE, byte_address};
        end else begin
          flash_state_next <= FLASH_STATE_ERASE_ENABLE;
        end
      end

      FLASH_STATE_ERASE_COMMAND : begin
        if (!transfer_busy) begin
          flash_state_next <= FLASH_STATE_ERASE_BUSY;
          command_start <= 1;
          command_bits  <= 16;
          command_buf   <= {48'b0, FLASH_CMD_READ_SR1, 8'b0};
        end else begin
          flash_state_next <= FLASH_STATE_ERASE_COMMAND;
        end
      end

      FLASH_STATE_ERASE_BUSY : begin
        if (transfer_busy) begin
          flash_state_next <= FLASH_STATE_ERASE_BUSY;
        end else if (read_buf[0]) begin
          flash_state_next <= FLASH_STATE_ERASE_BUSY;
          command_start <= 1;
          command_bits  <= 16;
          command_buf   <= {48'b0, FLASH_CMD_READ_SR1, 8'b0};
        end else begin
          flash_state_next <= FLASH_STATE_WRITE_ENABLE;
          command_start <= 1;
          command_bits  <= 8;
          command_buf   <= {56'b0, FLASH_CMD_WRITE_ENABLE};
        end
      end
    
      FLASH_STATE_WRITE_ENABLE : begin
        if (!transfer_busy) begin
          flash_state_next <= FLASH_STATE_WRITE_COMMAND;
          command_start <= 1;
          command_bits  <= 32;
          command_csel  <= 0;
          command_buf   <= {32'b0, FLASH_CMD_PAGE_PROGRAM, byte_address};
        end else begin
          flash_state_next <= FLASH_STATE_WRITE_ENABLE;
        end
      end
    
      FLASH_STATE_WRITE_COMMAND : begin
        if (!transfer_busy) begin
          flash_state_next <= FLASH_STATE_WRITE_DATA;
        end else begin
          flash_state_next <= FLASH_STATE_WRITE_COMMAND;
        end
      end

      FLASH_STATE_WRITE_DATA : begin
        // TODO: Detect EOF
        if (!transfer_busy && !wr_request && (wr_cache_read_addr == wr_cache_write_addr)) begin
          // TODO: Poll for write complete.
          flash_state_next <= FLASH_STATE_IDLE;
        end else begin
          flash_state_next <= FLASH_STATE_WRITE_DATA;
        end
      end

      default begin
        flash_state_next <= FLASH_STATE_IDLE;
      end
    endcase
  end

  always @(posedge clk) begin
    if (reset) begin
      flash_state <= FLASH_STATE_IDLE;
    end else begin
      flash_state <= flash_state_next;
    end
  end

  always @(posedge clk) begin
    // Start a new command by asserting CSEL and setting up counters.
    if (command_start) begin
      bitcount <= command_bits;
      cseldelay <= command_csel;
      write_buf <= command_buf;
      wr_cache_read_addr <= 0;
      spi_csel <= 1'b0;
      spi_clk <= 1'b0;
      spi_mosi <= command_buf[command_bits-1];
    end else if (flash_state == FLASH_STATE_IDLE) begin
      spi_csel <= 1'b1;
      spi_clk <= 1'b0;
      spi_mosi <= 1'b0;
    end

    // Transfer bits to and from the flash.
    else if (bitcount) begin
      if (spi_clk) begin
        spi_clk <= 1'b0;
        spi_mosi <= write_buf[bitcount-1];
      end else begin
        spi_clk <= 1'b1;
        read_buf <= {read_buf[62:0], spi_miso};
        bitcount <= bitcount - 1;
      end
    end else if (cseldelay) begin
      spi_csel <= 1'b1;
      spi_clk <= 1'b0;
      cseldelay <= cseldelay - 1;
    end

    // Transfer another byte if we have made it to the data stage.
    // This is hacky, but the data free signal needs a clock to propagate after an rd_data_put.
    if ((flash_state == FLASH_STATE_READ_DATA) && !bitcount && !rd_data_put && rd_data_free) bitcount <= 8;
    if ((flash_state == FLASH_STATE_WRITE_DATA) && !bitcount && (wr_cache_read_addr < wr_cache_write_addr)) begin
      write_buf <= {56'b0, wr_cache_read_data[7:0]};
      bitcount <= 8;
      wr_cache_read_addr <= wr_cache_read_addr + 1;
    end
  end
endmodule