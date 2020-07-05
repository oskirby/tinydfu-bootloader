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

module usb_dfu (
  input  clk,
  input reset,
  input boot,

  // USB pins
  inout  pin_usb_p,
  inout  pin_usb_n,

  // SPI pins
  output spi_csel,
  output spi_clk,
  output spi_mosi,
  input spi_miso

  // DFU state and debug
  //output [7:0] dfu_state,
  //output [11:0] debug
);

    localparam c_user_start = 48'h0000_0200_0000;

    wire usb_p_tx;
    wire usb_n_tx;
    wire usb_p_rx;
    wire usb_n_rx;
    wire usb_tx_en;
    wire [7:0] dfu_state;
    wire [11:0] debug;
    wire clk_48mhz;
    //wire [3:0] debug;
    
     
    clk_wiz_0 pll
    (
        .clk_out1(clk_48mhz),
        .clk_in1(clk)
    );

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

         // DFU state and debug
        .dfu_state(dfu_state),
        .debug( debug )
    );

    
  
  wbicapetwo #(
    .G_START_ADDRESS(c_user_start)
    )                        
  wbicapetwo_inst (
    .clk(clk_48mhz),
    .reset(reset),
    .boot(boot)
  );

    wire usb_p_in;
    wire usb_n_in;

    assign usb_p_rx = usb_tx_en ? 1'b1 : usb_p_in;
    assign usb_n_rx = usb_tx_en ? 1'b0 : usb_n_in;
    
    assign pin_usb_p = usb_tx_en ? usb_p_tx : 1'bz;
    assign usb_p_in = pin_usb_p;
    
    assign pin_usb_n = usb_tx_en ? usb_n_tx : 1'bz;
    assign usb_n_in = pin_usb_n;

endmodule
