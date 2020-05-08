/*
    USB Serial

    Wrapping usb/usb_uart_ice40.v to create a loopback.
*/

module usbserial_tbx (
        input  pin_clk,

        inout  pin_usb_p,
        inout  pin_usb_n,
        output pin_pu,

        output pin_red_led,
        output pin_grn_led,
        output pin_blu_led,

        output pin_16_cs,
        output pin_15_sck,
        output pin_14_mosi,
        input pin_17_miso,

        output [3:0] debug
    );

    wire clk_48mhz;

    wire clk_locked;

    // Use an icepll generated pll
    pll pll48( .clock_in(pin_clk), .clock_out(clk_48mhz), .locked( clk_locked ) );


    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    ////////
    //////// interface with iCE40 RGB LED driver
    ////////
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    wire [7:0] led_debug;

    // LED
    reg [22:0] ledCounter;
    always @(posedge clk_48mhz) begin
        ledCounter <= ledCounter + 1;
    end

    SB_RGBA_DRV RGBA_DRIVER (
        .CURREN(1'b1),
        .RGBLEDEN(1'b1),
        .RGB0PWM(ledCounter[ 22 ]),
        .RGB1PWM(led_debug[0]),
        .RGB2PWM(led_debug[1]),
        .RGB0(pin_red_led),
        .RGB1(pin_grn_led),
        .RGB2(pin_blu_led)
    );

    defparam RGBA_DRIVER.CURRENT_MODE = "0b1";
    defparam RGBA_DRIVER.RGB0_CURRENT = "0b000001";  // Blue
    defparam RGBA_DRIVER.RGB1_CURRENT = "0b000001";  // Red
    defparam RGBA_DRIVER.RGB2_CURRENT = "0b000001";  // Green
  


    // Generate reset signal
    reg [5:0] reset_cnt = 0;
    wire reset = ~reset_cnt[5];
    always @(posedge clk_48mhz)
        if ( clk_locked )
            reset_cnt <= reset_cnt + reset;

    // uart pipeline in
    wire [7:0] uart_in_data;
    wire       uart_in_valid;
    wire       uart_in_ready;

    // assign debug = { uart_in_valid, uart_in_ready, reset, clk_48mhz };

    wire usb_p_tx;
    wire usb_n_tx;
    wire usb_p_rx;
    wire usb_n_rx;
    wire usb_tx_en;

    // usb uart - this instanciates the entire USB device.
    usb_uart uart (
        .clk_48mhz  (clk_48mhz),
        .reset      (reset),

        // pins
        .pin_usb_p( pin_usb_p ),
        .pin_usb_n( pin_usb_n ),

        // SPI
        .spi_csel( pin_16_cs ),
        .spi_clk( pin_15_sck ),
        .spi_mosi( pin_14_mosi ),
        .spi_miso( pin_17_miso ),  

        // uart pipeline in
        .uart_in_data( uart_in_data ),
        .uart_in_valid( uart_in_valid ),
        .uart_in_ready( uart_in_ready ),

        .uart_out_data( uart_in_data ),
        .uart_out_valid( uart_in_valid ),
        .uart_out_ready( uart_in_ready  ),

        .debug( debug )
    );

    // USB Host Detect Pull Up
    assign pin_pu = 1'b1;

endmodule
