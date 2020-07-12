/*
    TinyDFU Bootloader

    Wrapping usb/usb_dfu_ice40.v.
*/

module tinydfu_tbx (
        input  pin_clk,

        inout  pin_usb_p,
        inout  pin_usb_n,
        output pin_pu,

        output pin_led,

        output pin_16_cs,
        output pin_15_sck,
        output pin_14_mosi,
        input pin_17_miso,

        output [3:0] debug
    );

    wire clk_48mhz;
    reg  clk_24mhz = 0;
    reg  clk = 0;

    wire clk_locked;

    // Use an icepll generated pll
    pll pll48( .clock_in(pin_clk), .clock_out(clk_48mhz), .locked( clk_locked ) );
    always @(posedge clk_48mhz) clk_24mhz = ~clk_24mhz;
    always @(posedge clk_24mhz) clk = ~clk;

    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    ////////
    //////// interface with iCE40 RGB LED driver
    ////////
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    wire [7:0] led_debug;

    // LED
    reg [22:0] led_counter;
    always @(posedge clk) begin
        led_counter <= led_counter + 1;
    end
    assign pin_led = led_counter[20];

    // Generate reset signal
    reg [5:0] reset_cnt = 0;
    wire reset = ~reset_cnt[5];
    always @(posedge clk)
        if ( clk_locked )
            reset_cnt <= reset_cnt + reset;

    wire usb_p_tx;
    wire usb_n_tx;
    wire usb_p_rx;
    wire usb_n_rx;
    wire usb_tx_en;

    // USB DFU - this instanciates the entire USB device.
    usb_dfu_core dfu (
        .clk_48mhz  (clk_48mhz),
        .clk        (clk),
        .reset      (reset),

        // USB signals
        .usb_p_tx( usb_p_tx ),
        .usb_n_tx( usb_n_tx ),
        .usb_p_rx( usb_p_rx ),
        .usb_n_rx( usb_n_rx ),
        .usb_tx_en( usb_tx_en ),

        // SPI
        .spi_csel( pin_16_cs ),
        .spi_clk( pin_15_sck ),
        .spi_mosi( pin_14_mosi ),
        .spi_miso( pin_17_miso ),  

        .debug( debug )
    );

    // USB Physical interface
    usb_phy_ice40 phy (
        .clk_48mhz  (clk_48mhz),
        .reset      (usb_reset),

        .pin_usb_p (pin_usb_p),
        .pin_usb_n (pin_usb_n),

        .usb_p_tx( usb_p_tx ),
        .usb_n_tx( usb_n_tx ),
        .usb_p_rx( usb_p_rx ),
        .usb_n_rx( usb_n_rx ),
        .usb_tx_en( usb_tx_en ),
    );

    // USB Host Detect Pull Up
    assign pin_pu = 1'b1;

endmodule
