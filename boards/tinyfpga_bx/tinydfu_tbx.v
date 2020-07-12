/*
 *  TinyDFU Bootloader for the TinyFPGA BX.
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
    wire clk = clkdiv[1];
    reg [1:0] clkdiv = 0;

    wire clk_locked;

    // Use an icepll generated pll
    pll pll48( .clock_in(pin_clk), .clock_out(clk_48mhz), .locked( clk_locked ) );
    always @(posedge clk_48mhz) clkdiv <= clkdiv + 1;

    //////////////////////////
    // LED Patterns
    //////////////////////////
    reg [31:0] led_counter = 0;
    always @(posedge clk) begin
        led_counter <= led_counter + 1;
    end

    // Double blink when idle.
    wire [2:0] led_blinkstate  = led_counter[22:20];
    wire led_idle = (led_blinkstate == 3) || (led_blinkstate == 5);

    // Fadepulse during programming.
    wire [4:0] led_pwmval = led_counter[23] ? led_counter[22:18] : (5'b11111 - led_counter[22:18]);
    wire led_busy = (led_counter[17:13] >= led_pwmval);

    // Select the LED pattern by DFU state.
    wire [7:0] dfu_state;
    assign pin_led = (dfu_state == 'h00) ? ~led_idle : (dfu_state == 'h02) ? led_idle : led_busy;

    //////////////////////////
    // Reset and Multiboot
    //////////////////////////
    reg user_bootmode = 1'b0;
    reg [15:0] reset_delay = 12000;
    reg [31:0] boot_delay = (12000000 * 3); // Give the user 3 seconds to enumerate USB.
    wire usb_reset;
    always @(posedge clk) begin
        // Stay in the bootloader if USB enumeration completes before boot_delay times out.
        if (clk_locked && reset_delay) reset_delay <= reset_delay - 1;
        if (clk_locked && boot_delay) boot_delay <= boot_delay - 1;
        if (dfu_state != 8'h00) user_bootmode <= 1'b1;
        
        // If we find outself in appDETACH and a USB reset occurs - start the user image.
        if (dfu_state == 8'h01 && usb_reset) begin
            user_bootmode <= 1'b0; 
            boot_delay <= 0;
        end
    end
  
    SB_WARMBOOT warmboot_inst (
        .S1(1'b0),
        .S0(1'b1),
        .BOOT(~user_bootmode && (boot_delay == 0))
    );

    //////////////////////////
    // USB DFU Device
    //////////////////////////
    wire usb_p_tx;
    wire usb_n_tx;
    wire usb_p_rx;
    wire usb_n_rx;
    wire usb_tx_en;

    // USB DFU - this instanciates the entire USB device.
    usb_dfu_core dfu (
        .clk_48mhz  (clk_48mhz),
        .clk        (clk),
        .reset      (reset_delay != 0),

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

        .dfu_state( dfu_state ),
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
