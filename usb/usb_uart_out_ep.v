/*
    usb_uart_out_ep

    This is the endpoint to uart translator.  Two things to highlight:  the directions
    IN and OUT are set with respect to the HOST, and also in USB, the HOST runs all
    endpoint interactions.

    The out endpoint interface.  This is the out w.r.t. the host, hence in to
    us.  There are request grant, data available and data get signals, stall and
    acked signals.  And the data itself.

    To get data in and out there are two pipeline interfaces - one in and one out.

    OUT (or into this device)

    Roughly, the USB innards signal that a packet has arrived by raising out_ep_data_available.
    The data multiplexor has to be switched, so the interface is requested.  This is
    combinatorial logic so clever req and grant stuff can happen in the same line.

         assign out_ep_req = ( out_ep_req_reg || out_ep_data_avail );

    With the interface granted, the data is free to get.  Every cycle that the out_ep_data_get
    signal is high, the input address is advanced.  Inside the USB innards, when the 
    read address pointer equals the address of the write pointer (when all the data is 
    retreived, the out_ep_data_available flag is lowered and we withdraw our request for
    the interface and go back to idle.

    Interestingly, if you stop taking data... you lose your buffer.  So don't.

*/

module usb_uart_out_ep#(
  parameter MAX_OUT_PACKET_SIZE = 32
)(
  input clk,
  input reset,

  ///////////////////////////
  // OUT endpoint interface
  ///////////////////////////
  output out_ep_req,               // request the data interface for the out endpoint
  input out_ep_grant,              // data interface granted
  input out_ep_data_avail,         // flagging data available to get from the host - stays up until the cycle upon which it is empty
  input out_ep_setup,              // [setup packet sent? - not used here]
  output out_ep_data_get,          // request to get the data
  input [7:0] out_ep_data,         // data from the host
  output out_ep_stall,             // an output enabling the device to stop inputs (not used)
  input out_ep_acked,              // indicating that the outgoing data was acked

  ///////////////////////////
  // UART Pipeline interface
  ///////////////////////////
  output reg [7:0] uart_out_data,
  output       uart_out_valid,
  input        uart_out_get,
);

  localparam MAX_OUT_PACKET_BITS = $clog2(MAX_OUT_PACKET_SIZE);

  // Receive FIFO
  reg rx_pingpong = 0;   // Indicates which of the RX FIFOs is ready for USB data.
  reg rx_fifo_full = 0;  // Flag set if both FIFOs are full.
  reg [MAX_OUT_PACKET_BITS:0] rx_write_ptr = 0; // Write pointer into rx_pingpong.
  reg [MAX_OUT_PACKET_BITS:0] rx_read_ptr = 0;  // Read pointer into ~rx_pingpong.
  reg [MAX_OUT_PACKET_BITS:0] rx_read_len = 0;  // Data availabe in ~rx_pingpong.
  reg [7:0] rx_fifo[1:0][MAX_OUT_PACKET_SIZE-1:0];

  // Route signals into the USB core.
  assign out_ep_stall = 1'b0;
  assign out_ep_req = (rx_write_ptr < MAX_OUT_PACKET_SIZE) && ~rx_fifo_full;
  assign out_ep_data_get = (out_ep_req && out_ep_grant && out_ep_data_avail);

  // Route signals out of UART interface.
  assign uart_out_valid = (rx_read_ptr < rx_read_len);
  
  // Apply a latch delay to the data_get signal, since the USB core
  // takes one clock for data to start flowing.
  reg out_ep_data_valid = 0;
  always @(posedge clk) out_ep_data_valid <= out_ep_data_get;

  // Receive Data into the FIFO.
  always @(posedge clk) begin
    if ( reset ) begin
      rx_pingpong <= 0;
      rx_write_ptr <= 0;
      rx_read_ptr <= 0;
      rx_read_len <= 0;
      rx_fifo_full <= 0;
    end
    else begin
      // Receive data into USB side of the FIFO.
      if (!out_ep_setup && out_ep_data_valid) begin
        rx_fifo[rx_pingpong][rx_write_ptr] <= out_ep_data;
        rx_write_ptr <= rx_write_ptr + 1;
      end
      // Handle the end-of-packet event when out_ep_data_valid goes low.
      else if (rx_write_ptr) begin
          // If the UART fifo is free, swap the FIFOs and continue.
          if (~uart_out_valid) begin
            rx_pingpong <= ~rx_pingpong;
            rx_read_len <= rx_write_ptr;
            rx_read_ptr <= 0;
            rx_write_ptr <= 0;
            rx_fifo_full <= 0;
          end
          // Otherwise, both FIFOs are now full.
          else begin
            rx_fifo_full <= 1;
          end
      end

      // Read data from the UART side of the FIFO.
      if (uart_out_valid && uart_out_get) begin
        uart_out_data <= rx_fifo[~rx_pingpong][rx_read_ptr + 1];
        rx_read_ptr <= rx_read_ptr + 1;
      end
      else begin
        uart_out_data <= rx_fifo[~rx_pingpong][rx_read_ptr];
      end
    end
  end

endmodule
