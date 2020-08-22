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

module usb_uart_out_ep (
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
  output [7:0] uart_out_data,
  output       uart_out_valid,
  input        uart_out_ready,
);

  // Timeout counter width.
  localparam TimeoutWidth = 3;

  // We don't stall
  assign out_ep_stall = 1'b0;

  // Registers for the out pipeline (out of the module)
  reg [7:0] uart_out_data_reg;
  reg [7:0] uart_out_data_overflow_reg;
  reg       uart_out_valid_reg;

  // registers for the out end point (out of the host)
  reg       out_ep_req_reg;
  reg       out_ep_data_get_reg;

  // out pipeline / out endpoint state machine state (6 states -> 3 bits)
  reg [1:0] pipeline_out_state;

  localparam PipelineOutState_Idle         = 0;
  localparam PipelineOutState_WaitData     = 1;
  localparam PipelineOutState_PushData     = 2;
  localparam PipelineOutState_WaitPipeline = 3;

  // connect the pipeline registers to the outgoing ports
  assign uart_out_data = uart_out_data_reg;
  assign uart_out_valid = uart_out_valid_reg;

  // automatically make the bus request from the data_available
  // latch it with out_ep_req_reg
  assign out_ep_req = ( out_ep_req_reg || out_ep_data_avail );

  wire out_granted_data_available;

  assign out_granted_data_available = out_ep_req && out_ep_grant;

  assign out_ep_data_get = ( uart_out_ready || ~uart_out_valid_reg ) && out_ep_data_get_reg;

  reg [7:0] out_stall_data;
  reg       out_stall_valid;

  // do HOST OUT, DEVICE IN, PIPELINE OUT (!)
  always @(posedge clk) begin
      if ( reset ) begin
          pipeline_out_state <= PipelineOutState_Idle;
          uart_out_data_reg <= 0;
          uart_out_valid_reg <= 0;
          out_ep_req_reg <= 0;
          out_ep_data_get_reg <= 0;
          out_stall_data <= 0;
          out_stall_valid <= 0;
      end else begin
          case( pipeline_out_state )
              PipelineOutState_Idle: begin
                  // Waiting for the data_available signal indicating that a data packet has arrived
                  if ( out_granted_data_available ) begin
                      // indicate that we want the data
                      out_ep_data_get_reg <= 1;
                      // although the bus has been requested automatically, we latch the request so we control it
                      out_ep_req_reg <= 1;
                      // now wait for the data to set up
                      pipeline_out_state <= PipelineOutState_WaitData;
                      uart_out_valid_reg <= 0;
                      out_stall_data <= 0;
                      out_stall_valid <= 0;
                  end
              end
              PipelineOutState_WaitData: begin
                  // it takes one cycle for the juices to start flowing
                  // we got here when we were starting or if the outgoing pipe stalled
                  if ( uart_out_ready || ~uart_out_valid_reg ) begin
                      //if we were stalled, we can send the byte we caught while we were stalled
                      // the actual stalled byte now having been VALID & READY'ed
                      if ( out_stall_valid ) begin
                        uart_out_data_reg <= out_stall_data;
                        uart_out_valid_reg <= 1;
                        out_stall_data <= 0;
                        out_stall_valid <= 0;
                        if ( out_ep_data_avail )
                          pipeline_out_state <= PipelineOutState_PushData;
                        else begin
                          pipeline_out_state <= PipelineOutState_WaitPipeline;
                        end
                      end else begin
                          pipeline_out_state <= PipelineOutState_PushData;
                      end
                  end
              end
              PipelineOutState_PushData: begin
                  // can grab a character if either the out was accepted or the out reg is empty
                  if ( uart_out_ready || ~uart_out_valid_reg ) begin
                    // now we really have got some data and a place to shove it
                    uart_out_data_reg <= out_ep_data;
                    uart_out_valid_reg <= 1;
                    if ( ~out_ep_data_avail ) begin
                        // stop streaming, now just going to wait until the character is accepted
                        out_ep_data_get_reg <= 0;
                        pipeline_out_state <= PipelineOutState_WaitPipeline;
                    end
                  end else begin
                    // We're in push data so there is a character, but our pipeline has stalled
                    // need to save the character and wait.
                    out_stall_data <= out_ep_data;
                    out_stall_valid <= 1;
                    pipeline_out_state <= PipelineOutState_WaitData;                     
                    if ( ~out_ep_data_avail )
                        out_ep_data_get_reg <= 0;
                  end
              end
              PipelineOutState_WaitPipeline: begin
                  // unhand the bus (don't want to block potential incoming) - be careful, this works instantly!
                  out_ep_req_reg <= 0;
                  if ( uart_out_ready ) begin
                      uart_out_valid_reg <= 0;
                      uart_out_data_reg <= 0;
                      pipeline_out_state <= PipelineOutState_Idle;
                  end

              end

          endcase
      end
  end

endmodule
