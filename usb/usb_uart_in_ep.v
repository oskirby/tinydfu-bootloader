/*
    usb_uart_in_ep

    This is the endpoint to uart translator.  Two things to highlight:  the directions
    IN and OUT are set with respect to the HOST, and also in USB, the HOST runs all
    endpoint interactions.

    The in endpoint interface.  This is the in w.r.t. the host, hence out to us.
    This interface also has a req and grant.  There's a put signal and a free
    signal.  Stall and acked.  And the data.

    To get data in and out there are two pipeline interfaces - one in and one out.

    IN (or out of this device back to the host)

    The IN EP works by providing a buffer and waiting for the local logic to fill it, 
    or to say that it's done.  When this happens the interface switches to a new state where
    it waits for a token from the host.  When it get the token, it sends the data.  When that
    is acknoledged, the buffer is released and returned ready to be filled again.

    in_ep_data_free signals that there's a buffer waiting.  And that signal goes low when 
    the buffer is full and not available.

    In the case where a buffer is not full - just sitting around with some data in it, a decision
    has to be made at some point just to send.  This is handled by a timeout mechanism, which 
    asserts in_ep_data_done and lets the buffer be sent.

    In the case where the buffer fills to the top, in_ep_data_free goes low by itself.

*/

module usb_uart_in_ep (
  input clk,
  input reset,

  ///////////////////////////
  // IN endpoint interface
  ///////////////////////////
  output in_ep_req,                // request the data interface for the in endpoint
  input in_ep_grant,               // data interface granted
  input in_ep_data_free,           // end point is ready for data - (specifically there is a buffer and it has space)
                                   // after going low it takes a while to get another back, but it does this automatically
  output in_ep_data_put,           // forces end point to read our data
  output [7:0] in_ep_data,         // data back to the host
  output in_ep_data_done,          // signalling that we're done sending data
  output in_ep_stall,              // an output enabling the device to stop outputs (not used)
  input in_ep_acked,               // indicating that the outgoing data was acked

  ///////////////////////////
  // UART Pipeline interface
  ///////////////////////////
  input [7:0] uart_in_data,
  input       uart_in_valid,
  output      uart_in_ready,
);

  // Timeout counter width.
  localparam TimeoutWidth = 3;

  // We don't stall
  assign in_ep_stall = 1'b0;

  // in endpoint control registers
  reg       in_ep_req_reg;
  reg       in_ep_data_done_reg;

  // in pipeline / in endpoint state machine state (4 states -> 2 bits)
  reg [1:0] pipeline_in_state;

  localparam PipelineInState_Idle      = 0;
  localparam PipelineInState_WaitData  = 1;
  localparam PipelineInState_CycleData = 2;
  localparam PipelineInState_WaitEP    = 3;

  // connect the pipeline register to the outgoing port
  assign uart_in_ready = ( pipeline_in_state == PipelineInState_CycleData ) && in_ep_data_free;

  // uart_in_valid and a buffer being ready is the request for the bus. 
  // It is granted automatically if available, and latched on by the SM.
  // Note once requested, uart_in_valid may go on and off as data is available.
  // When requested, connect the end point registers to the outgoing ports
  assign in_ep_req = ( uart_in_valid && in_ep_data_free) || in_ep_req_reg;

  // Confirmation that the bus was granted
  wire in_granted_in_valid = in_ep_grant && uart_in_valid;

  // Here are the things we use to get data sent
  // ... put this word
  assign in_ep_data_put = ( pipeline_in_state == PipelineInState_CycleData ) && uart_in_valid && in_ep_data_free;
  // ... we're done putting - send the buffer
  assign in_ep_data_done = in_ep_data_done_reg;
  // ... the actual data, direct from the pipeline to the usb in buffer
  assign in_ep_data = uart_in_data;

  // If we have a half filled buffer, send it after a while by using a timer
  // 4 bits of counter, we'll just count up until bit 3 is high... 8 clock cycles seems more than enough to wait
  // to send the packet
  reg [TimeoutWidth:0] in_ep_timeout;

  // do PIPELINE IN, FPGA/Device OUT, Host IN
  always @(posedge clk) begin
      if ( reset ) begin
          pipeline_in_state <= PipelineInState_Idle;
          in_ep_req_reg <= 0;
          in_ep_data_done_reg <= 0;
      end else begin
          case( pipeline_in_state )
              PipelineInState_Idle: begin
                  in_ep_data_done_reg <= 0;
                  if ( in_granted_in_valid && in_ep_data_free ) begin
                      // got the bus, there is free space, now do the data
                      // confirm request bus - this will hold the request up until we're done with it
                      in_ep_req_reg <= 1;
                      pipeline_in_state <= PipelineInState_CycleData;
                  end
              end
              PipelineInState_CycleData: begin
                  if  (uart_in_valid ) begin
                      if ( ~in_ep_data_free ) begin
                        // back to idle
                        pipeline_in_state <= PipelineInState_Idle;
                        // release the bus
                        in_ep_req_reg <= 0;
                      end
                  end else begin
                      // No valid character.  Let's just pause for a second to see if any more are forthcoming.
                      // clear the timeout counter
                      in_ep_timeout <= 0;
                      pipeline_in_state <= PipelineInState_WaitData;
                 end
              end
              PipelineInState_WaitData: begin
                  in_ep_timeout <= in_ep_timeout + 1;
                  if ( uart_in_valid ) begin
                      pipeline_in_state <= PipelineInState_CycleData;                
                  end else begin
                        // check for a timeout
                        if ( in_ep_timeout[ TimeoutWidth ] ) begin
                            in_ep_data_done_reg <= 1;
                            pipeline_in_state <= PipelineInState_WaitEP;
                        end
                  end
              end

              PipelineInState_WaitEP: begin
                  // maybe done is ignored if putting?
                  in_ep_data_done_reg <= 0;

                  // back to idle
                  pipeline_in_state <= PipelineInState_Idle;
                  // release the bus
                  in_ep_req_reg <= 0;
              end
          endcase
      end
  end

  //assign debug = { in_ep_data_free, in_ep_data_done, pipeline_in_state[ 1 ], pipeline_in_state[ 0 ] };  

endmodule
