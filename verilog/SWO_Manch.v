`default_nettype none
/* verilator lint_off WIDTH */

// SWO (Manchester formatted)
// ==========================
// Collects sequences of 8 bits from the SWO pin and delivers them to the layer above.
// Format of a sequence of a packet is defined in ARM DDI 0314H Section 1.11.2.
//
// Deals with sync and clocking and delivers clean 8 bit bytes of data to layer above.

module swoManchIF (
		   input 	    rst,          // Reset synchronised to clock
		   input            clk,          // Module clock used for edge sampling
		   
		   // Downwards interface to the swo pin
		   input 	    SWOina,       // SWO data rising edge
		   input 	    SWOinb,       // SWO data falling edge

		   // DIAGNOSTIC
                   output 	    edgeOutput,

		   // Upwards interface to packet processor
		   output reg 	    byteAvail,    // Toggling indicator byte ready
		   output reg [7:0] completeByte  // The last constructed byte
		   );
   
   // Internals =======================================================================

   // Frame maintenance
   reg [6:0]  construct;                // Track of data being constructed
   reg [16:0] halfbitlen;               // Clock ticks for a half bit length
   reg [16:0] activeCount;              // Clock ticks through this bit
   reg [2:0]  bitcount;                 // Index through this byte

   reg        oldState;                 // Previous state of the SWO bit
   
   reg [1:0]  decodeState;              // Current state of decoder

   parameter DECODE_STATE_IDLE              = 0;
   parameter DECODE_STATE_GET_HBLEN         = 1;
   parameter DECODE_STATE_RXS_GETTING_BITS0 = 2;
   parameter DECODE_STATE_RXS_GETTING_BITS1 = 3;   

   // Calculations for bitlengths
   wire [16:0] quarterbitlen    = { 1'b0, halfbitlen[16:1]};
   wire [16:0] threeightbitlen  = halfbitlen+quarterbitlen;
   wire [16:0] endofpacket      = { halfbitlen[13:0],1'b0,1'b0,1'b0 };
   wire [16:0] nextCount;

   // Bit construction slider
   wire [2:0] bitsnow = { oldState, SWOina, SWOinb };

   // ...and edge detection
   wire        isEdge;
   wire        newState   = bitsnow[0];
   wire [1:0]  startCount = (bitsnow[0]==bitsnow[1])?2:1;

   always @(posedge clk, posedge rst)
     begin
        // Default status bits
	if (rst)
	  begin
	     decodeState      <= DECODE_STATE_IDLE;
	  end
	else
	  begin
	     /* Calculate next count increment */
	     case (bitsnow)
	       3'b111, 3'b000:
		 begin
		    nextCount = activeCount+2;
		    isEdge    = 0;
		 end
	       
	       3'b110, 3'b001:
		 begin
		    nextCount = activeCount+1;
		    isEdge    = 1;
		 end
	       
	       3'b100, 3'b011:
		 begin
		    nextCount = activeCount;
		    isEdge    = 1;
		 end
	       
	       3'b010, 3'b101:
		 begin
		    nextCount = activeCount;
		    isEdge    = 0;
		 end
	     endcase // case (bitsnow)
	     
	     activeCount <= nextCount;
	     oldState    <= newState;
	     			       
	     case (decodeState)
	       DECODE_STATE_IDLE: // --------------------------------------------------
		 begin
		    if ((isEdge==1) && (newState==1'b1))
		      begin
			 decodeState <= DECODE_STATE_GET_HBLEN;
		      end
		    else
		      begin
			 activeCount <= 0;
		      end
		 end
	       
	       DECODE_STATE_GET_HBLEN: // --------------------------------------------
		 // If both halves are still high then extend count
		 begin
		    if ((isEdge==1) && (newState==0))
		      begin
			 halfbitlen  <= nextCount;
			 activeCount <= startCount;
			 bitcount    <= 0;
			 
			 // Get the first half of the bit
			 decodeState <= DECODE_STATE_RXS_GETTING_BITS0;
		      end
		 end // case: DECODE_STATE_GET_HBLEN
	       
	       DECODE_STATE_RXS_GETTING_BITS0: // --------------------------------------------
		 // First part of a bit
		 if (isEdge==1)
		   begin
		      // This is an edge change here...so we restart counting
		      activeCount   <= startCount;
		      
		      if (nextCount < threeightbitlen)
			begin
			   // This is a change at the start of a bit..so now wait for the mid-change
			   decodeState <= DECODE_STATE_RXS_GETTING_BITS1;
			end
		      else
			begin
			   // This is a change in the middle of a bit..so here we need to record the value
			   // but we stay in the GETTING_BITS0 state because we're looking for another bitstart
			   bitcount <= bitcount + 1;
			   if (bitcount==7)
			     begin
				completeByte <= {oldState,construct[6:0]};
				byteAvail    <= ~byteAvail;
			     end
			   else
			     begin
				construct[bitcount] <= oldState;
			     end
			end
		   end // else: !if(!isEdge)
		 else
		   begin
		      if (nextCount > endofpacket)
			begin
			   // We ran out of packet (continious zero for more than two symbol periods)
			   decodeState <= DECODE_STATE_IDLE;
			end
		   end // else: !if(isEdge)
	       
	       DECODE_STATE_RXS_GETTING_BITS1: // --------------------------------------------
		 if (isEdge==1)
		   begin
		      // This is definately a change in the middle of a bit..so here we need to record the value
		      activeCount  <= startCount;
		      
		      // The next bit we're looking for will be the first half
		      decodeState <= DECODE_STATE_RXS_GETTING_BITS0;
		      
		      bitcount <= bitcount + 1;
		      if (bitcount==7)
			begin
			   completeByte <= {oldState,construct[6:0]};
			   byteAvail    <= ~byteAvail;
			end
		      else
			begin
			   construct[bitcount] <= oldState;
			end
		   end // if (isEdge)
		 else
		   begin
		      if (nextCount > endofpacket)
			begin
			   // We ran out of packet as per case above
			   decodeState <= DECODE_STATE_IDLE;
			end
		   end // else: !if(isEdge)		 
	     endcase // case (decodeState)
	  end // else: !if(rst)
     end // always @ (posedge traceClkin, posedge rst)
endmodule // swoManchIF
