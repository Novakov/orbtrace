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
                   output	    edgeOutput,

		   // Upwards interface to packet processor
		   output reg 	    byteAvail,    // Toggling indicator byte ready
		   output reg [7:0] completeByte  // The last constructed byte
		   );
   
   // Internals =======================================================================

   // Frame maintenance
   reg [6:0]  construct;                // Track of data being constructed
   reg [16:0] halfbitlen;               // Clock ticks for a half bit length
   reg [MAXBITLEN:0] activeCount;       // Clock ticks through this bit
   reg [2:0]  bitcount;                 // Index through this byte

   reg [1:0]  decodeState;              // Current state of decoder

   // Max count for bitlength calculations
   parameter MAXBITLEN          = 16;

   parameter DECODE_STATE_IDLE              = 0;
   parameter DECODE_STATE_GET_HBLEN         = 1;
   parameter DECODE_STATE_RXS_GETTING_BITS  = 2;

   // Calculations for bitlengths
   wire [MAXBITLEN:0] endofpacket      = { halfbitlen[MAXBITLEN-3:0],3'b0 };
   wire [MAXBITLEN:0] bitlenmin        = { halfbitlen[MAXBITLEN-1:0],1'b0 } - 1;

   // Bit construction slider
   reg [2:0]   bitsnow;

   // ...and edge detection
   wire        isEdge     = ((bitsnow==3'b011) || (bitsnow==3'b100));
   
   wire        newState   = bitsnow[1];
   wire        oldState   = bitsnow[2];
   
   assign 		    edgeOutput = activeCount>=bitlenmin;

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
	     activeCount <= activeCount+1;

	     bitsnow <= { bitsnow[1:0], SWOina };	     
	     			       
	     case (decodeState)
	       DECODE_STATE_IDLE: // --------------------------------------------------------
		 begin
		    activeCount <= 0;
		    if ((isEdge) && (newState==1))
		      decodeState <= DECODE_STATE_GET_HBLEN;
		 end
	       
	       DECODE_STATE_GET_HBLEN: // ----------------------------------------------------
		 if (isEdge)
		   begin
		      // Start looking for the middle of the first bit
		      halfbitlen  <= activeCount;
		      activeCount <= 0;
		      bitcount    <= 0;
		      decodeState <= DECODE_STATE_RXS_GETTING_BITS;
		   end
	       
	       DECODE_STATE_RXS_GETTING_BITS: // --------------------------------------------
		 if (isEdge)
		   begin
		      if (activeCount >= bitlenmin)
			begin
			   // This is a change in the middle of a bit..so here we need to record the value
			   activeCount   <= 0;
			   bitcount      <= bitcount + 1;
			   if (bitcount==7)
			     begin
				completeByte <= {oldState,construct[6:0]};
				byteAvail    <= ~byteAvail;
			     end
			   else
			     construct[bitcount] <= oldState;
			end // if (nextCount > bitlenmin)
		   end // else: !if(!isEdge)
		 else
		   if (activeCount > endofpacket)
		     // We ran out of packet (continious zero for more than two symbol periods)
		     decodeState <= DECODE_STATE_IDLE;
	     endcase // case (decodeState)
	  end // else: !if(rst)
     end // always @ (posedge traceClkin, posedge rst)
endmodule // swoManchIF
