/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2025 MicroFPGA UG(h)
 * https://github.com/micro-FPGA/OpenMBMC
 */ 

`include "openmbmc_platform.v"

//  do we include debug signals?
`define DEBUG_IO

// define this to enable tristate buffers in the code like with Quartus
//`define BIDIR
// define this to enable byte addressing, like in Vivado
//`define BYTEADDR

`ifdef QUARTUS
`define BIDIR
`define RESET_FIX
`endif

`ifdef VIVADO
`define BYTEADDR
`define USE_READDATAVALID
`endif

`ifdef PROPEL
`define BYTEADDR
`define RESET_FIX
`define USE_READDATAVALID
`endif

`ifdef PROPEL_EMU
`define BYTEADDR
`define USE_READDATAVALID
`endif



`timescale 1 ps / 1 ps
module openmbmc 
		#(
		parameter G_ADDRBITS = 23,
		parameter G_DEVICE = 0,
		parameter G_VARLAT = 1,
		parameter G_DDP = 0,
		parameter G_TVCS = 15000
		)
		(
		input  wire        reset_rst,             //       
`ifdef BYTEADDR		
		input  wire [G_ADDRBITS-1:0] avl_mem_address,       //
`else	
		input  wire [G_ADDRBITS-3:0] avl_mem_address, // word adressing two address bits less       
`endif	
		input  wire        avl_mem_read,          //            .read
		output reg  [31:0] avl_mem_readdata,      //            .readdata
`ifdef USE_READDATAVALID		
		output wire        avl_mem_readdatavalid, //            .readdatavalid
`endif		
		input  wire        avl_mem_write,         //            .write
		input  wire [31:0] avl_mem_writedata,     //            .writedata
		input  wire [3:0]  avl_mem_byteenable,    //            .byteenable
		output wire        avl_mem_waitrequest,   //            .waitrequest
		input  wire        avl_clock,             //   
		input  wire        hyper_clock,           // 
		output wire        cs_n,                  //    hyperbus.cs_n
		output wire	       ck,                    //            .ck
		output wire        ck_n,                  //            .ck_n
		
`ifdef BIDIR
		inout  wire        rwds,                  //            .rwds
		inout  wire [7:0]  dq,                    //            .dq
`else
		input  wire        rwds_i,                //
		output wire        rwds_o,                //
		output wire        rwds_t,                //

		input  wire [7:0]  dq_i,                  //
		output wire [7:0]  dq_o,                  //
		output wire [7:0]  dq_t,                  //
`endif
		
		output wire        init_done,             //
		
`ifdef DEBUG_IO
		output wire [7:0]  dq_dbg,   
		output wire        rwds_dbg,     
		output wire [9:0]  state_dbg,   
`endif		    
		output wire        reset_n                //            .reset_n
	);

localparam DEV_HYPERRAM = 0;
localparam DEV_HYPERFLASH = 1;
localparam DEV_OPI = 2; // not currently supported

reg avl_mem_readdatavalid_s;

`ifdef USE_READDATAVALID		
assign avl_mem_readdatavalid = avl_mem_readdatavalid_s;		
`endif		

reg avl_mem_wr_rd_ready;
	
// 
reg [47:0] hyper_shift_reg;
reg hyper_end;
reg hyper_start_rd;
reg hyper_start_wr;
reg hyper_busy;

reg [7:0] hyper_dq_out_reg;
reg hyper_dq_out_oe;
reg hyper_rwds_oe;
reg hyper_ck;
reg hyper_cs_reg;
reg hyper_rwds_reg;
reg hyper_rst;
reg [9:0] hyper_bus_state;
reg hyper_recovery_reg;

// new stuff
reg [15:0] tvcs_count;   // count for reset recovery time, after power on and reset of the core
reg tvcs_done;           // we are done with tVCS
reg init_done_int;
reg hyper_start_cr;
reg latency; // 1 2LC, 0 1LC

wire rwds_s;     // copy of rwds or rwds_i
reg [7:0] dq_f;  // delay falling edge copy of incoming DQ

`ifdef DEBUG_IO
assign rwds_dbg = rwds_s;
assign dq_dbg = dq_f;
assign state_dbg = hyper_bus_state;
`endif

assign init_done = init_done_int;

`ifdef BIDIR
assign dq 		= (hyper_dq_out_oe) ? hyper_dq_out_reg : 8'bzzzzzzzz;
assign rwds 	= (hyper_rwds_oe) ? hyper_rwds_reg : 1'bz;
assign rwds_s 	= rwds; 
`else
assign dq_o		= hyper_dq_out_reg;
assign dq_t[0]  = ~hyper_dq_out_oe;
assign dq_t[1]  = ~hyper_dq_out_oe;
assign dq_t[2]  = ~hyper_dq_out_oe;
assign dq_t[3]  = ~hyper_dq_out_oe;
assign dq_t[4]  = ~hyper_dq_out_oe;
assign dq_t[5]  = ~hyper_dq_out_oe;
assign dq_t[6]  = ~hyper_dq_out_oe;
assign dq_t[7]  = ~hyper_dq_out_oe;

assign rwds_s 	= rwds_i;
assign rwds_o 	= hyper_rwds_reg;
assign rwds_t 	= ~hyper_rwds_oe;
`endif

assign ck 		= hyper_ck;
assign ck_n 	= ~hyper_ck;

assign reset_n 	= ~reset_rst; // dummy just forward inverse of incoming reset, should work?
assign cs_n		= hyper_cs_reg;

assign avl_mem_waitrequest	= (avl_mem_write || avl_mem_read) ? !avl_mem_wr_rd_ready : 1'b0;

always @(negedge hyper_clock)
begin
`ifdef BIDIR
    dq_f <= dq;
`else 
    dq_f <= dq_i;
`endif    
end


`ifdef RESET_FIX
always @(posedge hyper_clock)
`else
// this causes QUARTUS to give synthese error! Same with Lattice synthesis!
always @(posedge hyper_clock or posedge reset_rst or posedge hyper_recovery_reg)
`endif

begin
	if(reset_rst) begin
	    tvcs_count              <= 16'b0000000000000000;
	    tvcs_done               <= 1'b0;
	    init_done_int           <= 1'b0;   
	    
	    hyper_start_cr          <= 1'b0;
	end else begin
		// count for the tVCS to elapse
if (G_DEVICE == DEV_HYPERFLASH) begin				
        init_done_int <= 1'b1; // we do not do init for HyperFlash
        tvcs_done <= 1'b1; // we do not wait for HyperFlash
end else begin
		if (tvcs_done == 1'b0) begin
		      tvcs_count <= tvcs_count + 1;
		      // tVCS counter elapsed?
		      if (tvcs_count == G_TVCS) begin // 100MHz clock = 150uS
		          tvcs_done <= 1'b1; // Flag we are done here
		          // we start CR0 writing process
		          hyper_start_cr         <= 1'b1;
		      end
		end
		if (init_done_int == 0) begin
		  if (hyper_bus_state == 101) begin
		      hyper_start_cr         <= 1'b0; // we are done with CR0 writing!
		      init_done_int <= 1'b1; // init done set flag!
		  end
		end 
end		     
	end
	
	if(reset_rst || hyper_recovery_reg) begin
		hyper_rst				<= 1'b0;
		hyper_end				= 1'b0;
		hyper_busy				= 1'b0;
		hyper_cs_reg			<= 1'b1;
		hyper_ck				<= 1'b0;
		hyper_rwds_reg			= 1'b1;
		hyper_rwds_oe			= 1'b0; // ??
		hyper_bus_state			<= 0;
	end else begin
		hyper_rst				<= 1'b1;
		

        // keep in idle for tVCS time (about 150uS)
		if((hyper_start_rd || hyper_start_wr || hyper_start_cr) && !(hyper_end) && tvcs_done) begin
			hyper_bus_state				<= hyper_bus_state + 1;
			hyper_busy					= 1'b1;			

//			if(hyper_dq_ack && (hyper_bus_state < 96)) begin
//				hyper_bus_state			<= 96;
//			end else begin
            //begin -- not needed			
				// keep CS high a bit
				if(hyper_bus_state == 0) begin
					hyper_rwds_oe		= 0;
					hyper_cs_reg		<= 1;
					hyper_dq_out_oe		<= 0;
					hyper_ck			<= 0;
					hyper_dq_out_reg	<= 8'b00000000;
				end

				if(hyper_bus_state == 1) begin
					hyper_cs_reg		<= 1;
					hyper_dq_out_oe		<= 0;
					hyper_ck			<= 0;
					hyper_dq_out_reg	<= 8'b00000000;
				end

				if(hyper_bus_state == 2) begin
					hyper_cs_reg		<= 1;
					hyper_dq_out_oe		<= 0;
					hyper_ck			<= 0;
					hyper_dq_out_reg	<= 8'b00000000;
				end

				if(hyper_bus_state == 3) begin
					hyper_cs_reg		<= 0;
					hyper_dq_out_oe		<= 1;
					hyper_ck			<= 0;
					hyper_dq_out_reg	<= hyper_shift_reg[47:40];
				end

				if(hyper_bus_state == 4) begin
					hyper_dq_out_oe		<= 1;
					hyper_ck			<= 0;
					hyper_dq_out_reg	<= hyper_shift_reg[47:40];
				end

				if(hyper_bus_state == 5) begin
					hyper_dq_out_oe		<= 1;
					hyper_ck			<= 1;
					hyper_dq_out_reg	<= hyper_shift_reg[47:40];
				end

				if(hyper_bus_state == 6) begin
					hyper_dq_out_oe		<= 1;
					hyper_ck			<= 1;
					hyper_dq_out_reg	<= hyper_shift_reg[39:32];
				end

				if(hyper_bus_state == 7) begin
					hyper_dq_out_oe		<= 1;
					hyper_ck			<= 0;
					hyper_dq_out_reg	<= hyper_shift_reg[39:32];
				end

				if(hyper_bus_state == 8) begin
					hyper_ck			<= 0;
					hyper_dq_out_oe		<= 1;
					hyper_dq_out_reg	<= hyper_shift_reg[31:24];
				end

				if(hyper_bus_state == 9) begin
					hyper_ck			<= 1;
					hyper_dq_out_oe		<= 1;
					hyper_dq_out_reg	<= hyper_shift_reg[31:24];
				end

				if(hyper_bus_state == 10) begin
					hyper_ck			<= 1;
					hyper_dq_out_oe		<= 1;
					hyper_dq_out_reg	<= hyper_shift_reg[23:16];
					// latch latency from RWDS
if (G_VARLAT == 1)	begin				
					latency             <= rwds_s;
end else begin
                    latency             <= 1'b1;
end					
				end

				if(hyper_bus_state == 11) begin
					hyper_ck			<= 0;
					hyper_dq_out_oe		<= 1;
					hyper_dq_out_reg	<= hyper_shift_reg[23:16];
				end

				if(hyper_bus_state == 12) begin
					hyper_ck			<= 0;
					hyper_dq_out_oe		<= 1;
					hyper_dq_out_reg	<= hyper_shift_reg[15:8];
				end

				if(hyper_bus_state == 13) begin
					hyper_ck			<= 1;
					hyper_dq_out_oe		<= 1;
					hyper_dq_out_reg	<= hyper_shift_reg[15:8];
				end

				if(hyper_bus_state == 14) begin
					hyper_ck			<= 1;
					hyper_dq_out_oe		<= 1;
					hyper_dq_out_reg	<= hyper_shift_reg[7:0];
				end

				if(hyper_bus_state == 15) begin
					hyper_ck			<= 0;
					hyper_dq_out_oe		<= 1;
					hyper_dq_out_reg	<= hyper_shift_reg[7:0];
				end

				//release DQ bus if not register write!
				if(hyper_bus_state == 16) begin
					hyper_ck				<= 0;
					
					if(hyper_start_cr) begin
					   hyper_dq_out_reg <= 8'b10001111; // 0x8F high byte of CR0 
					end else if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
					end
					
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
						
if (G_DEVICE == DEV_HYPERRAM) begin						
						hyper_dq_out_reg	<= 8'h00;
end
						
if (G_DEVICE == DEV_HYPERFLASH) begin
                        if (avl_mem_byteenable[2] == 1'b1) begin
						      hyper_dq_out_reg	<= avl_mem_writedata[31:24];
						end else begin
						      hyper_dq_out_reg	<= avl_mem_writedata[15:8];
						end
end						
					end
				end

				// ck high
				if(hyper_bus_state == 17) begin
					hyper_ck				<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end

				// ck high
				if(hyper_bus_state == 18) begin
					hyper_ck				<= 1;
					
					if(hyper_start_cr) begin
if (G_VARLAT == 1)	begin					   
					   hyper_dq_out_reg <= 8'b11100111;   // 0xE7 low byte of CR0 VARIABLE LATENCY 3
end else begin
					   hyper_dq_out_reg <= 8'b11101111;   // 0xEF low byte of CR0 FIXED LATENCY 3

end					   
					end else if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
					end
					
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
						
						
if (G_DEVICE == DEV_HYPERFLASH) begin
                        if (avl_mem_byteenable[2] == 1'b1) begin
						      hyper_dq_out_reg	<= avl_mem_writedata[23:16];
						end else begin
						      hyper_dq_out_reg	<= avl_mem_writedata[7:0];
						end
end						
					end
				end
                 
				if(hyper_bus_state == 19) begin
					hyper_ck				<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
					end

					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
if (G_DEVICE == DEV_HYPERRAM) begin					
					// skip 3 clocks is LC=1, but only when init is done!
					if((latency == 1'b0) && init_done_int) begin
					   hyper_bus_state <= 32;
					end
end					
					
				end
//3
				if(hyper_bus_state == 20) begin
					hyper_ck				<= 0;

					if(hyper_start_cr) begin
                        hyper_bus_state <= 100; 
						hyper_dq_out_oe		<= 0;
					end
					
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
						
if (G_DEVICE == DEV_HYPERFLASH) begin
                        hyper_bus_state     <= 100; 
						hyper_dq_out_oe		<= 0;
end						
					end
				end
                // HyperFlash Write done

				if(hyper_bus_state == 21) begin
					hyper_ck				<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 22) begin
					hyper_ck				<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 23) begin
					hyper_ck				<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end
//2
				if(hyper_bus_state == 24) begin
					hyper_ck				<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 25) begin
					hyper_ck				<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end

				if(hyper_bus_state == 26) begin
					hyper_ck				<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 27) begin
					hyper_ck				<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					
					// we skip one more to have latency 5
if (G_DEVICE == DEV_HYPERFLASH) begin
                        hyper_bus_state     <= 32; 
end						
					
					
				end
//1
				if(hyper_bus_state == 28) begin
					hyper_ck				<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 29) begin
					hyper_ck				<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end

				if(hyper_bus_state == 30) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 31) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end
// jump here if single latency count skipping 3 clocks
				if(hyper_bus_state == 32) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 33) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end

				if(hyper_bus_state == 34) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 35) begin
					hyper_ck				<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					//
					// skip 6 clocks of latency, we are in variable latency 3 mode
					hyper_bus_state <= 60;
				end

/*
//6
				if(hyper_bus_state == 36) begin
					hyper_ck				<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 37) begin
					hyper_ck				<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end

				if(hyper_bus_state == 38) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end
				////////////////////////

				if(hyper_bus_state == 39) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end
//5
				if(hyper_bus_state == 40) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 41) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end

				if(hyper_bus_state == 42) begin
					hyper_ck				<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 43) begin
					hyper_ck				<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end
//4
				if(hyper_bus_state == 44) begin
					hyper_ck				<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 45) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end

				if(hyper_bus_state == 46) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 47) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end
//3
				if(hyper_bus_state == 48) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 49) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end

				if(hyper_bus_state == 50) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 51) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end
//2
				if(hyper_bus_state == 52) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 53) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end

				if(hyper_bus_state == 54) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end
				///////////////

				if(hyper_bus_state == 55) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end
//1
				if(hyper_bus_state == 56) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 57) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end

				if(hyper_bus_state == 58) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 59) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end
*/
				////////////////////////////////////////////////////////////////////
				// 
				
				// we need to land here after latency is expired
				if(hyper_bus_state == 60) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
                        hyper_rwds_reg		= ~avl_mem_byteenable[1];						
						hyper_dq_out_reg	<= avl_mem_writedata[15:8];
					end
				end

                
				if(hyper_bus_state == 61) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end

				if(hyper_bus_state == 62) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						//hyper_dq_out_reg	<= 8'b00000000;
						//avl_mem_readdata[15:8]	= dq[7:0];
						// we read one full clock later..
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= ~avl_mem_byteenable[0];
						hyper_dq_out_reg	<= avl_mem_writedata[7:0];
					end
				end

				if(hyper_bus_state == 63) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
//adjusted delay						
						avl_mem_readdata[15:8]	= dq_f[7:0];
						
					end
				end

				if(hyper_bus_state == 64) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						//hyper_dq_out_reg	<= 8'b00000000;
						//avl_mem_readdata[7:0]	= dq[7:0];
//						avl_mem_readdata[15:8]	= dq_f[7:0];

					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
						hyper_rwds_reg		= ~avl_mem_byteenable[3];
						hyper_dq_out_reg	<= avl_mem_writedata[31:24];
					end
				end

				if(hyper_bus_state == 65) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
//adjusted delay						
						avl_mem_readdata[7:0]	= dq_f[7:0];						
					end
				end

				if(hyper_bus_state == 66) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						//hyper_dq_out_reg	<= 8'b00000000;
						//avl_mem_readdata[31:24]	= dq[7:0];
//						avl_mem_readdata[7:0]	= dq_f[7:0];
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
						hyper_rwds_reg		= ~avl_mem_byteenable[2];
						hyper_dq_out_reg	<= avl_mem_writedata[24:16];
					end
				end

				if(hyper_bus_state == 67) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
//adjusted delay						
						avl_mem_readdata[31:24]	= dq_f[7:0];						
					end

					if(hyper_start_wr) begin
						//hyper_rwds_oe		= 1;
						//hyper_rwds_reg		= 0;
						hyper_bus_state		<= 100;
					end
				end
				
				//////////// end writing //////////////

				if(hyper_bus_state == 68) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						//hyper_dq_out_reg	<= 8'b00000000;
						//avl_mem_readdata[23:16]	= dq[7:0];
//						avl_mem_readdata[31:24]	= dq_f[7:0];
						//hyper_bus_state		<= 100;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 69) begin
					hyper_ck			<= 0; //1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
// adjusted delay
						avl_mem_readdata[23:16]	= dq_f[7:0];
						hyper_bus_state		<= 100;
					end
				end

				if(hyper_bus_state == 70) begin
					hyper_ck			<= 0; //1; 
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
//						avl_mem_readdata[23:16]	= dq_f[7:0];
//						hyper_bus_state		<= 100;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end
/*
				if(hyper_bus_state == 71) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end

				if(hyper_bus_state == 72) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 73) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end

				if(hyper_bus_state == 74) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 75) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end

				if(hyper_bus_state == 76) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 77) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end

				if(hyper_bus_state == 78) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 79) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end

				if(hyper_bus_state == 80) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 81) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
				end

				if(hyper_bus_state == 82) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_oe		= 1;
						hyper_rwds_reg		= 0;
					end
				end

				if(hyper_bus_state == 83) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end

					if(hyper_start_wr) begin
						hyper_dq_out_reg	<= avl_mem_writedata[15:8];
						hyper_dq_out_oe		<= 1;
						hyper_rwds_reg		= 1'b1;
						hyper_rwds_oe		= 1'b1;
					end
				end

				if(hyper_bus_state == 84) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_reg	<= 8'b00000000;
						hyper_dq_out_oe		<= 0;
					end
					
					if(hyper_start_wr) begin
						hyper_rwds_reg		= 1'b1;
						hyper_rwds_oe		= 1'b1;
					end
				end

				if(hyper_bus_state == 85) begin
					hyper_ck			<= 1;

					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					
					if(hyper_start_wr) begin
						hyper_rwds_reg		= 1'b0;
					end
				end

				if(hyper_bus_state == 86) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end

					// step 4 - RWDS low, LSB load on BUS
					if(hyper_start_wr) begin
						hyper_dq_out_reg	<= avl_mem_writedata[7:0];
						hyper_rwds_reg		= 1'b0;
					end
				end

				if(hyper_bus_state == 87) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end

					if(hyper_start_wr) begin
						hyper_rwds_reg		= 1'b1;
						hyper_dq_out_reg	<= avl_mem_writedata[31:24];
					end
				end

				if(hyper_bus_state == 88) begin
					hyper_ck			<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end

					if(hyper_start_wr) begin
						hyper_rwds_reg		= 1'b1;
					end

				end

				if(hyper_bus_state == 89) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end

					if(hyper_start_wr) begin
						hyper_rwds_reg		= 1'b0;
					end
				end

				if(hyper_bus_state == 90) begin
					hyper_ck			<= 1;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end

					if(hyper_start_wr) begin
						hyper_rwds_reg		= 1'b0;
						hyper_dq_out_reg	<= avl_mem_writedata[24:16];
					end

				end

				if(hyper_bus_state == 91) begin
					hyper_ck				<= 0;
					if(hyper_start_rd) begin
						hyper_dq_out_oe		<= 0;
						hyper_dq_out_reg	<= 8'b00000000;
					end
					if(hyper_start_wr) begin
						hyper_rwds_reg		= 1'b1;
					end
				end

				if(hyper_bus_state == 92) begin
					hyper_ck			<= 0;
					hyper_dq_out_oe		<= 0;
					hyper_dq_out_reg	<= 8'b00000000;
					if(hyper_start_wr) begin
						hyper_rwds_reg		= 1'b1;
					end
				end

				if(hyper_bus_state == 93) begin
					hyper_ck			<= 1;
					hyper_dq_out_oe		<= 0;
					hyper_dq_out_reg	<= 8'b00000000;
					if(hyper_start_wr) begin
						hyper_rwds_reg		= 1'b1;
					end
				end

				if(hyper_bus_state == 94) begin
					hyper_ck			<= 1;
					hyper_dq_out_oe		<= 0;
					hyper_dq_out_reg	<= 8'b00000000;					
				end

				if(hyper_bus_state == 95) begin
					hyper_ck			<= 0;
					hyper_dq_out_oe		<= 0;
					hyper_dq_out_reg	<= 8'b00000000;
				end

				if(hyper_bus_state == 96) begin
					hyper_ck			<= 0;
					hyper_dq_out_oe		<= 0;
					hyper_dq_out_reg	<= 8'b00000000;
					
					if(hyper_start_wr) begin
						hyper_bus_state		<= 99;
					end
				end
*/				
			//end

			if(hyper_bus_state == 97) begin
				hyper_dq_out_oe		<= 0;
				hyper_ck			<= 1;
				hyper_dq_out_reg	<= 8'b00000000;

				if(hyper_start_wr) begin
					hyper_rwds_reg	= 1'b1;
				end
			end

			if(hyper_bus_state == 98) begin
				hyper_dq_out_oe		<= 0;
				hyper_ck			<= 1;
				hyper_dq_out_reg	<= 8'b00000000;

				if(hyper_start_wr) begin
					hyper_rwds_reg	= 1'b1;
				end
			end

			if(hyper_bus_state == 99) begin
				hyper_dq_out_oe		<= 0;
				hyper_ck			<= 0;
				hyper_dq_out_reg	<= 8'b00000000;

				if(hyper_start_wr) begin
					hyper_rwds_reg	= 1'b1;
				end
			end

			if(hyper_bus_state == 100) begin
				hyper_cs_reg		<= 0;
				hyper_dq_out_oe		<= 0;
				hyper_ck			<= 0;
				hyper_dq_out_reg	<= 8'b00000000;
				hyper_end			= 1;

				if(hyper_start_wr) begin
					hyper_rwds_reg	= 1'b0;
					hyper_rwds_oe	= 0;
				end
			end

		end else begin
			hyper_bus_state			<= 0;
			hyper_cs_reg			<= 1;

			if(!avl_mem_read && !avl_mem_write) begin
				hyper_end				= 1'b0;
				hyper_busy				= 1'b0;
				hyper_ck				<= 1'b0;
				hyper_rwds_reg			= 1'b0;
				hyper_rwds_oe			= 1'b0;
			end
		end
	end
end



always @(posedge avl_clock)
begin
	if(reset_rst) begin
		hyper_start_rd			= 1'b0;
		hyper_start_wr			= 1'b0;
		avl_mem_readdatavalid_s	= 1'b0;
		avl_mem_wr_rd_ready		<= 1'b0;
		//avl_wr_reg				<= 1'b0;
	end else begin
	
// register auto init	
if (G_DEVICE == DEV_HYPERRAM) begin	
	    // CR0 write data
        if(hyper_start_cr) begin
				hyper_shift_reg[7:0]	<= 8'b00000000;								// address CR0 = 0
				hyper_shift_reg[15:8]	<= 8'b00000000;								// 
				hyper_shift_reg[23:16]	<= 8'b00000000;								// 
				hyper_shift_reg[31:24]	<= 8'b00000001;								// must be 0x01 
				hyper_shift_reg[39:32]	<= 8'b00000000;								// 
				hyper_shift_reg[47:40]  <= 8'b01100000; 							// reg write
        end
end        


		// avalon read transaction
		if(avl_mem_read && !avl_mem_write && !hyper_recovery_reg && init_done_int) begin

			// hyperbus transaction is ended
			if(hyper_start_rd && hyper_end) begin
				hyper_start_rd			= 1'b0;
				avl_mem_readdatavalid_s	= 1'b1;
				avl_mem_wr_rd_ready		<= 1'b1;
				hyper_recovery_reg		<= 1'b1;
			end

			// not started yet
			if(!hyper_start_rd && !hyper_busy) begin
				avl_mem_wr_rd_ready		<= 1'b0;
				hyper_start_rd			= 1'b1;

// HyperRAM read
if (G_DEVICE == DEV_HYPERRAM) begin	
				hyper_shift_reg[0]	    <= 1'b0;									       // 
				hyper_shift_reg[15:3]	<= 13'b0000000000000;						       // reserved				
`ifdef BYTEADDR
				hyper_shift_reg[2:1]	<= avl_mem_address[3:2];					       // address
				hyper_shift_reg[44:16]	<= { {(33-G_ADDRBITS){1'b0}}, avl_mem_address[22:4] };      // address
`else
				hyper_shift_reg[2:1]	<= avl_mem_address[1:0];					       // address
				hyper_shift_reg[44:16]	<= { {(33-G_ADDRBITS){1'b0}}, avl_mem_address[20:2] };      // address
`endif				
				hyper_shift_reg[45]		<= 1'b1;									       // 0 - wrappedburst, 1 - linear burst
				hyper_shift_reg[46]		<= 1'b0;              						       // 0 - memory space, 1 - register space
				hyper_shift_reg[47]		<= 1'b1; 									       // 0 - write, 1 - read
end

// HyperFlash read
if (G_DEVICE == DEV_HYPERFLASH) begin	
				hyper_shift_reg[0]	    <= 1'b0;									       // 
				hyper_shift_reg[15:3]	<= 13'b0000000000000;						       // reserved				
`ifdef BYTEADDR
				hyper_shift_reg[2:1]	<= avl_mem_address[3:2];					       // address
				hyper_shift_reg[44:16]	<= { {(33-G_ADDRBITS){1'b0}}, avl_mem_address[22:4] };      // address
`else
				hyper_shift_reg[2:1]	<= avl_mem_address[1:0];					       // address
				hyper_shift_reg[44:16]	<= { {(33-G_ADDRBITS){1'b0}}, avl_mem_address[20:2] };      // address
`endif				
				hyper_shift_reg[45]		<= 1'b1;									       // 0 - wrappedburst, 1 - linear burst
				hyper_shift_reg[46]		<= 1'b0;              						       // 0 - memory space, 1 - register space
				hyper_shift_reg[47]		<= 1'b1; 									       // 0 - write, 1 - read
end


				
			end
		end else begin
			avl_mem_readdatavalid_s	= 1'b0; // we need to clear this really important
		end
		
		// avalon write transaction
		if(!avl_mem_read && avl_mem_write && !hyper_recovery_reg && init_done_int) begin
			// hyperbus transaction is ended
			if(hyper_start_wr && hyper_end) begin
				hyper_start_wr			= 1'b0;
				avl_mem_wr_rd_ready		<= 1'b1;
				hyper_recovery_reg		<= 1'b1; // is it needed here?
			end

			// not started yet
			if(!hyper_start_wr && !hyper_busy) begin
				avl_mem_wr_rd_ready		<= 1'b0;
				hyper_start_wr			= 1'b1;

if (G_DEVICE == DEV_HYPERRAM) begin				
				hyper_shift_reg[0]	    <= 1'b0;									       // 
				hyper_shift_reg[15:3]	<= 13'b0000000000000;						       // reserved

`ifdef BYTEADDR
				hyper_shift_reg[2:1]	<= avl_mem_address[3:2];					       // address
				hyper_shift_reg[44:16]	<= { {(33-G_ADDRBITS){1'b0}}, avl_mem_address[22:4] };      // address
`else
				hyper_shift_reg[2:1]	<= avl_mem_address[1:0];					       // address
				hyper_shift_reg[44:16]	<= { {(33-G_ADDRBITS){1'b0}}, avl_mem_address[20:2] };      // address
`endif				

				hyper_shift_reg[45]		<= 1'b1;									// 0 - wrappedburst, 1 - linear burst
				hyper_shift_reg[46]		<= 1'b0; 					                // 0 - memory space, 1 - register space
				hyper_shift_reg[47]		<= 1'b0; 									// 0 - write, 1 - read
end

if (G_DEVICE == DEV_HYPERFLASH) begin				
				hyper_shift_reg[0]	    <= avl_mem_byteenable[2];					       // writing to offset+2 
				hyper_shift_reg[15:3]	<= 13'b0000000000000;						       // reserved

`ifdef BYTEADDR
				hyper_shift_reg[2:1]	<= avl_mem_address[3:2];					       // address
				hyper_shift_reg[44:16]	<= { {(33-G_ADDRBITS){1'b0}}, avl_mem_address[22:4] };      // address
`else
				hyper_shift_reg[2:1]	<= avl_mem_address[1:0];					       // address
				hyper_shift_reg[44:16]	<= { {(33-G_ADDRBITS){1'b0}}, avl_mem_address[20:2] };      // address
`endif				

				hyper_shift_reg[45]		<= 1'b0;									// 0 - wrappedburst, 1 - linear burst
				hyper_shift_reg[46]		<= 1'b0; 					                // 0 - memory space, 1 - register space
				hyper_shift_reg[47]		<= 1'b0; 									// 0 - write, 1 - read
end
				
			end
		end
		
		// recovery
		if(!avl_mem_read && !avl_mem_write) begin
			hyper_recovery_reg			<= 1'b0;
			avl_mem_readdatavalid_s		= 1'b0;
			hyper_start_rd				= 1'b0;
			hyper_start_wr				= 1'b0;
			avl_mem_wr_rd_ready			<= 1'b0;
		end
	end
end


endmodule
