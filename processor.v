`timescale 1ns/1ns

/*
 * Module: processor
 * Description: The top module of this lab 
 */
module processor (
	input CLK,
	input [7:0] SW,
	input [4:0] BTN,
	input CPU_RESETN,
	output [7:0] LED,
	output [6:0] SEG,
	output DP,
	output [7:0] AN
); 
	
	wire [15:0] right_display_num;
	wire [6:0] display_seg;
	
	reg [15:0] RTC_seconds;	
	
	wire display_clk_en;
	wire seconds_clk_en;
	wire cpu_clk_en;
	
	wire blink_clk_en;
	wire blink_en;
	reg blink;
	
	// PC
	wire [15:0] PC;
	
	// Instruction mem
	wire [15:0] instruction;
	
	// Instruction decode
	wire  [2:0] alu_func;
	wire  [2:0] destination_reg;
	wire  [2:0] source_reg1;
	wire  [2:0] source_reg2;
	wire [11:0] immediate;
	wire        arith_2op;
	wire        arith_1op;
	wire        movi_lower;
	wire        movi_higher;
	wire        addi;
	wire        subi;
	wire        load;
	wire        store;
	wire        branch_eq;
	wire        branch_ge;
	wire        branch_le;
	wire        branch_carry;
	wire        jump;
	wire        stc_cmd;
	wire        stb_cmd;
	wire        halt_cmd;
	wire        rst_cmd;
	
	// Reg_file
	wire [15:0] reg1_data;
	wire [15:0] reg2_data;
	wire [15:0] regD_data;
	
	// ALU
	wire [15:0] alu_result;
	wire alu_carry_bit;
	
	// Branch comparator
	wire branch_taken;
	
	// Data memory
	wire [15:0] dmem_rdata;
	
	// RESET
	wire reset = ~CPU_RESETN | rst_cmd;
	
	// halt 
	reg halted;
	
	assign DP = 1'b1;
	assign SEG = (~blink_en | blink) ? display_seg : 7'b1111111;
	
	// Program Counter instance
	program_counter Iprogram_counter (
		.clk(CLK),
		.clk_en(~halted && cpu_clk_en),
		.reset(reset),
		.pc(PC),
		.branch_taken(branch_taken),
		.branch_immediate(immediate[5:0]),
		.jump_taken(jump | jump_and_link),
		.jump_immediate(immediate[11:0])
	);
	
	// Instruction Memory instance
	instruction_mem Iinstruction_mem (
		.PC(PC),
		.instruction(instruction)
	);
	
	// Instruction decode instance
	instruction_decode Iinstruction_decode (
		.instruction(instruction),
		.alu_func(alu_func),
		.destination_reg(destination_reg),
		.source_reg1(source_reg1),
		.source_reg2(source_reg2),
		.immediate(immediate),
		.arith_2op(arith_2op),
		.arith_1op(arith_1op),
		.movi_lower(movi_lower),
		.movi_higher(movi_higher),
		.addi(addi),
		.subi(subi),
		.load(load),
		.store(store),
		.branch_eq(branch_eq),
		.branch_ge(branch_ge),
		.branch_le(branch_le),
		.branch_carry(branch_carry),
		.jump(jump),
		.stc_cmd(stc_cmd),
		.stb_cmd(stb_cmd),
		.halt_cmd(halt_cmd),
		.rst_cmd(rst_cmd)
	);
	
	reg_file Ireg_file (
		.clk(CLK),
		.clk_en(~halted & cpu_clk_en),
		.reset(reset),
		.destination_reg(destination_reg),
		.dest_result_data(load ? dmem_rdata : alu_result),
		.wr_destination_reg(arith_2op | arith_1op | movi_lower | movi_higher | addi | subi | load),
		.source_reg1(source_reg1),
		.source_reg2(source_reg2),
		.reg1_data(reg1_data),
		.reg2_data(reg2_data),
		.regD_data(regD_data),
		.movi_lower(movi_lower),
		.movi_higher(movi_higher),
		.immediate(immediate[7:0])
	);
	
	alu Ialu (
		.clk(CLK),
		.clk_en(~halted & cpu_clk_en),
		.reset(reset),
		.arith_1op(arith_1op),
		.arith_2op(arith_2op),
		.alu_func(alu_func),
		.addi(addi),
		.subi(subi),
		.load_or_store(load | store),
		.immediate(immediate[5:0]),
		.reg1_data(reg1_data),
		.reg2_data(reg2_data),
		.stc_cmd(stc_cmd),
		.stb_cmd(stb_cmd),
		.alu_carry_bit(alu_carry_bit),
		.alu_result(alu_result)
	);
	
	branch_comparator Ibranch_comparator (
		.branch_eq(branch_eq),
		.branch_ge(branch_ge),
		.branch_le(branch_le),
		.branch_carry(branch_carry),
		.reg1_data(reg1_data),
		.reg2_data(reg2_data),
		.alu_carry_bit(alu_carry_bit),
		.branch_taken(branch_taken));
	
	data_mem Idata_mem (
		.clk_pi(CLK),
		.clk_en(~halted & cpu_clk_en),
		.reset_pi(reset),
		// Commands and data from instruction_decode and reg_file
		.write_pi(store),
		.wdata_pi(regD_data),
		.rdata_po(dmem_rdata),
		.addr_pi(alu_result),
		// Memory mapped 
		.bt_pi(BTN),
		.sw_pi(SW),
		.rtc_pi(RTC_seconds),
		.led_po(LED),
		.right_display_num_po(right_display_num),
		.blink_en_po(blink_en)
	);
	
	display_clkdiv Idisplay_clkdiv (
		.clk_pi(CLK),
		.clk_en_po(display_clk_en));
	
	seconds_clkdiv Iseconds_clkdiv (
		.clk_pi(CLK),
		.clk_en_po(seconds_clk_en));
		
	display_clkdiv #(.SIZE(12)) Icpu_clkdiv (
		.clk_pi(CLK),
		.clk_en_po(cpu_clk_en));
		
	display_clkdiv #(.SIZE(23)) Iblink_clkdiv (
		.clk_pi(CLK),
		.clk_en_po(blink_clk_en));
		
	sevenSegDisplay IsevenSegDisplay (
		.clk_pi(CLK),
		.clk_en_pi(display_clk_en),
		.right_num_pi(right_display_num),
		.left_num_pi(16'h00),
		.seg_po(display_seg),
		.an_po(AN));
	
	initial begin
		blink <= 0;
		RTC_seconds <= 16'h0;
		halted <= 1'b0;
	end
	
   always @(posedge CLK) begin
		if(reset) begin
			RTC_seconds <= 16'h0;
			halted <= 1'b0;
		end
	
		if(halt_cmd)
			halted <= 1'b1;
	
	   if(blink_clk_en)
		  blink <= ~blink;
		  
		if(seconds_clk_en)
			RTC_seconds <= RTC_seconds + 1;
	end
	
endmodule // alarm_clock

/* 
 * Module: segmentFormatter
 * 
 * Description: Combinational logic for the seven segment bits of a digit of the seven segment display
 *
 * 0 - LSB of disp_po
 * 6 - MSB of disp_po
 *      --(0)   
 * (5)|      |(1)
 *      --(6)
 * (4)|      |(2)
 *      --(3) 
 * 
 * disp_po is active low 
 */
module segmentFormatter(
    input enable, 
	input [3:0] num_pi,
	output reg [6:0] disp_po
);
	always @(*) begin
	    if (enable) begin
		case(num_pi)
			4'h0: disp_po = 7'b1000000;
			4'h1: disp_po = 7'b1111001;
			4'h2: disp_po = 7'b0100100;
			4'h3: disp_po = 7'b0110000;
			4'h4: disp_po = 7'b0011001;
			4'h5: disp_po = 7'b0010010;
			4'h6: disp_po = 7'b0000010;
			4'h7: disp_po = 7'b1111000;
			4'h8: disp_po = 7'b0000000;
			4'h9: disp_po = 7'b0010000;
			4'hA: disp_po = 7'b0001000;
			4'hB: disp_po = 7'b0000011;
			4'hC: disp_po = 7'b1000110;
			4'hD: disp_po = 7'b0100001;
			4'hE: disp_po = 7'b0000110;
			4'hF: disp_po = 7'b0001110;
		endcase
		end
		else
		  disp_po = 7'b1111111; 
	end
endmodule // segmentFormatter

/*
 * Module: sevenSegDisplay
 * Description: Formats an input 16 bit number for the four digit seven-segment display
 */
module sevenSegDisplay(
	input clk_pi,
	input clk_en_pi,
	input[15:0] right_num_pi,
	input[15:0] left_num_pi,
	output reg [6:0] seg_po,
	output reg [7:0] an_po
);
	
	wire [6:0] disp0, disp1, disp2, disp3, disp4, disp5, disp6, disp7;
		
	segmentFormatter IsegmentFormat0 (.enable(1'b1),  .num_pi(right_num_pi[3:0]),   .disp_po(disp0));
	segmentFormatter IsegmentFormat1 (.enable(1'b1),  .num_pi(right_num_pi[7:4]),   .disp_po(disp1));
	segmentFormatter IsegmentFormat2 (.enable(1'b1),  .num_pi(right_num_pi[11:8]),  .disp_po(disp2));
	segmentFormatter IsegmentFormat3 (.enable(1'b1),  .num_pi(right_num_pi[15:12]), .disp_po(disp3));
	segmentFormatter IsegmentFormat4 (.enable(1'b0),  .num_pi(4'h0), .disp_po(disp4));
	segmentFormatter IsegmentFormat5 (.enable(1'b0),  .num_pi(4'h0), .disp_po(disp5));
	segmentFormatter IsegmentFormat6 (.enable(1'b0),  .num_pi(4'h0), .disp_po(disp6));
	segmentFormatter IsegmentFormat7 (.enable(1'b0),  .num_pi(4'h0), .disp_po(disp7));
	
	initial begin
		seg_po <= 7'h7F;
		an_po <= 4'b1111;
	end
	
	always @(posedge clk_pi) begin
		if(clk_en_pi) begin
		
	       case(an_po) 
                8'b11111110: begin
            	   seg_po <= disp1;
            	   an_po  <= 8'b11111101;
                end
                8'b11111101: begin
                   seg_po <= disp2;
            	   an_po  <= 8'b11111011;
                end
                8'b11111011: begin
        	       seg_po <= disp3;
        	       an_po  <= 8'b11110111;
                end
                8'b11110111: begin
                    seg_po <= disp4;
                    an_po  <= 8'b11101111;
                end
                8'b11101111: begin
                    seg_po <= disp5;
                    an_po  <= 8'b11011111;
                end
                8'b11011111: begin
                    seg_po <= disp6;
                    an_po  <= 8'b10111111;
                end
                8'b10111111: begin
                    seg_po <= disp7;
                    an_po  <= 8'b01111111;
                end
            default: begin
            	seg_po <= disp0;
            	an_po <= 8'b11111110;
            end
			endcase
		end // clk_en
	end // always @(posedge clk_pi)
endmodule // sevenSegDisplay


/*
 * Module: display_clkdiv
 * Description: Generates a clk_en signal that can be used to effectively divide the clock used elsewhere
 *              The seven segment display is not particularly visible with the full 50Mhz clock
 *
 * Parameterized to experiment with different clock frequencies for the display
 */
module display_clkdiv (
	input clk_pi,
	output clk_en_po
);
	
	parameter SIZE = 10;
	
	reg [SIZE-1:0] counter;
		
	initial begin
		counter <= 0;
	end
	
	always @(posedge clk_pi) begin
		counter = counter + 1;
	end
	
	assign clk_en_po = (counter == {SIZE{1'b0}}); 
	
endmodule // display_clkdiv

/*
 * Module: seconds_clkdiv
 * Description: Generates a clk_en signal that triggers once per second
 */
module seconds_clkdiv (
	input clk_pi,
	output clk_en_po
);
	reg [31:0] counter;

	initial begin
		counter <= 32'h0;
	end
	
	always @(posedge clk_pi) begin
	if(counter == 32'h5F5E100)  
			counter <= 32'h0;
		else
			counter <= counter + 1;
	end
	
	assign clk_en_po = (counter == 32'h0); 
	
endmodule // seconds_clock
