`timescale 1ns/1ns

`define NOP 4'b0000
`define ARITH_2OP 4'b0001
`define ARITH_1OP 4'b0010
`define MOVI 4'b0011
`define ADDI 4'b0100
`define SUBI 4'b0101
`define LOAD 4'b0110
`define STOR 4'b0111
`define BEQ 4'b1000
`define BGE 4'b1001
`define BLE 4'b1010
`define BC 4'b1011
`define J 4'b1100
`define JL 4'b1101
`define INT 4'b1110
`define CONTROL 4'b1111

`define ADD 3'b000
`define ADDC 3'b001
`define SUB 3'b010
`define SUBB 3'b011
`define AND 3'b100
`define OR 3'b101
`define XOR 3'b110
`define XNOR 3'b111

`define NOT 3'b000
`define SHIFTL 3'b001
`define SHIFTR 3'b010
`define CP 3'b011

`define INT_DISABLE 2'b00
`define INT_ENABLE 2'b01 
`define INT_TRIGGER 2'b10

`define RETURN 12'b000000000000
`define STC    12'b000000000001
`define STB    12'b000000000010
`define RESET  12'b101010101010
`define HALT   12'b111111111111


/*
 * Module: program_counter
 * Description: Program counter.
 *              Increments by instruction word (2 bytes) every cycle unless halted.
 *              Changes to the input program counter if a branch or jump is valid. 
 */
module program_counter (
		input clk,
		input clk_en,
		input reset,
		
		output reg [15:0] pc,
		
		input branch_taken,
		input [5:0] branch_immediate, // Needs to be sign extended
		
		input jump_taken,
		input [11:0] jump_immediate // Needs to be sign extended
);

	wire [15:0] signext_branch_imm; 
	wire [15:0] signext_jump_imm; 
	
	reg [15:0] pc_next;
		
	// STEP 4: Sign extend branch_immediate and jump_immediate
	
		
	initial 
		pc <= 16'h0;
		
	always @(*) begin
		// STEP 4 : Logic for pc_next
		pc_next = pc + 2;
	end

	always @(posedge clk) begin
		if (reset)
			pc <= 16'h0;
		if (clk_en) begin
			pc <= pc_next;
		end
	end
endmodule


 /*
 * Module: instruction_decode
 * Description: Decodes the instruction. 
 *		Fields corresponding to Register numbers, ALU operation specifier, and Immediate data must be extracted.
 *		The 1-bit signal corresponding to the instruction must be asserted TRUE.
 *              All logic should be combinational.
 */
module instruction_decode (
	input [15:0] instruction,
	
	output [2:0] alu_func,
	
	output [2:0] destination_reg,
	output [2:0] source_reg1,
	output [2:0] source_reg2,
	
	output [11:0] immediate,
	
	output arith_2op,
	output arith_1op, 
	
	output movi_lower,
	output movi_higher,
	
	output addi,
	output subi,
	
	output load,
	output store,
	
	output branch_eq,
	output branch_ge,
	output branch_le,
	output branch_carry,
	
	output jump,
	
	output stc_cmd,
	output stb_cmd,
	output halt_cmd,
	output rst_cmd
);
	wire [3:0] op_code;
	wire branchinstr;
    assign branchinstr = branch_eq | branch_ge | branch_le | branch_carry;
	assign op_code = instruction[15:12];

	assign alu_func = instruction[2:0];
	assign destination_reg = instruction[11:9];
	assign source_reg1 = branchinstr ? instruction[11:9] : instruction[8:6];
	assign source_reg2 = branchinstr ? instruction[8:6] : instruction[5:3];
//	assign source_reg2 = ~op_code[3] ? instruction[5:3] : instruction[8:6];
	assign immediate = instruction[11:0];

	assign arith_1op = (op_code == `ARITH_1OP);
	assign arith_2op = (op_code == `ARITH_2OP);
	assign movi_lower = ((op_code == `MOVI) && ~instruction[8]);
	assign movi_higher = ((op_code == `MOVI) && instruction[8]);
	assign addi = (op_code == `ADDI);
	assign subi = (op_code == `SUBI);
	assign load = (op_code == `LOAD);
	assign store = (op_code == `STOR);
	assign branch_eq = (op_code == `BEQ);
	assign branch_ge = (op_code == `BGE);
	assign branch_le = (op_code == `BLE);
	assign branch_carry = (op_code == `BC);
	assign jump = (op_code == `J);
	assign stc_cmd = (op_code == `CONTROL && immediate == `STC);
	assign stb_cmd = (op_code == `CONTROL && immediate == `STB);
	assign halt_cmd = (op_code == `CONTROL && immediate == `HALT);
	assign rst_cmd = (op_code == `CONTROL && immediate == `RESET);
		
endmodule




/*
 * Module: reg_file
 * Description: Register file
 */
module reg_file (
	input clk,
	input clk_en,
	input reset,
	
	// Source Register data for 1 and 2 register operations
	input [2:0] source_reg2,
	input [2:0] source_reg1,
	output [15:0] reg1_data,
	output [15:0] reg2_data,
	
	// Destination register and write back command/data for operations that write to a register
	input [2:0] destination_reg,
	input wr_destination_reg,
	input  [15:0] dest_result_data,
	
	// Source register data for STORE operations. Indexed on destination_reg input
	output [15:0] regD_data, 
	
	// Move immediate commands and data.
	input       movi_lower,
	input       movi_higher,
	input [7:0] immediate
);
		
	// STEP 2 - Registers
	reg [15:0] registers [7:0];

	integer i;

	always@(posedge clk) begin
	    if (reset) begin
			for (i = 0; i < 8; i = i+1) begin
				registers[i] <= 16'b0;
			end
		end
		else begin
            if (wr_destination_reg && clk_en) begin
                if (movi_lower) begin
                    registers[destination_reg][7:0] <= immediate;
                end
                else if (movi_higher) begin
                    registers[destination_reg][15:8] <= immediate;
                end
                else begin
                    registers[destination_reg] <= dest_result_data;
                end
    
            end
		end
	end
	
	assign reg1_data = registers[source_reg1];
    assign reg2_data = registers[source_reg2];
    assign regD_data = registers[destination_reg];
	
endmodule


/*
 * Module: alu
 * Description: Arithmetic Logic Unit
 *              This module contains the circuits that perform arithmetic and logic operations for the processor, and is used for the following functions:
 *              - 2 Operand Arithmetic/Logic operations:
 *                  Add, Add with Carry, Subtract, Subtract with Borrow, bitwise AND/OR/XOR/XNOR
 *              - 1 Operand Arithmetic/Logic operations:
 *                  Bitwise NOT, Shift Left, Shift Right, Register Copy
 *              - Add immediate (no carry bit)
 *              - Subtract immediate (no borrow bit)
 *              - Load and Store (Address addition - effectively the same to the ALU as Add immediate)
 *              This module does not contain the adder for the Program Counter, nor does it have the comparator logic for branches
 */
module alu (
	input        clk,
	input        clk_en,
	input        reset,
	
	input        arith_1op,
	input        arith_2op,
	input [2:0]  alu_func,
	input        addi,
	input        subi,
	input        load_or_store,
	input [15:0] reg1_data,
	input [15:0] reg2_data,
	input  [5:0] immediate,
	
	input        stc_cmd,
	input        stb_cmd,
	
	output reg        alu_carry_bit,
	output     [15:0] alu_result
);
	// Carry and Borrow bits are used for math that exceeds 16-bits, to carry or borrow a bit from one operation to the next.
	// ONLY use the carry/borrow bit for ADDC or SUBB, respectively
	// SET the carry/borrow bit for any ADD/ADDI/ADDC/STC or SUB/SUBI/SUBB/STB instruction
	
	reg alu_borrow_bit;
	
	// The full ALU result, including the extra bit for overflow or underflow. Overflow/underflow bit used for carry/borrow
	
	reg [16:0] full_result; 

	// STEP 3 - ALU

	always@(*) begin
	   full_result = 17'b0;
		if (arith_2op) begin
			case (alu_func)
				`ADD: full_result = reg1_data + reg2_data;
				`ADDC: full_result = reg1_data + reg2_data + alu_carry_bit;
				`SUB: full_result = reg1_data - reg2_data;
				`SUBB: full_result = reg1_data - reg2_data - alu_borrow_bit;
				`AND: full_result = reg1_data & reg2_data;
				`OR: full_result = reg1_data | reg2_data;
				`XOR: full_result = reg1_data ^ reg2_data;
				`XNOR: full_result = reg1_data ~^ reg2_data;
			endcase
		end	
		else if (arith_1op) begin
			case (alu_func)
				`NOT: full_result = ~reg1_data;
				`SHIFTL: full_result = reg1_data << 1;
				`SHIFTR: full_result = reg1_data >> 1;
				`CP: full_result = reg1_data;
			endcase
		end
		else  if (addi | load_or_store)
			full_result = reg1_data + immediate;
		 if (subi)
			full_result = reg1_data - immediate;
	end
	
	always@(posedge clk) begin
            if (reset) begin
                alu_carry_bit <= 1'b0;
                alu_borrow_bit <= 1'b0;
            end
            else begin
                if (clk_en) begin
                    if (stc_cmd)
                        alu_carry_bit <= 1'b1;
                    else if (addi | ( (alu_func == `ADDC | alu_func == `ADD) && arith_2op))
                        alu_carry_bit <= full_result[16];
                    if (stb_cmd)
                        alu_borrow_bit <= 1'b1;
                    else if (subi || ((alu_func[2:1] == 2'b01) && arith_2op))
                        alu_borrow_bit <= full_result[16];
                end
            end
        end
	
	assign alu_result = full_result[15:0];
	
endmodule



/*
 * Module: branch comparator
 * Description: Branch comparator. Given a set of operands (register values or machine state, as in BranchOnCarryBit), 
 *              outputs a single flag that is true only if both the current instruction is a branch operation and the branch should be taken.
 */
module branch_comparator (
	input branch_eq,
	input branch_ge,
	input branch_le,
	input branch_carry,
	input [15:0] reg1_data,
	input [15:0] reg2_data,
	input alu_carry_bit,
	output reg branch_taken);
	
	
	// TEMP
	initial
	 branch_taken = 1'b0;
	
	// STEP 4: branch logic
	always@(*) begin
		branch_taken = (branch_eq && (reg1_data == reg2_data)) || (branch_ge && (reg1_data >= reg2_data)) 
			|| (branch_le && (reg1_data <= reg2_data)) || (branch_carry && alu_carry_bit);
	end
endmodule
