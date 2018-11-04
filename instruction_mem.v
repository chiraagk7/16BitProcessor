`timescale 1ns / 1ps

/**
 * Module: instruction_mem
 * 
 * The instruction memory model for the processor.
 * 
 */
module instruction_mem(
	input [15:0] PC,
	output reg [15:0] instruction
);

	always @(*) begin
		case(PC)
			// TEST PROGRAM FOR STEPS 1 &2
			// Corresponds to test1.asm
	0: instruction = 16'b0000000000000000;   //NOP
         2: instruction = 16'b0011000110010000;    // MOVIH $0, #0x90
         4: instruction = 16'b0011001110111110;    // MOVIH $1, 0xBE
         6: instruction = 16'b0011001011101111;   // MOVIL $1, #0xEF
         8: instruction = 16'b0111001000000000;  // ST $1, $0, #0
			
			// TEST PROGRAM FOR STEP 3
			// Corresponds to test2.asm
			/*
         0: instruction = 16'b0011010000000001;
         2: instruction = 16'b0011010101000000;
         4: instruction = 16'b0010010010000001;
         6: instruction = 16'b0011011000000101;
         8: instruction = 16'b0011011110000000;
         10: instruction = 16'b0001100010011000;
         12: instruction = 16'b0001100100100001;
         14: instruction = 16'b0010011011000010;
         16: instruction = 16'b0010011011000010;
         18: instruction = 16'b0001101100010010;
         20: instruction = 16'b0001101011100011;
         22: instruction = 16'b0011110110101010;
         24: instruction = 16'b0011110010111011;
         26: instruction = 16'b0001111101110100;
         28: instruction = 16'b0001111111010101;
         30: instruction = 16'b0001111111101110;
         32: instruction = 16'b0010111111000000;
         34: instruction = 16'b0010110111000011;
         36: instruction = 16'b0001100110111111;
         38: instruction = 16'b0101100100110001;
         40: instruction = 16'b0100011011110101;
         42: instruction = 16'b0001001100011110;
         44: instruction = 16'b0011000110010000;
         46: instruction = 16'b0011000000000000;
         48: instruction = 16'b0111001000000000;
			*/
			
			// FINAL TEST PROGRAM
			// Corresponds to test3.asm
			/*
         0: instruction = 16'b0011000110000000;
         2: instruction = 16'b0011100000000001;
         4: instruction = 16'b0001010011100000;
         6: instruction = 16'b1011000000011010;
         8: instruction = 16'b0010100011000011;
         10: instruction = 16'b0010011010000011;
         12: instruction = 16'b0011101110010000;
         14: instruction = 16'b0111010101000000;
         16: instruction = 16'b0011101111110000;
         18: instruction = 16'b0110110101000000;
         20: instruction = 16'b0100110110000001;
         22: instruction = 16'b0110001000000000;
         24: instruction = 16'b0111001000000000;
         26: instruction = 16'b0110111101000000;
         28: instruction = 16'b1010111110111000;
         30: instruction = 16'b1001111110100100;
         32: instruction = 16'b0000000000000000;
         34: instruction = 16'b0011101110010000;
         36: instruction = 16'b0011101000000010;
         38: instruction = 16'b0011000000000001;
         40: instruction = 16'b0111000101000000;

*/

			default: instruction = 16'h0;
		endcase
	end

endmodule
