`timescale 1ns / 1ps

/**
 * Module: data_mem
 * 
 * The data memory model for the processor.
 * 
 * General purpose registers could be added, but for now this is just a memory mapping for board components:
 *   0x8000 - Read from Switches, Write to LEDs
 *   0x9000 - Write to 7-segment display
 *
 */
module data_mem(
	input             clk_pi,   // 100 MHz clk
	input             clk_en,   // Clock enable
	input             reset_pi, // synchronous reset
	
	input             write_pi, // write enable
	input      [15:0] wdata_pi, // write data
	output reg [15:0] rdata_po, // read data
	input      [15:0] addr_pi,  // address
	
	input       [4:0] bt_pi, // Buttons
	input       [7:0] sw_pi, // Switches
	input      [15:0] rtc_pi, // Real-time clock. Time since last power-on or reset in seconds.
	output reg  [7:0] led_po, // LEDs
	output reg [15:0] right_display_num_po, // Display number for 7-seg display
	output reg        blink_en_po
);
	
	initial begin
		led_po <= 8'h0;
		right_display_num_po <= 16'h0;
		blink_en_po <= 1'b0;
	end
	
	always @(*) begin
			rdata_po = 16'h0;			
			if (addr_pi == 16'h8000)
				rdata_po[7:0] = sw_pi;
			if (addr_pi == 16'hF000)
				rdata_po = rtc_pi;
		end
	
	always @(posedge clk_pi) begin

		if (reset_pi) begin
			led_po <= 8'h0;
			right_display_num_po <= 16'h0;
			blink_en_po <= 1'b0;
		end 
		
		else if (write_pi && clk_en) begin			
				if (addr_pi == 16'h8000)
					led_po <= wdata_pi[7:0];
			
				if (addr_pi == 16'h9000)
					right_display_num_po <= wdata_pi;
				
				if (addr_pi == 16'h9002) 
					blink_en_po <= wdata_pi[0];
				
		end
	end
endmodule
