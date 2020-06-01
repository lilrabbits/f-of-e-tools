`timescale 1ns/1ns

module adder_testbench;
	reg[31:0] input1;
	reg[31:0] input2;
	wire[31:0] data_out;

	dsp_adder adder_inst(
		.input1(input1),
		.input2(input2),
		.out(data_out)
	);

	initial begin
	#1 	input1 = 32'd0;
 		input2 = 32'd0;

	#1 	input1 = 32'd0;
 		input2 = 32'd10;

	#1 	input1 = 32'd1000;
 		input2 = 32'd10;

	#1 	input1 = 32'd10230;
 		input2 = 32'd1602;

	#1 	input1 = 32'd13413;
 		input2 = 32'd12823;

	#1 	input1 = 32'd151;
 		input2 = 32'd153321;
	end

	initial begin
		$dumpfile ("adder.vcd");
		$dumpvars;
	end
endmodule
