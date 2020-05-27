`timescale 1ns/1ns

module or_testbench;
	reg[31:0] input1;
	reg[31:0] input2;
    reg[31:0] input3;
	reg[31:0] input4;
	wire[31:0] data_out;

	or_gate or_inst(
		.input1(input1),
		.input2(input2),
        .input3(input3),
        .input4(input4),
		.out(data_out)
	);

	initial begin
	#1 	input1 = 32'b0;
 		input2 = 32'b0;
        input3 = 32'b0;
        input4 = 32'b0;

	#1 	input1 = 32'b0;
 		input2 = 32'b1;
        input3 = 32'b0;
        input4 = 32'b0;

    #1 	input1 = 32'b1;
 		input2 = 32'b0;
        input3 = 32'b0;
        input4 = 32'b0;
    
    #1 	input1 = 32'b1;
 		input2 = 32'b1;
        input3 = 32'b0;
        input4 = 32'b0;
	end

	initial begin
		$dumpfile ("or.vcd");
		$dumpvars;
	end
endmodule