`timescale 1ns/1ns

module and_testbench;
	reg[31:0] input1;
	reg[31:0] input2;
    reg[31:0] input3;
	reg[31:0] input4;
	wire[31:0] data_out;

	and_gate and_inst(
		.input1(input1),
		.input2(input2),
        .input3(input3),
        .input4(input4),
		.out(data_out)
	);

	initial begin
	#1 	input1 = 32'b0;
 		input2 = 32'b0;
        input3 = 32'b1;
        input4 = 32'b1;

	#1 	input1 = 32'b0;
 		input2 = 32'b1;
        input3 = 32'b1;
        input4 = 32'b1;

    #1 	input1 = 32'b1;
 		input2 = 32'b0;
        input3 = 32'b1;
        input4 = 32'b1;
    
    #1 	input1 = 32'b1;
 		input2 = 32'b1;
        input3 = 32'b1;
        input4 = 32'b1;
	end

	initial begin
		$dumpfile ("and.vcd");
		$dumpvars;
	end
endmodule