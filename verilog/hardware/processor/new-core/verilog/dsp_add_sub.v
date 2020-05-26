/*
 *	Description:
 *
 *		This module implements an add or subtractor using the DSP blocks on the iCE 40 FPGA
 */



module dsp_add_sub(input1, input2, add_N, out);
	input [31:0]	input1;
	input [31:0]	input2;
    input           add_N;
	output [31:0]	out;

    SB_MAC16 dsp_adder_i(
        .C(input1[31:16]),
        .A(input2[31:16]),
        .B(input2[15:0]),
        .D(input1[15:0]),
        .O(out),
        .ADDSUBTOP(add_N),
        .OLOADTOP(1'b0),
        .ADDSUBBOT(add_N),
        .OLOADBOT(1'b0),
    );

    defparam dsp_adder_i.C_REG = 1'b0;
    defparam dsp_adder_i.A_REG = 1'b0;
    defparam dsp_adder_i.B_REG = 1'b0;
    defparam dsp_adder_i.D_REG = 1'b0;
    defparam dsp_adder_i.TOPOUTPUT_SELECT = 2'b00;
    defparam dsp_adder_i.TOPADDSUB_LOWERINPUT = 2'b00;
    defparam dsp_adder_i.TOPADDSUB_UPPERINPUT = 1'b1;
    defparam dsp_adder_i.TOPADDSUB_CARRYSELECT = 2'b10;
    defparam dsp_adder_i.BOTOUTPUT_SELECT = 2'b00;
    defparam dsp_adder_i.BOTADDSUB_LOWERINPUT = 2'b00;
    defparam dsp_adder_i.BOTADDSUB_UPPERINPUT = 1'b1;
    defparam dsp_adder_i.BOTADDSUB_CARRYSELECT = 2'b00;
endmodule