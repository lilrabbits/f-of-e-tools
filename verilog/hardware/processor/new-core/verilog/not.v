/*
	Authored 2019-2020, Rae Zhao.

	All rights reserved.
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY out OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/



/*
 *	Description:
 *
 *		This module implements a NOT gate for use by the branch unit
 *		and program counter increment among other things.
 */

module not_gate #(parameter n = 32) (
    input [31:0]	input1_not, input2_not, input3_not, input4_not,
	output [31:0]   out_not
    );

    // SB_LUT4 : 4-input Look-Up Table  

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not0 ( 
		.O (out_not[0]),    // output 
		.I0 (input1_not[0]),    // data input 0 
		.I1 (input2_not[0]),    // data input 1 
		.I2 (input3_not[0]),    // data input 2 
		.I3 (input4_not[0])     // data input 3 
		);  

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not1 ( 
		.O (out_not[1]),    // output 
		.I0 (input1_not[1]),    // data input 0 
		.I1 (input2_not[1]),    // data input 1 
		.I2 (input3_not[1]),    // data input 2 
		.I3 (input4_not[1])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not2 ( 
		.O (out_not[2]),    // output 
		.I0 (input1_not[2]),    // data input 0 
		.I1 (input2_not[2]),    // data input 1 
		.I2 (input3_not[2]),    // data input 2 
		.I3 (input4_not[2])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not3 ( 
		.O (out_not[3]),    //output 
		.I0 (input1_not[3]),    // data input 0 
		.I1 (input2_not[3]),    // data input 1 
		.I2 (input3_not[3]),    // data input 2 
		.I3 (input4_not[3])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not4 ( 
		.O (out_not[4]),    //output 
		.I0 (input1_not[4]),    // data input 0 
		.I1 (input2_not[4]),    // data input 1 
		.I2 (input3_not[4]),    // data input 2 
		.I3 (input4_not[4])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not5 ( 
		.O (out_not[5]),    //output 
		.I0 (input1_not[5]),    // data input 0 
		.I1 (input2_not[5]),    // data input 1 
		.I2 (input3_not[5]),    // data input 2 
		.I3 (input4_not[5])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not6 ( 
		.O (out_not[6]),    //output 
		.I0 (input1_not[6]),    // data input 0 
		.I1 (input2_not[6]),    // data input 1 
		.I2 (input3_not[6]),    // data input 2 
		.I3 (input4_not[6])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not7 ( 
		.O (out_not[7]),    //output 
		.I0 (input1_not[7]),    // data input 0 
		.I1 (input2_not[7]),    // data input 1 
		.I2 (input3_not[7]),    // data input 2 
		.I3 (input4_not[7])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not8 ( 
		.O (out_not[8]),    //output 
		.I0 (input1_not[8]),    // data input 0 
		.I1 (input2_not[8]),    // data input 1 
		.I2 (input3_not[8]),    // data input 2 
		.I3 (input4_not[8])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not9 ( 
		.O (out_not[9]),    //output 
		.I0 (input1_not[9]),    // data input 0 
		.I1 (input2_not[9]),    // data input 1 
		.I2 (input3_not[9]),    // data input 2 
		.I3 (input4_not[9])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not10 ( 
		.O (out_not[10]),    //output 
		.I0 (input1_not[10]),    // data input 0 
		.I1 (input2_not[10]),    // data input 1 
		.I2 (input3_not[10]),    // data input 2 
		.I3 (input4_not[10])     // data input 3 
		);  

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not11 ( 
		.O (out_not[11]),    //output 
		.I0 (input1_not[11]),    // data input 0 
		.I1 (input2_not[11]),    // data input 1 
		.I2 (input3_not[11]),    // data input 2 
		.I3 (input4_not[11])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not12 ( 
		.O (out_not[12]),    //output 
		.I0 (input1_not[12]),    // data input 0 
		.I1 (input2_not[12]),    // data input 1 
		.I2 (input3_not[12]),    // data input 2 
		.I3 (input4_not[12])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not13 ( 
		.O (out_not[13]),    //output 
		.I0 (input1_not[13]),    // data input 0 
		.I1 (input2_not[13]),    // data input 1 
		.I2 (input3_not[13]),    // data input 2 
		.I3 (input4_not[13])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not14 ( 
		.O (out_not[14]),    //output 
		.I0 (input1_not[14]),    // data input 0 
		.I1 (input2_not[14]),    // data input 1 
		.I2 (input3_not[14]),    // data input 2 
		.I3 (input4_not[14])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not15 ( 
		.O (out_not[15]),    //output 
		.I0 (input1_not[15]),    // data input 0 
		.I1 (input2_not[15]),    // data input 1 
		.I2 (input3_not[15]),    // data input 2 
		.I3 (input4_not[15])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not16 ( 
		.O (out_not[16]),    //output 
		.I0 (input1_not[16]),    // data input 0 
		.I1 (input2_not[16]),    // data input 1 
		.I2 (input3_not[16]),    // data input 2 
		.I3 (input4_not[16])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not17 ( 
		.O (out_not[17]),    //output 
		.I0 (input1_not[17]),    // data input 0 
		.I1 (input2_not[17]),    // data input 1 
		.I2 (input3_not[17]),    // data input 2 
		.I3 (input4_not[17])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not18 ( 
		.O (out_not[18]),    //output 
		.I0 (input1_not[18]),    // data input 0 
		.I1 (input2_not[18]),    // data input 1 
		.I2 (input3_not[18]),    // data input 2 
		.I3 (input4_not[18])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not19 ( 
		.O (out_not[19]),    //output 
		.I0 (input1_not[19]),    // data input 0 
		.I1 (input2_not[19]),    // data input 1 
		.I2 (input3_not[19]),    // data input 2 
		.I3 (input4_not[19])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not20 ( 
		.O (out_not[20]),    //output 
		.I0 (input1_not[20]),    // data input 0 
		.I1 (input2_not[20]),    // data input 1 
		.I2 (input3_not[20]),    // data input 2 
		.I3 (input4_not[20])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not21 ( 
		.O (out_not[21]),    //output 
		.I0 (input1_not[21]),    // data input 0 
		.I1 (input2_not[21]),    // data input 1 
		.I2 (input3_not[21]),    // data input 2 
		.I3 (input4_not[21])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not22 ( 
		.O (out_not[22]),    //output 
		.I0 (input1_not[22]),    // data input 0 
		.I1 (input2_not[22]),    // data input 1 
		.I2 (input3_not[22]),    // data input 2 
		.I3 (input4_not[22])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not23 ( 
		.O (out_not[23]),    //output 
		.I0 (input1_not[23]),    // data input 0 
		.I1 (input2_not[23]),    // data input 1 
		.I2 (input3_not[23]),    // data input 2 
		.I3 (input4_not[23])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not24 ( 
		.O (out_not[24]),    //output 
		.I0 (input1_not[24]),    // data input 0 
		.I1 (input2_not[24]),    // data input 1 
		.I2 (input3_not[24]),    // data input 2 
		.I3 (input4_not[24])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not25 ( 
		.O (out_not[25]),    //output 
		.I0 (input1_not[25]),    // data input 0 
		.I1 (input2_not[25]),    // data input 1 
		.I2 (input3_not[25]),    // data input 2 
		.I3 (input4_not[25])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not26 ( 
		.O (out_not[26]),    //output 
		.I0 (input1_not[26]),    // data input 0 
		.I1 (input2_not[26]),    // data input 1 
		.I2 (input3_not[26]),    // data input 2 
		.I3 (input4_not[26])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not27 ( 
		.O (out_not[27]),    //output 
		.I0 (input1_not[27]),    // data input 0 
		.I1 (input2_not[27]),    // data input 1 
		.I2 (input3_not[27]),    // data input 2 
		.I3 (input4_not[27])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not28 ( 
		.O (out_not[28]),    //output 
		.I0 (input1_not[28]),    // data input 0 
		.I1 (input2_not[28]),    // data input 1 
		.I2 (input3_not[28]),    // data input 2 
		.I3 (input4_not[28])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not29 ( 
		.O (out_not[29]),    //output 
		.I0 (input1_not[29]),    // data input 0 
		.I1 (input2_not[29]),    // data input 1 
		.I2 (input3_not[29]),    // data input 2 
		.I3 (input4_not[29])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not30 ( 
		.O (out_not[30]),    //output 
		.I0 (input1_not[30]),    // data input 0 
		.I1 (input2_not[30]),    // data input 1 
		.I2 (input3_not[30]),    // data input 2 
		.I3 (input4_not[30])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'hFF00)) SB_LUT4_not31 ( 
		.O (out_not[31]),    //output 
		.I0 (input1_not[31]),    // data input 0 
		.I1 (input2_not[31]),    // data input 1 
		.I2 (input3_not[31]),    // data input 2 
		.I3 (input4_not[31])     // data input 3 
		); 

endmodule