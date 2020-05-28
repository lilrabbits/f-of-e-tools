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
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/



/*
 *	Description:
 *
 *		This module implements an AND gate for use by the branch unit
 *		and program counter increment among other things.
 */

module and_gate #(parameter n = 32) (
    input [31:0]	input1, input2, input3, input4,
	output [31:0]   out
    );

    // SB_LUT4 : 4-input Look-Up Table  

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and0 ( 
		.O (out[0]),    // output 
		.I0 (input1[0]),    // data input 0 
		.I1 (input2[0]),    // data input 1 
		.I2 (input3[0]),    // data input 2 
		.I3 (input4[0])     // data input 3 
		);  

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and1 ( 
		.O (out[1]),    // output 
		.I0 (input1[1]),    // data input 0 
		.I1 (input2[1]),    // data input 1 
		.I2 (input3[1]),    // data input 2 
		.I3 (input4[1])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and2 ( 
		.O (out[2]),    // output 
		.I0 (input1[2]),    // data input 0 
		.I1 (input2[2]),    // data input 1 
		.I2 (input3[2]),    // data input 2 
		.I3 (input4[2])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and3 ( 
		.O (out[3]),    // output 
		.I0 (input1[3]),    // data input 0 
		.I1 (input2[3]),    // data input 1 
		.I2 (input3[3]),    // data input 2 
		.I3 (input4[3])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and4 ( 
		.O (out[4]),    // output 
		.I0 (input1[4]),    // data input 0 
		.I1 (input2[4]),    // data input 1 
		.I2 (input3[4]),    // data input 2 
		.I3 (input4[4])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and5 ( 
		.O (out[5]),    // output 
		.I0 (input1[5]),    // data input 0 
		.I1 (input2[5]),    // data input 1 
		.I2 (input3[5]),    // data input 2 
		.I3 (input4[5])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and6 ( 
		.O (out[6]),    // output 
		.I0 (input1[6]),    // data input 0 
		.I1 (input2[6]),    // data input 1 
		.I2 (input3[6]),    // data input 2 
		.I3 (input4[6])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and7 ( 
		.O (out[7]),    // output 
		.I0 (input1[7]),    // data input 0 
		.I1 (input2[7]),    // data input 1 
		.I2 (input3[7]),    // data input 2 
		.I3 (input4[7])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and8 ( 
		.O (out[8]),    // output 
		.I0 (input1[8]),    // data input 0 
		.I1 (input2[8]),    // data input 1 
		.I2 (input3[8]),    // data input 2 
		.I3 (input4[8])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and9 ( 
		.O (out[9]),    // output 
		.I0 (input1[9]),    // data input 0 
		.I1 (input2[9]),    // data input 1 
		.I2 (input3[9]),    // data input 2 
		.I3 (input4[9])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and10 ( 
		.O (out[10]),    // output 
		.I0 (input1[10]),    // data input 0 
		.I1 (input2[10]),    // data input 1 
		.I2 (input3[10]),    // data input 2 
		.I3 (input4[10])     // data input 3 
		);  

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and11 ( 
		.O (out[11]),    // output 
		.I0 (input1[11]),    // data input 0 
		.I1 (input2[11]),    // data input 1 
		.I2 (input3[11]),    // data input 2 
		.I3 (input4[11])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and12 ( 
		.O (out[12]),    // output 
		.I0 (input1[12]),    // data input 0 
		.I1 (input2[12]),    // data input 1 
		.I2 (input3[12]),    // data input 2 
		.I3 (input4[12])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and13 ( 
		.O (out[13]),    // output 
		.I0 (input1[13]),    // data input 0 
		.I1 (input2[13]),    // data input 1 
		.I2 (input3[13]),    // data input 2 
		.I3 (input4[13])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and14 ( 
		.O (out[14]),    // output 
		.I0 (input1[14]),    // data input 0 
		.I1 (input2[14]),    // data input 1 
		.I2 (input3[14]),    // data input 2 
		.I3 (input4[14])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and15 ( 
		.O (out[15]),    // output 
		.I0 (input1[15]),    // data input 0 
		.I1 (input2[15]),    // data input 1 
		.I2 (input3[15]),    // data input 2 
		.I3 (input4[15])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and16 ( 
		.O (out[16]),    // output 
		.I0 (input1[16]),    // data input 0 
		.I1 (input2[16]),    // data input 1 
		.I2 (input3[16]),    // data input 2 
		.I3 (input4[16])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and17 ( 
		.O (out[17]),    // output 
		.I0 (input1[17]),    // data input 0 
		.I1 (input2[17]),    // data input 1 
		.I2 (input3[17]),    // data input 2 
		.I3 (input4[17])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and18 ( 
		.O (out[18]),    // output 
		.I0 (input1[18]),    // data input 0 
		.I1 (input2[18]),    // data input 1 
		.I2 (input3[18]),    // data input 2 
		.I3 (input4[18])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and19 ( 
		.O (out[19]),    // output 
		.I0 (input1[19]),    // data input 0 
		.I1 (input2[19]),    // data input 1 
		.I2 (input3[19]),    // data input 2 
		.I3 (input4[19])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and20 ( 
		.O (out[20]),    // output 
		.I0 (input1[20]),    // data input 0 
		.I1 (input2[20]),    // data input 1 
		.I2 (input3[20]),    // data input 2 
		.I3 (input4[20])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and21 ( 
		.O (out[21]),    // output 
		.I0 (input1[21]),    // data input 0 
		.I1 (input2[21]),    // data input 1 
		.I2 (input3[21]),    // data input 2 
		.I3 (input4[21])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and22 ( 
		.O (out[22]),    // output 
		.I0 (input1[22]),    // data input 0 
		.I1 (input2[22]),    // data input 1 
		.I2 (input3[22]),    // data input 2 
		.I3 (input4[22])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and23 ( 
		.O (out[23]),    // output 
		.I0 (input1[23]),    // data input 0 
		.I1 (input2[23]),    // data input 1 
		.I2 (input3[23]),    // data input 2 
		.I3 (input4[23])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and24 ( 
		.O (out[24]),    // output 
		.I0 (input1[24]),    // data input 0 
		.I1 (input2[24]),    // data input 1 
		.I2 (input3[24]),    // data input 2 
		.I3 (input4[24])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and25 ( 
		.O (out[25]),    // output 
		.I0 (input1[25]),    // data input 0 
		.I1 (input2[25]),    // data input 1 
		.I2 (input3[25]),    // data input 2 
		.I3 (input4[25])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and26 ( 
		.O (out[26]),    // output 
		.I0 (input1[26]),    // data input 0 
		.I1 (input2[26]),    // data input 1 
		.I2 (input3[26]),    // data input 2 
		.I3 (input4[26])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and27 ( 
		.O (out[27]),    // output 
		.I0 (input1[27]),    // data input 0 
		.I1 (input2[27]),    // data input 1 
		.I2 (input3[27]),    // data input 2 
		.I3 (input4[27])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and28 ( 
		.O (out[28]),    // output 
		.I0 (input1[28]),    // data input 0 
		.I1 (input2[28]),    // data input 1 
		.I2 (input3[28]),    // data input 2 
		.I3 (input4[28])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and29 ( 
		.O (out[29]),    // output 
		.I0 (input1[29]),    // data input 0 
		.I1 (input2[29]),    // data input 1 
		.I2 (input3[29]),    // data input 2 
		.I3 (input4[29])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and30 ( 
		.O (out[30]),    // output 
		.I0 (input1[30]),    // data input 0 
		.I1 (input2[30]),    // data input 1 
		.I2 (input3[30]),    // data input 2 
		.I3 (input4[30])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and31 ( 
		.O (out[31]),    // output 
		.I0 (input1[31]),    // data input 0 
		.I1 (input2[31]),    // data input 1 
		.I2 (input3[31]),    // data input 2 
		.I3 (input4[31])     // data input 3 
		); 

endmodule