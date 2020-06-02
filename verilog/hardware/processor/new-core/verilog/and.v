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
 *		This module implements an AND gate for use by the branch unit
 *		and program counter increment among other things.
 */

module and_gate #(parameter n = 32) (
    input [31:0]	input1_and, input2_and, input3_and, input4_and,
	output [31:0]   out_and
    );

    // SB_LUT4 : 4-input Look-Up Table  

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and0 ( 
		.O (out_and[0]),    // output 
		.I0 (input1_and[0]),    // data input 0 
		.I1 (input2_and[0]),    // data input 1 
		.I2 (input3_and[0]),    // data input 2 
		.I3 (input4_and[0])     // data input 3 
		);  

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and1 ( 
		.O (out_and[1]),    // output 
		.I0 (input1_and[1]),    // data input 0 
		.I1 (input2_and[1]),    // data input 1 
		.I2 (input3_and[1]),    // data input 2 
		.I3 (input4_and[1])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and2 ( 
		.O (out_and[2]),    // output 
		.I0 (input1_and[2]),    // data input 0 
		.I1 (input2_and[2]),    // data input 1 
		.I2 (input3_and[2]),    // data input 2 
		.I3 (input4_and[2])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and3 ( 
		.O (out_and[3]),    //output 
		.I0 (input1_and[3]),    // data input 0 
		.I1 (input2_and[3]),    // data input 1 
		.I2 (input3_and[3]),    // data input 2 
		.I3 (input4_and[3])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and4 ( 
		.O (out_and[4]),    //output 
		.I0 (input1_and[4]),    // data input 0 
		.I1 (input2_and[4]),    // data input 1 
		.I2 (input3_and[4]),    // data input 2 
		.I3 (input4_and[4])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and5 ( 
		.O (out_and[5]),    //output 
		.I0 (input1_and[5]),    // data input 0 
		.I1 (input2_and[5]),    // data input 1 
		.I2 (input3_and[5]),    // data input 2 
		.I3 (input4_and[5])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and6 ( 
		.O (out_and[6]),    //output 
		.I0 (input1_and[6]),    // data input 0 
		.I1 (input2_and[6]),    // data input 1 
		.I2 (input3_and[6]),    // data input 2 
		.I3 (input4_and[6])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and7 ( 
		.O (out_and[7]),    //output 
		.I0 (input1_and[7]),    // data input 0 
		.I1 (input2_and[7]),    // data input 1 
		.I2 (input3_and[7]),    // data input 2 
		.I3 (input4_and[7])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and8 ( 
		.O (out_and[8]),    //output 
		.I0 (input1_and[8]),    // data input 0 
		.I1 (input2_and[8]),    // data input 1 
		.I2 (input3_and[8]),    // data input 2 
		.I3 (input4_and[8])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and9 ( 
		.O (out_and[9]),    //output 
		.I0 (input1_and[9]),    // data input 0 
		.I1 (input2_and[9]),    // data input 1 
		.I2 (input3_and[9]),    // data input 2 
		.I3 (input4_and[9])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and10 ( 
		.O (out_and[10]),    //output 
		.I0 (input1_and[10]),    // data input 0 
		.I1 (input2_and[10]),    // data input 1 
		.I2 (input3_and[10]),    // data input 2 
		.I3 (input4_and[10])     // data input 3 
		);  

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and11 ( 
		.O (out_and[11]),    //output 
		.I0 (input1_and[11]),    // data input 0 
		.I1 (input2_and[11]),    // data input 1 
		.I2 (input3_and[11]),    // data input 2 
		.I3 (input4_and[11])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and12 ( 
		.O (out_and[12]),    //output 
		.I0 (input1_and[12]),    // data input 0 
		.I1 (input2_and[12]),    // data input 1 
		.I2 (input3_and[12]),    // data input 2 
		.I3 (input4_and[12])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and13 ( 
		.O (out_and[13]),    //output 
		.I0 (input1_and[13]),    // data input 0 
		.I1 (input2_and[13]),    // data input 1 
		.I2 (input3_and[13]),    // data input 2 
		.I3 (input4_and[13])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and14 ( 
		.O (out_and[14]),    //output 
		.I0 (input1_and[14]),    // data input 0 
		.I1 (input2_and[14]),    // data input 1 
		.I2 (input3_and[14]),    // data input 2 
		.I3 (input4_and[14])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and15 ( 
		.O (out_and[15]),    //output 
		.I0 (input1_and[15]),    // data input 0 
		.I1 (input2_and[15]),    // data input 1 
		.I2 (input3_and[15]),    // data input 2 
		.I3 (input4_and[15])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and16 ( 
		.O (out_and[16]),    //output 
		.I0 (input1_and[16]),    // data input 0 
		.I1 (input2_and[16]),    // data input 1 
		.I2 (input3_and[16]),    // data input 2 
		.I3 (input4_and[16])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and17 ( 
		.O (out_and[17]),    //output 
		.I0 (input1_and[17]),    // data input 0 
		.I1 (input2_and[17]),    // data input 1 
		.I2 (input3_and[17]),    // data input 2 
		.I3 (input4_and[17])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and18 ( 
		.O (out_and[18]),    //output 
		.I0 (input1_and[18]),    // data input 0 
		.I1 (input2_and[18]),    // data input 1 
		.I2 (input3_and[18]),    // data input 2 
		.I3 (input4_and[18])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and19 ( 
		.O (out_and[19]),    //output 
		.I0 (input1_and[19]),    // data input 0 
		.I1 (input2_and[19]),    // data input 1 
		.I2 (input3_and[19]),    // data input 2 
		.I3 (input4_and[19])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and20 ( 
		.O (out_and[20]),    //output 
		.I0 (input1_and[20]),    // data input 0 
		.I1 (input2_and[20]),    // data input 1 
		.I2 (input3_and[20]),    // data input 2 
		.I3 (input4_and[20])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and21 ( 
		.O (out_and[21]),    //output 
		.I0 (input1_and[21]),    // data input 0 
		.I1 (input2_and[21]),    // data input 1 
		.I2 (input3_and[21]),    // data input 2 
		.I3 (input4_and[21])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and22 ( 
		.O (out_and[22]),    //output 
		.I0 (input1_and[22]),    // data input 0 
		.I1 (input2_and[22]),    // data input 1 
		.I2 (input3_and[22]),    // data input 2 
		.I3 (input4_and[22])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and23 ( 
		.O (out_and[23]),    //output 
		.I0 (input1_and[23]),    // data input 0 
		.I1 (input2_and[23]),    // data input 1 
		.I2 (input3_and[23]),    // data input 2 
		.I3 (input4_and[23])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and24 ( 
		.O (out_and[24]),    //output 
		.I0 (input1_and[24]),    // data input 0 
		.I1 (input2_and[24]),    // data input 1 
		.I2 (input3_and[24]),    // data input 2 
		.I3 (input4_and[24])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and25 ( 
		.O (out_and[25]),    //output 
		.I0 (input1_and[25]),    // data input 0 
		.I1 (input2_and[25]),    // data input 1 
		.I2 (input3_and[25]),    // data input 2 
		.I3 (input4_and[25])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and26 ( 
		.O (out_and[26]),    //output 
		.I0 (input1_and[26]),    // data input 0 
		.I1 (input2_and[26]),    // data input 1 
		.I2 (input3_and[26]),    // data input 2 
		.I3 (input4_and[26])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and27 ( 
		.O (out_and[27]),    //output 
		.I0 (input1_and[27]),    // data input 0 
		.I1 (input2_and[27]),    // data input 1 
		.I2 (input3_and[27]),    // data input 2 
		.I3 (input4_and[27])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and28 ( 
		.O (out_and[28]),    //output 
		.I0 (input1_and[28]),    // data input 0 
		.I1 (input2_and[28]),    // data input 1 
		.I2 (input3_and[28]),    // data input 2 
		.I3 (input4_and[28])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and29 ( 
		.O (out_and[29]),    //output 
		.I0 (input1_and[29]),    // data input 0 
		.I1 (input2_and[29]),    // data input 1 
		.I2 (input3_and[29]),    // data input 2 
		.I3 (input4_and[29])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and30 ( 
		.O (out_and[30]),    //output 
		.I0 (input1_and[30]),    // data input 0 
		.I1 (input2_and[30]),    // data input 1 
		.I2 (input3_and[30]),    // data input 2 
		.I3 (input4_and[30])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and31 ( 
		.O (out_and[31]),    //output 
		.I0 (input1_and[31]),    // data input 0 
		.I1 (input2_and[31]),    // data input 1 
		.I2 (input3_and[31]),    // data input 2 
		.I3 (input4_and[31])     // data input 3 
		); 

endmodule