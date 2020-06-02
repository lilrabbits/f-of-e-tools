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
 *		This module implements an XNOR gate for use by the branch unit
 *		and program counter increment among other things.
 */

module xor_gate #(parameter n = 32) (
    input [31:0]	input1_xor, input2_xor, input3_xor, input4_xor,
	output [31:0]   out_xor
    );

    // SB_LUT4 : 4-input Look-Up Table  

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor0 ( 
		.O (out_xor[0]),    // output 
		.I0 (input1_xor[0]),    // data input 0 
		.I1 (input2_xor[0]),    // data input 1 
		.I2 (input3_xor[0]),    // data input 2 
		.I3 (input4_xor[0])     // data input 3 
		);  

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor1 ( 
		.O (out_xor[1]),    // output 
		.I0 (input1_xor[1]),    // data input 0 
		.I1 (input2_xor[1]),    // data input 1 
		.I2 (input3_xor[1]),    // data input 2 
		.I3 (input4_xor[1])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor2 ( 
		.O (out_xor[2]),    // output 
		.I0 (input1_xor[2]),    // data input 0 
		.I1 (input2_xor[2]),    // data input 1 
		.I2 (input3_xor[2]),    // data input 2 
		.I3 (input4_xor[2])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor3 ( 
		.O (out_xor[3]),    // output 
		.I0 (input1_xor[3]),    // data input 0 
		.I1 (input2_xor[3]),    // data input 1 
		.I2 (input3_xor[3]),    // data input 2 
		.I3 (input4_xor[3])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor4 ( 
		.O (out_xor[4]),    // output 
		.I0 (input1_xor[4]),    // data input 0 
		.I1 (input2_xor[4]),    // data input 1 
		.I2 (input3_xor[4]),    // data input 2 
		.I3 (input4_xor[4])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor5 ( 
		.O (out_xor[5]),    // output 
		.I0 (input1_xor[5]),    // data input 0 
		.I1 (input2_xor[5]),    // data input 1 
		.I2 (input3_xor[5]),    // data input 2 
		.I3 (input4_xor[5])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor6 ( 
		.O (out_xor[6]),    // output 
		.I0 (input1_xor[6]),    // data input 0 
		.I1 (input2_xor[6]),    // data input 1 
		.I2 (input3_xor[6]),    // data input 2 
		.I3 (input4_xor[6])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor7 ( 
		.O (out_xor[7]),    // output 
		.I0 (input1_xor[7]),    // data input 0 
		.I1 (input2_xor[7]),    // data input 1 
		.I2 (input3_xor[7]),    // data input 2 
		.I3 (input4_xor[7])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor8 ( 
		.O (out_xor[8]),    // output 
		.I0 (input1_xor[8]),    // data input 0 
		.I1 (input2_xor[8]),    // data input 1 
		.I2 (input3_xor[8]),    // data input 2 
		.I3 (input4_xor[8])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor9 ( 
		.O (out_xor[9]),    // output 
		.I0 (input1_xor[9]),    // data input 0 
		.I1 (input2_xor[9]),    // data input 1 
		.I2 (input3_xor[9]),    // data input 2 
		.I3 (input4_xor[9])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor10 ( 
		.O (out_xor[10]),    // output 
		.I0 (input1_xor[10]),    // data input 0 
		.I1 (input2_xor[10]),    // data input 1 
		.I2 (input3_xor[10]),    // data input 2 
		.I3 (input4_xor[10])     // data input 3 
		);  

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor11 ( 
		.O (out_xor[11]),    // output 
		.I0 (input1_xor[11]),    // data input 0 
		.I1 (input2_xor[11]),    // data input 1 
		.I2 (input3_xor[11]),    // data input 2 
		.I3 (input4_xor[11])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor12 ( 
		.O (out_xor[12]),    // output 
		.I0 (input1_xor[12]),    // data input 0 
		.I1 (input2_xor[12]),    // data input 1 
		.I2 (input3_xor[12]),    // data input 2 
		.I3 (input4_xor[12])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor13 ( 
		.O (out_xor[13]),    // output 
		.I0 (input1_xor[13]),    // data input 0 
		.I1 (input2_xor[13]),    // data input 1 
		.I2 (input3_xor[13]),    // data input 2 
		.I3 (input4_xor[13])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor14 ( 
		.O (out_xor[14]),    // output 
		.I0 (input1_xor[14]),    // data input 0 
		.I1 (input2_xor[14]),    // data input 1 
		.I2 (input3_xor[14]),    // data input 2 
		.I3 (input4_xor[14])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor15 ( 
		.O (out_xor[15]),    // output 
		.I0 (input1_xor[15]),    // data input 0 
		.I1 (input2_xor[15]),    // data input 1 
		.I2 (input3_xor[15]),    // data input 2 
		.I3 (input4_xor[15])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor16 ( 
		.O (out_xor[16]),    // output 
		.I0 (input1_xor[16]),    // data input 0 
		.I1 (input2_xor[16]),    // data input 1 
		.I2 (input3_xor[16]),    // data input 2 
		.I3 (input4_xor[16])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor17 ( 
		.O (out_xor[17]),    // output 
		.I0 (input1_xor[17]),    // data input 0 
		.I1 (input2_xor[17]),    // data input 1 
		.I2 (input3_xor[17]),    // data input 2 
		.I3 (input4_xor[17])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor18 ( 
		.O (out_xor[18]),    // output 
		.I0 (input1_xor[18]),    // data input 0 
		.I1 (input2_xor[18]),    // data input 1 
		.I2 (input3_xor[18]),    // data input 2 
		.I3 (input4_xor[18])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor19 ( 
		.O (out_xor[19]),    // output 
		.I0 (input1_xor[19]),    // data input 0 
		.I1 (input2_xor[19]),    // data input 1 
		.I2 (input3_xor[19]),    // data input 2 
		.I3 (input4_xor[19])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor20 ( 
		.O (out_xor[20]),    // output 
		.I0 (input1_xor[20]),    // data input 0 
		.I1 (input2_xor[20]),    // data input 1 
		.I2 (input3_xor[20]),    // data input 2 
		.I3 (input4_xor[20])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor21 ( 
		.O (out_xor[21]),    // output 
		.I0 (input1_xor[21]),    // data input 0 
		.I1 (input2_xor[21]),    // data input 1 
		.I2 (input3_xor[21]),    // data input 2 
		.I3 (input4_xor[21])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor22 ( 
		.O (out_xor[22]),    // output 
		.I0 (input1_xor[22]),    // data input 0 
		.I1 (input2_xor[22]),    // data input 1 
		.I2 (input3_xor[22]),    // data input 2 
		.I3 (input4_xor[22])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor23 ( 
		.O (out_xor[23]),    // output 
		.I0 (input1_xor[23]),    // data input 0 
		.I1 (input2_xor[23]),    // data input 1 
		.I2 (input3_xor[23]),    // data input 2 
		.I3 (input4_xor[23])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor24 ( 
		.O (out_xor[24]),    // output 
		.I0 (input1_xor[24]),    // data input 0 
		.I1 (input2_xor[24]),    // data input 1 
		.I2 (input3_xor[24]),    // data input 2 
		.I3 (input4_xor[24])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor25 ( 
		.O (out_xor[25]),    // output 
		.I0 (input1_xor[25]),    // data input 0 
		.I1 (input2_xor[25]),    // data input 1 
		.I2 (input3_xor[25]),    // data input 2 
		.I3 (input4_xor[25])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor26 ( 
		.O (out_xor[26]),    // output 
		.I0 (input1_xor[26]),    // data input 0 
		.I1 (input2_xor[26]),    // data input 1 
		.I2 (input3_xor[26]),    // data input 2 
		.I3 (input4_xor[26])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor27 ( 
		.O (out_xor[27]),    // output 
		.I0 (input1_xor[27]),    // data input 0 
		.I1 (input2_xor[27]),    // data input 1 
		.I2 (input3_xor[27]),    // data input 2 
		.I3 (input4_xor[27])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor28 ( 
		.O (out_xor[28]),    // output 
		.I0 (input1_xor[28]),    // data input 0 
		.I1 (input2_xor[28]),    // data input 1 
		.I2 (input3_xor[28]),    // data input 2 
		.I3 (input4_xor[28])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor29 ( 
		.O (out_xor[29]),    // output 
		.I0 (input1_xor[29]),    // data input 0 
		.I1 (input2_xor[29]),    // data input 1 
		.I2 (input3_xor[29]),    // data input 2 
		.I3 (input4_xor[29])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor30 ( 
		.O (out_xor[30]),    // output 
		.I0 (input1_xor[30]),    // data input 0 
		.I1 (input2_xor[30]),    // data input 1 
		.I2 (input3_xor[30]),    // data input 2 
		.I3 (input4_xor[30])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h6996)) SB_LUT4_xor31 ( 
		.O (out_xor[31]),    // output 
		.I0 (input1_xor[31]),    // data input 0 
		.I1 (input2_xor[31]),    // data input 1 
		.I2 (input3_xor[31]),    // data input 2 
		.I3 (input4_xor[31])     // data input 3 
		); 

endmodule