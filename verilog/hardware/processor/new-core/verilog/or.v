/*
	Authored 2019-2020, Rae Zhao.

	All rights reserved.
	Redistribution or use in source or binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions or the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions or the following
		disclaimer in the documentation or/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS or CONTRIBUTORS
	"AS IS" or ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY or FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED or ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY out OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/



/*
 *	Description:
 *
 *		This module implements an OR gate for use by the branch unit
 *		or program counter increment among other things.
 */

module or_gate #(parameter n = 32) (
    input [31:0]	input1_or, input2_or, input3_or, input4_or,
	output [31:0]   out_or
    );

    // SB_LUT4 : 4-input Look-Up Table  
	
	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or0 ( 
		.O (out_or[0]),    // output 
		.I0 (input1_or[0]),    // data input 0 
		.I1 (input2_or[0]),    // data input 1 
		.I2 (input3_or[0]),    // data input 2 
		.I3 (input4_or[0])     // data input 3 
		);  

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or1 ( 
		.O (out_or[1]),    //output
		.I0 (input1_or[1]),    // data input 0 
		.I1 (input2_or[1]),    // data input 1 
		.I2 (input3_or[1]),    // data input 2 
		.I3 (input4_or[1])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or2 ( 
		.O (out_or[2]),    //output
		.I0 (input1_or[2]),    // data input 0 
		.I1 (input2_or[2]),    // data input 1 
		.I2 (input3_or[2]),    // data input 2 
		.I3 (input4_or[2])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or3 ( 
		.O (out_or[3]),    //output
		.I0 (input1_or[3]),    // data input 0 
		.I1 (input2_or[3]),    // data input 1 
		.I2 (input3_or[3]),    // data input 2 
		.I3 (input4_or[3])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or4 ( 
		.O (out_or[4]),    //output
		.I0 (input1_or[4]),    // data input 0 
		.I1 (input2_or[4]),    // data input 1 
		.I2 (input3_or[4]),    // data input 2 
		.I3 (input4_or[4])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or5 ( 
		.O (out_or[5]),    //output
		.I0 (input1_or[5]),    // data input 0 
		.I1 (input2_or[5]),    // data input 1 
		.I2 (input3_or[5]),    // data input 2 
		.I3 (input4_or[5])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or6 ( 
		.O (out_or[6]),    //output
		.I0 (input1_or[6]),    // data input 0 
		.I1 (input2_or[6]),    // data input 1 
		.I2 (input3_or[6]),    // data input 2 
		.I3 (input4_or[6])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or7 ( 
		.O (out_or[7]),    //output
		.I0 (input1_or[7]),    // data input 0 
		.I1 (input2_or[7]),    // data input 1 
		.I2 (input3_or[7]),    // data input 2 
		.I3 (input4_or[7])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or8 ( 
		.O (out_or[8]),    //output
		.I0 (input1_or[8]),    // data input 0 
		.I1 (input2_or[8]),    // data input 1 
		.I2 (input3_or[8]),    // data input 2 
		.I3 (input4_or[8])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or9 ( 
		.O (out_or[9]),    //output
		.I0 (input1_or[9]),    // data input 0 
		.I1 (input2_or[9]),    // data input 1 
		.I2 (input3_or[9]),    // data input 2 
		.I3 (input4_or[9])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or10 ( 
		.O (out_or[10]),    //output
		.I0 (input1_or[10]),    // data input 0 
		.I1 (input2_or[10]),    // data input 1 
		.I2 (input3_or[10]),    // data input 2 
		.I3 (input4_or[10])     // data input 3 
		);  

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or11 ( 
		.O (out_or[11]),    //output
		.I0 (input1_or[11]),    // data input 0 
		.I1 (input2_or[11]),    // data input 1 
		.I2 (input3_or[11]),    // data input 2 
		.I3 (input4_or[11])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or12 ( 
		.O (out_or[12]),    //output
		.I0 (input1_or[12]),    // data input 0 
		.I1 (input2_or[12]),    // data input 1 
		.I2 (input3_or[12]),    // data input 2 
		.I3 (input4_or[12])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or13 ( 
		.O (out_or[13]),    //output
		.I0 (input1_or[13]),    // data input 0 
		.I1 (input2_or[13]),    // data input 1 
		.I2 (input3_or[13]),    // data input 2 
		.I3 (input4_or[13])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or14 ( 
		.O (out_or[14]),    //output
		.I0 (input1_or[14]),    // data input 0 
		.I1 (input2_or[14]),    // data input 1 
		.I2 (input3_or[14]),    // data input 2 
		.I3 (input4_or[14])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or15 ( 
		.O (out_or[15]),    //output
		.I0 (input1_or[15]),    // data input 0 
		.I1 (input2_or[15]),    // data input 1 
		.I2 (input3_or[15]),    // data input 2 
		.I3 (input4_or[15])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or16 ( 
		.O (out_or[16]),    //output
		.I0 (input1_or[16]),    // data input 0 
		.I1 (input2_or[16]),    // data input 1 
		.I2 (input3_or[16]),    // data input 2 
		.I3 (input4_or[16])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or17 ( 
		.O (out_or[17]),    //output
		.I0 (input1_or[17]),    // data input 0 
		.I1 (input2_or[17]),    // data input 1 
		.I2 (input3_or[17]),    // data input 2 
		.I3 (input4_or[17])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or18 ( 
		.O (out_or[18]),    //output
		.I0 (input1_or[18]),    // data input 0 
		.I1 (input2_or[18]),    // data input 1 
		.I2 (input3_or[18]),    // data input 2 
		.I3 (input4_or[18])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or19 ( 
		.O (out_or[19]),    //output
		.I0 (input1_or[19]),    // data input 0 
		.I1 (input2_or[19]),    // data input 1 
		.I2 (input3_or[19]),    // data input 2 
		.I3 (input4_or[19])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or20 ( 
		.O (out_or[20]),    //output
		.I0 (input1_or[20]),    // data input 0 
		.I1 (input2_or[20]),    // data input 1 
		.I2 (input3_or[20]),    // data input 2 
		.I3 (input4_or[20])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or21 ( 
		.O (out_or[21]),    //output
		.I0 (input1_or[21]),    // data input 0 
		.I1 (input2_or[21]),    // data input 1 
		.I2 (input3_or[21]),    // data input 2 
		.I3 (input4_or[21])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or22 ( 
		.O (out_or[22]),    //output
		.I0 (input1_or[22]),    // data input 0 
		.I1 (input2_or[22]),    // data input 1 
		.I2 (input3_or[22]),    // data input 2 
		.I3 (input4_or[22])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or23 ( 
		.O (out_or[23]),    //output
		.I0 (input1_or[23]),    // data input 0 
		.I1 (input2_or[23]),    // data input 1 
		.I2 (input3_or[23]),    // data input 2 
		.I3 (input4_or[23])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or24 ( 
		.O (out_or[24]),    //output
		.I0 (input1_or[24]),    // data input 0 
		.I1 (input2_or[24]),    // data input 1 
		.I2 (input3_or[24]),    // data input 2 
		.I3 (input4_or[24])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or25 ( 
		.O (out_or[25]),    //output
		.I0 (input1_or[25]),    // data input 0 
		.I1 (input2_or[25]),    // data input 1 
		.I2 (input3_or[25]),    // data input 2 
		.I3 (input4_or[25])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or26 ( 
		.O (out_or[26]),    //output
		.I0 (input1_or[26]),    // data input 0 
		.I1 (input2_or[26]),    // data input 1 
		.I2 (input3_or[26]),    // data input 2 
		.I3 (input4_or[26])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or27 ( 
		.O (out_or[27]),    //output
		.I0 (input1_or[27]),    // data input 0 
		.I1 (input2_or[27]),    // data input 1 
		.I2 (input3_or[27]),    // data input 2 
		.I3 (input4_or[27])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or28 ( 
		.O (out_or[28]),    //output
		.I0 (input1_or[28]),    // data input 0 
		.I1 (input2_or[28]),    // data input 1 
		.I2 (input3_or[28]),    // data input 2 
		.I3 (input4_or[28])     // data input 3 
		); 
            
	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or29 ( 
		.O (out_or[29]),    //output
		.I0 (input1_or[29]),    // data input 0 
		.I1 (input2_or[29]),    // data input 1 
		.I2 (input3_or[29]),    // data input 2 
		.I3 (input4_or[29])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or30 ( 
		.O (out_or[30]),    //output
		.I0 (input1_or[30]),    // data input 0 
		.I1 (input2_or[30]),    // data input 1 
		.I2 (input3_or[30]),    // data input 2 
		.I3 (input4_or[30])     // data input 3 
		); 

	SB_LUT4 #(.LUT_INIT(16'h7FFF)) SB_LUT4_or31 ( 
		.O (out_or[31]),    //output
		.I0 (input1_or[31]),    // data input 0 
		.I1 (input2_or[31]),    // data input 1 
		.I2 (input3_or[31]),    // data input 2 
		.I3 (input4_or[31])     // data input 3 
		); 
          
endmodule