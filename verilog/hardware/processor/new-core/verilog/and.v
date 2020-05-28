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
    input [31:0]	input1,
	input [31:0]	input2,
    input [31:0]    input3,
    input [31:0]    input4,
	output [31:0]   out,
    genvar k
    );

    // SB_LUT4 : 4-input Look-Up Table  
    generate
        for (k = 0; k < n; k = k + 1)
        begin: and_logic

			SB_LUT4 #(.LUT_INIT(16'h0001)) SB_LUT4_and ( 
                .O (out[k]),    // output 
                .I0 (input1[k]),    // data input 0 
                .I1 (input2[k]),    // data input 1 
                .I2 (input3[k]),    // data input 2 
                .I3 (input4[k])     // data input 3 
                );   
            
        end
    endgenerate

endmodule