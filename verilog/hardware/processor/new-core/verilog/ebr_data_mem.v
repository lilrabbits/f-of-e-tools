/*
	Authored 2018-2019, Ryan Voo.

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



//Data cache

module ebr_data_mem (clk, addr, write_data, memwrite, memread, sign_mask, read_data, led, clk_stall);
	input				clk;
	input [31:0]		addr;
	input [31:0]		write_data;
	input				memwrite;
	input				memread;
	input [3:0]			sign_mask;
	output reg [31:0]	read_data;
	output [7:0]		led;
	output reg			clk_stall;	//Sets the clock high

	/*
	 *	led register
	 */
	reg [31:0]		led_reg;

	/*
	 *	Line buffer
	 */
	reg [31:0]		word_buf;

	/*
	 *	Read buffer
	 */
	wire [31:0]		read_buf;

	/*
	 *	Buffer to identify read or write operation
	 */
	reg			memread_buf;
	reg			memwrite_buf;

	/*
	 *	Buffers to store write data
	 */
	reg [31:0]		write_data_buffer;

	/*
	 *	Buffer to store address
	 */
	reg [31:0]		addr_buf;

	/*
	 *	Sign_mask buffer
	 */
	reg [3:0]		sign_mask_buf;

	/*
	 *	Block memory registers
	 *
	 *	(Bad practice: The constant for the size should be a `define).
	 */
	// Been replaced
	// reg [31:0]		data_block[0:1023];

	/*
	 *	wire assignments
	 */
	wire [9:0]		addr_block_addr;
	wire [1:0]		addr_byte_offset;
	
	wire [31:0]		replacement_word;

	assign			addr_block_addr	= addr[11:2];
	assign			addr_byte_offset	= addr[1:0];

	/*
	 *	Regs for multiplexer output
	 */
	wire [7:0]		buf0;
	wire [7:0]		buf1;
	wire [7:0]		buf2;
	wire [7:0]		buf3;

	assign 			buf0	= word[7:0];
	assign 			buf1	= word[15:8];
	assign 			buf2	= word[23:16];
	assign 			buf3	= word[31:24];

	/*
	 *	Byte select decoder
	 */
	wire bdec_sig0;
	wire bdec_sig1;
	wire bdec_sig2;
	wire bdec_sig3;

	assign bdec_sig0 = (~addr_byte_offset[1]) & (~addr_byte_offset[0]);
	assign bdec_sig1 = (~addr_byte_offset[1]) & (addr_byte_offset[0]);
	assign bdec_sig2 = (addr_byte_offset[1]) & (~addr_byte_offset[0]);
	assign bdec_sig3 = (addr_byte_offset[1]) & (addr_byte_offset[0]);


	/*
	 *	Constructing the word to be replaced for write byte
	 */
	wire[7:0] byte_r0;
	wire[7:0] byte_r1;
	wire[7:0] byte_r2;
	wire[7:0] byte_r3;
 
	assign byte_r0 = (bdec_sig0==1'b1) ? write_data_buffer[7:0] : buf0;
	assign byte_r1 = (bdec_sig1==1'b1) ? write_data_buffer[7:0] : buf1;
	assign byte_r2 = (bdec_sig2==1'b1) ? write_data_buffer[7:0] : buf2;
	assign byte_r3 = (bdec_sig3==1'b1) ? write_data_buffer[7:0] : buf3;

	/*
	 *	For write halfword
	 */
	wire[15:0] halfword_r0;
	wire[15:0] halfword_r1;

	assign halfword_r0 = (addr_buf_byte_offset[1]==1'b1) ? {buf1, buf0} : write_data_buffer[15:0];
	assign halfword_r1 = (addr_buf_byte_offset[1]==1'b1) ? write_data_buffer[15:0] : {buf3, buf2};

	/* a is sign_mask_buf[2], b is sign_mask_buf[1], c is sign_mask_buf[0] */
	wire write_select0;
	wire write_select1;
	
	wire[31:0] write_out1;
	wire[31:0] write_out2;
	
	assign write_select0 = ~sign_mask_buf[2] & sign_mask_buf[1];
	assign write_select1 = sign_mask_buf[2];
	
	assign write_out1 = (write_select0) ? {halfword_r1, halfword_r0} : {byte_r3, byte_r2, byte_r1, byte_r0};
	assign write_out2 = (write_select0) ? 32'b0 : write_data_buffer;
	
	assign replacement_word = (write_select1) ? write_out2 : write_out1;
	/*
	 *	Combinational logic for generating 32-bit read data
	 */
	
	wire select0;
	wire select1;
	wire select2;
	
	wire[31:0] out1;
	wire[31:0] out2;
	wire[31:0] out3;
	wire[31:0] out4;
	wire[31:0] out5;
	wire[31:0] out6;
	/* a is sign_mask_buf[2], b is sign_mask_buf[1], c is sign_mask_buf[0]
	 * d is addr_buf_byte_offset[1], e is addr_buf_byte_offset[0]
	 */
	
	assign select0 = (~sign_mask_buf[2] & ~sign_mask_buf[1] & ~addr_buf_byte_offset[1] & addr_buf_byte_offset[0]) | (~sign_mask_buf[2] & addr_buf_byte_offset[1] & addr_buf_byte_offset[0]) | (~sign_mask_buf[2] & sign_mask_buf[1] & addr_buf_byte_offset[1]); //~a~b~de + ~ade + ~abd
	assign select1 = (~sign_mask_buf[2] & ~sign_mask_buf[1] & addr_buf_byte_offset[1]) | (sign_mask_buf[2] & sign_mask_buf[1]); // ~a~bd + ab
	assign select2 = sign_mask_buf[1]; //b
	
	assign out1 = (select0) ? ((sign_mask_buf[3]==1'b1) ? {{24{buf1[7]}}, buf1} : {24'b0, buf1}) : ((sign_mask_buf[3]==1'b1) ? {{24{buf0[7]}}, buf0} : {24'b0, buf0});
	assign out2 = (select0) ? ((sign_mask_buf[3]==1'b1) ? {{24{buf3[7]}}, buf3} : {24'b0, buf3}) : ((sign_mask_buf[3]==1'b1) ? {{24{buf2[7]}}, buf2} : {24'b0, buf2}); 
	assign out3 = (select0) ? ((sign_mask_buf[3]==1'b1) ? {{16{buf3[7]}}, buf3, buf2} : {16'b0, buf3, buf2}) : ((sign_mask_buf[3]==1'b1) ? {{16{buf1[7]}}, buf1, buf0} : {16'b0, buf1, buf0});
	assign out4 = (select0) ? 32'b0 : {buf3, buf2, buf1, buf0};
	
	assign out5 = (select1) ? out2 : out1;
	assign out6 = (select1) ? out4 : out3;
	
	assign read_buf = (select2) ? out6 : out5;
	
    ebr_mem data_block (
        .clk(clk),
	    .addr(addr),
        .write_data(replacement_word),
        .memwrite(memwrite),
        .memread(memread),
        .read_data(word_buf),
    );

	/*
	 *	This uses Yosys's support for nonzero initial values:
	 *
	 *		https://github.com/YosysHQ/yosys/commit/0793f1b196df536975a044a4ce53025c81d00c7f
	 *
	 *	Rather than using this simulation construct (`initial`),
	 *	the design should instead use a reset signal going to
	 *	modules in the design.
	 */
    
    // TODO: Will need to replace this
	// initial begin
	// 	$readmemh("verilog/data.hex", data_block);
	// 	clk_stall = 0;
	// end

	/*
	 *	LED register interfacing with I/O
	 */
	always @(posedge clk) begin
		if(memwrite == 1'b1 && addr == 32'h2000) begin
			led_reg <= write_data;
		end
	end

	/*
	 *	Test led
	 */
	assign led = led_reg[7:0];
endmodule

module ebr_mem (clk, addr, write_data, memwrite, memread, read_data);
	input				clk;
	input [31:0]		addr;
	input [31:0]		write_data;
	input				memwrite;
	input				memread;
	output [31:0]	    read_data;

    genvar i;
	generate
		for (i = 0; i < 4; i = i + 1)
		begin: ram40_4k_gen_label
			SB_RAM40_4K #(
                .READ_MODE(0), 
                .WRITE_MODE(0)
            ) 
            ram_256x16_bot_i (
				.RDATA(read_data[31:16]),
				.RADDR({3'b0, addr[7:0]}),
				.RCLK(clk),
				.RCLKE(1'b1),
				.RE(memread & addr[9:8] == i),
				.WADDR({3'b0, addr[7:0]}),
				.WCLK(clk),
				.WCLKE(1'b1),
				.WDATA(write_data[31:16]),
				.WE(memwrite & addr[9:8] == i),
				.MASK(16'b0)
			);

			SB_RAM40_4K #(
                .READ_MODE(0), 
                .WRITE_MODE(0)
            ) 
            ram_256x16_top_i (
				.RDATA(read_data[15:0]),
				.RADDR({3'b0, addr[7:0]}),
				.RCLK(clk),
				.RCLKE(1'b1),
				.RE(memread & addr[9:8] == i),
				.WADDR({3'b0, addr[7:0]}),
				.WCLK(clk),
				.WCLKE(1'b1),
				.WDATA(write_data[15:0]),
				.WE(memwrite & addr[9:8] == i),
				.MASK(16'b0)
			);
		end
	endgenerate
			
endmodule