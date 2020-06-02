// Custom frequency generator sample file generated by iCEcube2

// Frequency formula for SIMPLE feedback mode
// FOUT = FREF * (DIVF + 1)
//        -------------------
//        2^DIVQ * (DIVR + 1)

// DIVR - 16 bit
// DIVF - 64 bit
// DIVQ - 8 bit

module clock_divider(REFERENCECLK,
                      PLLOUTCORE,
                      PLLOUTGLOBAL,
                      RESET);

	input REFERENCECLK;
	input RESET;    /* To initialize the simulation properly, the RESET signal (Active Low) must be asserted at the beginning of the simulation */ 
	output PLLOUTCORE;
	output PLLOUTGLOBAL;

	SB_PLL40_CORE clock_divider_inst(.REFERENCECLK(REFERENCECLK),
	                                  .PLLOUTCORE(PLLOUTCORE),
	                                  .PLLOUTGLOBAL(PLLOUTGLOBAL),
	                                  .EXTFEEDBACK(),
	                                  .DYNAMICDELAY(),
	                                  .RESETB(RESET),
	                                  .BYPASS(1'b0),
	                                  .LATCHINPUTVALUE(),
	                                  .LOCK(),
	                                  .SDI(),
	                                  .SDO(),
	                                  .SCLK());

	// Fin=24, Fout=19.5;
	defparam clock_divider_inst.DIVR = 4'd7;
	defparam clock_divider_inst.DIVF = 7'd12;
	defparam clock_divider_inst.DIVQ = 3'd1;
	defparam clock_divider_inst.FILTER_RANGE = 3'b001;
	defparam clock_divider_inst.FEEDBACK_PATH = "SIMPLE";
	defparam clock_divider_inst.DELAY_ADJUSTMENT_MODE_FEEDBACK = "FIXED";
	defparam clock_divider_inst.FDA_FEEDBACK = 4'b0000;
	defparam clock_divider_inst.DELAY_ADJUSTMENT_MODE_RELATIVE = "FIXED";
	defparam clock_divider_inst.FDA_RELATIVE = 4'b0000;
	defparam clock_divider_inst.SHIFTREG_DIV_MODE = 2'b00;
	defparam clock_divider_inst.PLLOUT_SELECT = "GENCLK";
	defparam clock_divider_inst.ENABLE_ICEGATE = 1'b0;
endmodule
