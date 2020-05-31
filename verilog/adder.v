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



/*
 *	Description:
 *
 *		This module implements an adder for use by the branch unit
 *		and program counter increment among other things.
 */



module adder(input1, input2, out, clk);
	input clk;
	input [31:0]	input1;
	input [31:0]	input2;
	output [31:0]	out;

	reg dsp_ce;
        reg [15:0] dsp_c;
        reg [15:0] dsp_a;
        reg [15:0] dsp_b;
        reg [15:0] dsp_d;

	reg dsp_addsubtop;
        reg dsp_addsubbot;

	wire [31:0] dsp_o;
        wire dsp_co;

	SB_MAC16 i_sbmac16_adder
                ( // port interfaces
                        .A(dsp_a),
                        .B(dsp_b),
                        .C(dsp_c),
                        .D(dsp_d),
                        .O(dsp_o),
                        .CLK(clk),
			.ADDSUBTOP(dsp_addsubtop),
                        .ADDSUBBOT(dsp_addsubbot),
		);

	defparam i_sbmac16_adder.C_REG = 1'b0;
        defparam i_sbmac16_adder.A_REG = 1'b0;
        defparam i_sbmac16_adder.B_REG = 1'b0;
        defparam i_sbmac16_adder.D_REG = 1'b0;

        defparam i_sbmac16_adder.TOPOUTPUT_SELECT = 2'b00;
        defparam i_sbmac16_adder.TOPADDSUB_LOWERINPUT = 2'b00;
        defparam i_sbmac16_adder.TOPADDSUB_UPPERINPUT = 1'b1;
        defparam i_sbmac16_adder.TOPADDSUB_CARRYSELECT = 2'b11;

        defparam i_sbmac16_adder.BOTOUTPUT_SELECT = 2'b00;
        defparam i_sbmac16_adder.BOTADDSUB_LOWERINPUT = 2'b00;
        defparam i_sbmac16_adder.BOTADDSUB_UPPERINPUT = 1'b1;
        defparam i_sbmac16_adder.BOTADDSUB_CARRYSELECT = 2'b00;

	assign dsp_c = input1[31:16];
        assign dsp_a = input2[31:16];
        assign dsp_d = input1[15:0];
        assign dsp_b = input2[15:0];
	assign dsp_addsubtop = 0;
	assign dsp_addsubbot = 0;


	assign		out = dsp_o;

/*	assign		out = input1 + input2;*/
endmodule
