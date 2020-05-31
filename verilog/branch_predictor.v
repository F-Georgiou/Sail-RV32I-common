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
 *		Branch Predictor FSM
 */

module branch_predictor(
		clk,
		actual_branch_decision,
		branch_decode_sig,
		branch_mem_sig,
		in_addr,
		offset,
		branch_addr,
		prediction
	);

	/*
	 *	inputs
	 */
	input		clk;
	input		actual_branch_decision;
	input		branch_decode_sig;
	input		branch_mem_sig;
	input [31:0]	in_addr;
	input [31:0]	offset;

	/*
	 *	outputs
	 */
	output [31:0]	branch_addr;
	output		prediction;

	/*
	 *	internal state
	 */
	reg [1:0]	s;

	reg		branch_mem_sig_reg;

	/*
	 *	DSP for addition
	 */ 

	reg dsp_ce;
        reg [15:0] dsp_c;
        reg [15:0] dsp_a;
        reg [15:0] dsp_b;
        reg [15:0] dsp_d;

        reg dsp_addsubtop;
        reg dsp_addsubbot;

        wire [31:0] dsp_o;
        wire dsp_co;

        SB_MAC16 i_sbmac16_branch
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

        defparam i_sbmac16_branch.C_REG = 1'b0;
        defparam i_sbmac16_branch.A_REG = 1'b0;
        defparam i_sbmac16_branch.B_REG = 1'b0;
        defparam i_sbmac16_branch.D_REG = 1'b0;

        defparam i_sbmac16_branch.TOPOUTPUT_SELECT = 2'b00;
        defparam i_sbmac16_branch.TOPADDSUB_LOWERINPUT = 2'b00;
        defparam i_sbmac16_branch.TOPADDSUB_UPPERINPUT = 1'b1;
        defparam i_sbmac16_branch.TOPADDSUB_CARRYSELECT = 2'b11;

        defparam i_sbmac16_branch.BOTOUTPUT_SELECT = 2'b00;
        defparam i_sbmac16_branch.BOTADDSUB_LOWERINPUT = 2'b00;
        defparam i_sbmac16_branch.BOTADDSUB_UPPERINPUT = 1'b1;
        defparam i_sbmac16_branch.BOTADDSUB_CARRYSELECT = 2'b00;

	/*
	 *	The `initial` statement below uses Yosys's support for nonzero
	 *	initial values:
	 *
	 *		https://github.com/YosysHQ/yosys/commit/0793f1b196df536975a044a4ce53025c81d00c7f
	 *
	 *	Rather than using this simulation construct (`initial`),
	 *	the design should instead use a reset signal going to
	 *	modules in the design and to thereby set the values.
	 */
	initial begin
		s = 2'b00;
		branch_mem_sig_reg = 1'b0;
	end

	always @(negedge clk) begin
		branch_mem_sig_reg <= branch_mem_sig;
	end

	/*
	 *	Using this microarchitecture, branches can't occur consecutively
	 *	therefore can use branch_mem_sig as every branch is followed by
	 *	a bubble, so a 0 to 1 transition
	 */
	always @(posedge clk) begin
		if (branch_mem_sig_reg) begin
			s[1] <= (s[1]&s[0]) | (s[0]&actual_branch_decision) | (s[1]&actual_branch_decision);
			s[0] <= (s[1]&(!s[0])) | ((!s[0])&actual_branch_decision) | (s[1]&actual_branch_decision);
		end
	end

	assign dsp_c = in_addr[31:16];
        assign dsp_a = offset[31:16];
        assign dsp_d = in_addr[15:0];
        assign dsp_b = offset[15:0];
        assign dsp_addsubtop = 0;
        assign dsp_addsubbot = 0;

	assign branch_addr = dsp_o;
	//assign branch_addr = in_addr + offset;
	assign prediction = s[1] & branch_decode_sig;
endmodule
