`default_nettype wire

/*
    Out-of-context top for the combinational AGU.

    The flat-port OOC wrapper generator needs a clock port on the top to build a
    registered timing boundary, but the AGU is purely combinational. So i_CLK_p
    exists only to clock the auto-generated boundary flops; it is not used inside
    the DUT. A single 200 MHz clock (= 5 ns) is exactly the cen_p->cen_n half-clock
    budget we are checking the AGU against: meeting timing here means the AGU fits
    one half-architectural-clock, and the netlist tells us the logic-cell depth.
*/

module agu_ooc_top (
    input   wire            i_CLK_p,        //OOC boundary-register clock only (DUT is combinational)
    input   wire    [31:0]  i_AGU_A,
    input   wire    [31:0]  i_AGU_B,
    input   wire    [31:0]  i_FETCH_PC,
    input   wire            i_USE_BASE,
    input   wire    [1:0]   i_EN_MODE,
    input   wire            i_R_T,
    input   wire            i_PC_INC,
    output  wire    [31:0]  o_ADDR
);

agu u_agu (
    .i_AGU_A     (i_AGU_A    ),
    .i_AGU_B     (i_AGU_B    ),
    .i_FETCH_PC  (i_FETCH_PC ),
    .i_USE_BASE  (i_USE_BASE ),
    .i_EN_MODE   (i_EN_MODE  ),
    .i_R_T       (i_R_T      ),
    .i_PC_INC    (i_PC_INC   ),
    .o_ADDR      (o_ADDR     )
);

//i_CLK_p drives only the OOC wrapper's boundary flops; sink it so lint stays quiet
wire    unused_clk = i_CLK_p;

endmodule

`default_nettype none
