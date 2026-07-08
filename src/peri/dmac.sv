`default_nettype wire

/*
    Direct memory access controller DMAC (SH7709S section 11, pp.327-387).

    This block is the DMAC's register face + the on-chip compare match
    timer CMT (section 11.4 - a 16-bit up-counter whose compare match is
    a DMA request source; it has NO INTC line on the SH7709S). The
    transfer engine (request priority, start-up control, bus interface of
    Fig 11.1) arrives in later phases; per the manual the engine calls
    the BSC like a function - external bus cycles are shaped by the BSC
    "in the same way as when the CPU is the bus master" (p.363).

    Register window (tables 11.2 + 11.7): channel quads at 0x04000020 +
    0x10*n {SAR, DAR, DMATCR, CHCR}, DMAOR at 0x60, CMT at 0x70-76. The
    0x62-6F hole is never decoded (section 11.6 note 11) - the BSC's
    front-end excludes it. P2 aliases (0xA4000020+) land here through the
    BSC's shadow-invariant area-1 decode.

    Access laws: writes arrive right-justified from the P bridge and are
    lane-aligned here (big-endian, matching the chip top); a 16-bit
    access to a 32-bit register keeps the untouched half (p.332 note 2).
    TE/NMIF/AE/CMF are write-0-only flags (hardware set outranks a
    same-edge clear). Size legality (section 11.6 note 1: DMAOR 8/16,
    others 16/32) is NOT enforced - the TMU precedent.

    Reset: CHCR/DMAOR/CMT clear on power-on AND manual reset (p.332);
    SAR/DAR/DMATCR are architecturally undefined - reset-to-0 here.

    DEI0-3 transfer-end interrupt requests are LEVELS (TE AND IE),
    dropped by the handler's TE write-0 (INTC codes 0x800-0x860, IPRE).
*/

module dmac (
    /* CLOCK AND RESET - clears on any reset flavor (p.332) */
    input   wire            i_RST_n,
    input   wire            i_CLK,
    input   wire            i_CEN,
    input   wire            i_PCEN,         //P-phi enable (i_CEN-qualified) for the CMT prescaler

    /* INTERFACES */
    IBus_2.slave            REG_BUS,        //P bus window 0x04000020-77 (behind the BSC)

    /* INTERRUPT REQUESTS - levels, table 6.4 order */
    output  wire    [3:0]   o_DEI           //{DEI3, DEI2, DEI1, DEI0} = per-channel TE & IE
);

///////////////////////////////////////////////////////////
//////  Register Storage (shared: DMAOR + CMT)
////

logic   [1:0]   pr;                         //DMAOR[9:8]: channel priority mode (p.343)
logic           ae;                         //DMAOR[2]: address error flag, write-0-only
logic           nmif;                       //DMAOR[1]: NMI flag, write-0-only
logic           dme;                        //DMAOR[0]: DMA master enable
logic           cmstr_rsv;                  //CMSTR[1]: R/W spare, write 0 (p.377)
logic           cmstr_str0;                 //CMSTR[0]: CMCNT0 count start
logic           cmf;                        //CMCSR0[7]: compare match flag, write-0-only
logic           cmcsr_rsv;                  //CMCSR0[6]: R/W spare, write 0 (p.378)
logic   [1:0]   cks;                        //CMCSR0[1:0]: clock select P-phi/4/8/16/64
logic   [15:0]  cmcnt;                      //16-bit up-counter (p.379)
logic   [15:0]  cmcor;                      //compare match constant, resets H'FFFF (p.380)

wire    [15:0]  dmaor = {6'd0, pr, 5'd0, ae, nmif, dme};
wire    [15:0]  cmstr = {14'd0, cmstr_rsv, cmstr_str0};
wire    [15:0]  cmcsr = {8'd0, cmf, cmcsr_rsv, 4'd0, cks};



///////////////////////////////////////////////////////////
//////  Write Lane Alignment (big-endian)
////

//the P bridge right-justifies payloads; spread them onto the 32-bit register
//lanes so partial accesses mask cleanly: byte offset a lands in lane 3-a
logic   [31:0]  wd_lane;                    //write data replicated onto its lanes
logic   [3:0]   wm_lane;                    //byte-lane mask, [3] = bits 31:24
always_comb begin
    unique case(REG_BUS.size)
        2'd0: begin                         //byte
            wd_lane = {4{REG_BUS.wdata[7:0]}};
            wm_lane = 4'b1000 >> REG_BUS.addr[1:0];
        end
        2'd1: begin                         //word (16-bit)
            wd_lane = {2{REG_BUS.wdata[15:0]}};
            wm_lane = REG_BUS.addr[1] ? 4'b0011 : 4'b1100;
        end
        default: begin                      //longword
            wd_lane = REG_BUS.wdata;
            wm_lane = 4'b1111;
        end
    endcase
end



///////////////////////////////////////////////////////////
//////  Register Access Decode
////

wire            reg_wr = REG_BUS.stb && REG_BUS.we;

//channel quad select: 0x20 + 0x10*n, +0 SAR +4 DAR +8 DMATCR +C CHCR
logic   [3:0]   wr_sar, wr_dar, wr_tcr, wr_chcr;
always_comb begin
    for(int c = 0; c < 4; c++) begin
        logic sel;
        sel = reg_wr && (REG_BUS.addr[7:4] == 4'(c + 2));
        wr_sar[c]  = sel && (REG_BUS.addr[3:2] == 2'd0);
        wr_dar[c]  = sel && (REG_BUS.addr[3:2] == 2'd1);
        wr_tcr[c]  = sel && (REG_BUS.addr[3:2] == 2'd2);
        wr_chcr[c] = sel && (REG_BUS.addr[3:2] == 2'd3);
    end
end

wire            wr_dmaor = reg_wr && (REG_BUS.addr[7:2] == 6'b01_1000);     //0x60
wire            wr_cmt_a = reg_wr && (REG_BUS.addr[7:2] == 6'b01_1100);     //0x70: CMSTR+CMCSR0
wire            wr_cmt_b = reg_wr && (REG_BUS.addr[7:2] == 6'b01_1101);     //0x74: CMCNT0+CMCOR0



///////////////////////////////////////////////////////////
//////  Channel Register Quads (the Fig 11.1 x4 block)
////

wire    [31:0]  ch_sar  [0:3];
wire    [31:0]  ch_dar  [0:3];
wire    [23:0]  ch_tcr  [0:3];
wire    [31:0]  ch_chcr [0:3];

//feature asymmetry per pp.336-342: DREQ/DACK bits on ch0/1, reload on
//ch2, indirect on ch3; TE set hooks arrive with the transfer engine
dmac_channel #(.CH_ID(0), .HAS_EXT(1'b1)) u_ch0 (
    .i_RST_n(i_RST_n), .i_CLK(i_CLK), .i_CEN(i_CEN),
    .i_WR_SAR(wr_sar[0]), .i_WR_DAR(wr_dar[0]), .i_WR_TCR(wr_tcr[0]), .i_WR_CHCR(wr_chcr[0]),
    .i_WDATA(wd_lane), .i_WMASK(wm_lane),
    .i_TE_SET(1'b0),
    .o_SAR(ch_sar[0]), .o_DAR(ch_dar[0]), .o_TCR(ch_tcr[0]), .o_CHCR(ch_chcr[0])
);
dmac_channel #(.CH_ID(1), .HAS_EXT(1'b1)) u_ch1 (
    .i_RST_n(i_RST_n), .i_CLK(i_CLK), .i_CEN(i_CEN),
    .i_WR_SAR(wr_sar[1]), .i_WR_DAR(wr_dar[1]), .i_WR_TCR(wr_tcr[1]), .i_WR_CHCR(wr_chcr[1]),
    .i_WDATA(wd_lane), .i_WMASK(wm_lane),
    .i_TE_SET(1'b0),
    .o_SAR(ch_sar[1]), .o_DAR(ch_dar[1]), .o_TCR(ch_tcr[1]), .o_CHCR(ch_chcr[1])
);
dmac_channel #(.CH_ID(2), .HAS_RELOAD(1'b1)) u_ch2 (
    .i_RST_n(i_RST_n), .i_CLK(i_CLK), .i_CEN(i_CEN),
    .i_WR_SAR(wr_sar[2]), .i_WR_DAR(wr_dar[2]), .i_WR_TCR(wr_tcr[2]), .i_WR_CHCR(wr_chcr[2]),
    .i_WDATA(wd_lane), .i_WMASK(wm_lane),
    .i_TE_SET(1'b0),
    .o_SAR(ch_sar[2]), .o_DAR(ch_dar[2]), .o_TCR(ch_tcr[2]), .o_CHCR(ch_chcr[2])
);
dmac_channel #(.CH_ID(3), .HAS_INDIRECT(1'b1)) u_ch3 (
    .i_RST_n(i_RST_n), .i_CLK(i_CLK), .i_CEN(i_CEN),
    .i_WR_SAR(wr_sar[3]), .i_WR_DAR(wr_dar[3]), .i_WR_TCR(wr_tcr[3]), .i_WR_CHCR(wr_chcr[3]),
    .i_WDATA(wd_lane), .i_WMASK(wm_lane),
    .i_TE_SET(1'b0),
    .o_SAR(ch_sar[3]), .o_DAR(ch_dar[3]), .o_TCR(ch_tcr[3]), .o_CHCR(ch_chcr[3])
);



///////////////////////////////////////////////////////////
//////  DMAOR and CMT Registers
////

/*
    DMAOR lives in word-view lanes 3:2 (16-bit register at 0x60, big-
    endian): PR = view bits 25:24, AE/NMIF/DME = view bits 18:16. AE and
    NMIF are write-0-only; their set conditions (DMAC address error, NMI
    edge from the INTC) arrive with the transfer/abort phases.

    CMT (section 11.4): CMCNT0 counts up on the CKS-selected P-phi tap
    while STR0 = 1; at CMCNT0 == CMCOR0 the counter clears and CMF sets
    in the same tick (fig 11.27 - one request per period, no re-fire
    without a fresh input clock). A bus write to CMCNT0 wins over a
    same-edge tick; the CMF set outranks a same-edge write-0 clear
    (the TMU UNF pattern).
*/

//free-running P-phi prescaler; a tap fires when its divided count wraps
logic   [5:0]   psc;
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) psc <= 6'd0;
    else begin if(i_CEN) begin
        if(i_PCEN) psc <= psc + 6'd1;
    end end
end

logic           cmt_tick;                   //CKS-selected count enable (p.379)
always_comb begin
    unique case(cks)
        2'd0:    cmt_tick = i_PCEN & (psc[1:0] == 2'h3);    //P-phi/4
        2'd1:    cmt_tick = i_PCEN & (psc[2:0] == 3'h7);    //P-phi/8
        2'd2:    cmt_tick = i_PCEN & (psc[3:0] == 4'hF);    //P-phi/16
        default: cmt_tick = i_PCEN & (psc[5:0] == 6'h3F);   //P-phi/64
    endcase
end

wire            wr_cmcnt = wr_cmt_b && (wm_lane[3] || wm_lane[2]);  //CMCNT0 = view lanes 3:2
wire            cmt_match = cmstr_str0 && cmt_tick && (cmcnt == cmcor);

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        pr    <= 2'd0;
        ae    <= 1'b0;
        nmif  <= 1'b0;
        dme   <= 1'b0;
        cmstr_rsv  <= 1'b0;
        cmstr_str0 <= 1'b0;
        cmf   <= 1'b0;
        cmcsr_rsv  <= 1'b0;
        cks   <= 2'd0;
        cmcnt <= 16'd0;
        cmcor <= 16'hFFFF;
    end
    else begin if(i_CEN) begin
        if(wr_dmaor) begin
            if(wm_lane[3]) pr <= wd_lane[25:24];
            if(wm_lane[2]) begin
                dme  <= wd_lane[16];
                ae   <= ae   & wd_lane[18];     //write-0-only flags (p.343)
                nmif <= nmif & wd_lane[17];
            end
        end

        if(wr_cmt_a && wm_lane[2]) begin        //CMSTR = view lanes 3:2
            cmstr_rsv  <= wd_lane[17];
            cmstr_str0 <= wd_lane[16];
        end
        if(wr_cmt_a && wm_lane[0]) begin        //CMCSR0 = view lanes 1:0
            cmcsr_rsv <= wd_lane[6];
            cks       <= wd_lane[1:0];
        end

        if(wr_cmcnt) begin                      //bus write wins over the tick
            if(wm_lane[3]) cmcnt[15:8] <= wd_lane[31:24];
            if(wm_lane[2]) cmcnt[7:0]  <= wd_lane[23:16];
        end
        else if(cmstr_str0 && cmt_tick) begin
            if(cmcnt == cmcor) cmcnt <= 16'd0;  //match: clear and count on (fig 11.25)
            else               cmcnt <= cmcnt + 16'd1;
        end
        if(wr_cmt_b && wm_lane[1]) cmcor[15:8] <= wd_lane[15:8];    //CMCOR0 = view lanes 1:0
        if(wr_cmt_b && wm_lane[0]) cmcor[7:0]  <= wd_lane[7:0];

        //CMF: match tick sets (unless the same-edge write replaced the count),
        //else CMCSR0 write-0 clears (write-1 holds)
        if(cmt_match && !wr_cmcnt)         cmf <= 1'b1;
        else if(wr_cmt_a && wm_lane[0])    cmf <= cmf & wd_lane[7];
    end end
end



///////////////////////////////////////////////////////////
//////  Register Read Mux
////

//32-bit word view per addr[7:2]; 16-bit registers pack big-endian
//({reg @+0, reg @+2}); undecoded offsets read 0
logic   [31:0]  rword;
always_comb begin
    unique case(REG_BUS.addr[7:2])
        6'h08:   rword = ch_sar[0];                 //0x20
        6'h09:   rword = ch_dar[0];                 //0x24
        6'h0A:   rword = {8'd0, ch_tcr[0]};         //0x28
        6'h0B:   rword = ch_chcr[0];                //0x2C
        6'h0C:   rword = ch_sar[1];                 //0x30
        6'h0D:   rword = ch_dar[1];                 //0x34
        6'h0E:   rword = {8'd0, ch_tcr[1]};         //0x38
        6'h0F:   rword = ch_chcr[1];                //0x3C
        6'h10:   rword = ch_sar[2];                 //0x40
        6'h11:   rword = ch_dar[2];                 //0x44
        6'h12:   rword = {8'd0, ch_tcr[2]};         //0x48
        6'h13:   rword = ch_chcr[2];                //0x4C
        6'h14:   rword = ch_sar[3];                 //0x50
        6'h15:   rword = ch_dar[3];                 //0x54
        6'h16:   rword = {8'd0, ch_tcr[3]};         //0x58
        6'h17:   rword = ch_chcr[3];                //0x5C
        6'h18:   rword = {dmaor, 16'd0};            //0x60 (16-bit, upper lanes)
        6'h1C:   rword = {cmstr, cmcsr};            //0x70, 0x72
        6'h1D:   rword = {cmcnt, cmcor};            //0x74, 0x76
        default: rword = 32'd0;
    endcase
end

//right-justify the addressed lane for the bridge's size replication:
//(3 - a)*8 = {~a, 3'b000} for the big-endian byte pick
wire    [31:0]  rw_byte = rword >> {~REG_BUS.addr[1:0], 3'b000};
always_comb begin
    unique case(REG_BUS.size)
        2'd0:    REG_BUS.rdata = {24'd0, rw_byte[7:0]};
        2'd1:    REG_BUS.rdata = REG_BUS.addr[1] ? {16'd0, rword[15:0]}
                                                 : {16'd0, rword[31:16]};
        default: REG_BUS.rdata = rword;
    endcase
end



///////////////////////////////////////////////////////////
//////  Interrupt Requests
////

//levels: TE AND IE per channel (p.372); the handler's TE write-0 drops them
logic   [3:0]   dei;
always_comb begin
    for(int c = 0; c < 4; c++) dei[c] = ch_chcr[c][1] & ch_chcr[c][2];
end
assign  o_DEI = dei;

endmodule

`default_nettype none
