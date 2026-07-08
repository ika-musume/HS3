`default_nettype wire

/*
    Direct memory access controller DMAC (SH7709S section 11, pp.327-387).

    This block is the DMAC's register face, the on-chip compare match
    timer CMT (section 11.4 - a 16-bit up-counter whose compare match is
    a DMA request source; it has NO INTC line on the SH7709S), and the
    transfer engine (request priority + start-up control + bus interface
    of Fig 11.1). The engine masters IBus_1 through the on-chip arbiter
    and calls the BSC like a function - external bus cycles are shaped
    by the BSC "in the same way as when the CPU is the bus master"
    (p.363). Phase 3 scope: auto-request + CMT-request dual-direct
    transfers, byte/word/long, cycle-steal + burst, fixed priority.
    External DREQ/DACK (phase 4), 16-byte/reload/indirect/round-robin
    (phase 5), NMI/AE aborts (phase 6) follow.

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
    IBus_1.master           I_BUS,          //transfer engine -> ibus_arb DMA leg

    /* BUS ARBITER HOOK */
    output  wire            o_BUS_HOLD,     //transfer-unit / burst bus hold (arb i_DMA_HOLD)

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

logic   [3:0]   ch_upd;                     //sequencer: unit completed on channel c

//feature asymmetry per pp.336-342: DREQ/DACK bits on ch0/1, reload on
//ch2, indirect on ch3; i_UPD drives each channel's iteration datapath
dmac_channel #(.CH_ID(0), .HAS_EXT(1'b1)) u_ch0 (
    .i_RST_n(i_RST_n), .i_CLK(i_CLK), .i_CEN(i_CEN),
    .i_WR_SAR(wr_sar[0]), .i_WR_DAR(wr_dar[0]), .i_WR_TCR(wr_tcr[0]), .i_WR_CHCR(wr_chcr[0]),
    .i_WDATA(wd_lane), .i_WMASK(wm_lane),
    .i_UPD(ch_upd[0]),
    .o_SAR(ch_sar[0]), .o_DAR(ch_dar[0]), .o_TCR(ch_tcr[0]), .o_CHCR(ch_chcr[0])
);
dmac_channel #(.CH_ID(1), .HAS_EXT(1'b1)) u_ch1 (
    .i_RST_n(i_RST_n), .i_CLK(i_CLK), .i_CEN(i_CEN),
    .i_WR_SAR(wr_sar[1]), .i_WR_DAR(wr_dar[1]), .i_WR_TCR(wr_tcr[1]), .i_WR_CHCR(wr_chcr[1]),
    .i_WDATA(wd_lane), .i_WMASK(wm_lane),
    .i_UPD(ch_upd[1]),
    .o_SAR(ch_sar[1]), .o_DAR(ch_dar[1]), .o_TCR(ch_tcr[1]), .o_CHCR(ch_chcr[1])
);
dmac_channel #(.CH_ID(2), .HAS_RELOAD(1'b1)) u_ch2 (
    .i_RST_n(i_RST_n), .i_CLK(i_CLK), .i_CEN(i_CEN),
    .i_WR_SAR(wr_sar[2]), .i_WR_DAR(wr_dar[2]), .i_WR_TCR(wr_tcr[2]), .i_WR_CHCR(wr_chcr[2]),
    .i_WDATA(wd_lane), .i_WMASK(wm_lane),
    .i_UPD(ch_upd[2]),
    .o_SAR(ch_sar[2]), .o_DAR(ch_dar[2]), .o_TCR(ch_tcr[2]), .o_CHCR(ch_chcr[2])
);
dmac_channel #(.CH_ID(3), .HAS_INDIRECT(1'b1)) u_ch3 (
    .i_RST_n(i_RST_n), .i_CLK(i_CLK), .i_CEN(i_CEN),
    .i_WR_SAR(wr_sar[3]), .i_WR_DAR(wr_dar[3]), .i_WR_TCR(wr_tcr[3]), .i_WR_CHCR(wr_chcr[3]),
    .i_WDATA(wd_lane), .i_WMASK(wm_lane),
    .i_UPD(ch_upd[3]),
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
wire            cmt_fire  = cmt_match && !wr_cmcnt; //CMF set + DMA request (fig 11.27)

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
        if(cmt_fire)                       cmf <= 1'b1;
        else if(wr_cmt_a && wm_lane[0])    cmf <= cmf & wd_lane[7];
    end end
end



///////////////////////////////////////////////////////////
//////  Transfer Engine (start-up + request priority + bus interface)
////

/*
    One transfer unit at a time: the dual-direct pair = read at SAR then
    write at DAR (figs 11.5/11.6). Per-edge dataflow (core clock, i_CEN):

      IDLE     ch_req (pending&enable regs, ~2 lvl) -> fixed-priority win
               (~2 lvl, p.349) -> latch grant_q/addr_q(=SAR)/sarlo_q/size_q,
               clear a cycle-steal pending (request withdrawn at the FIRST
               transfer, p.348), go RD_REQ
      RD_REQ   req_valid high, addr/size straight from regs (flat cone
               through the arb 2:1); accept -> RD_WAIT
      RD_WAIT  rsp_valid: extract the SAR-lane datum (shift ~2 lvl),
               replicate onto lanes, latch wdata_q/wstrb_q(DAR lane)/
               addr_q(=DAR), go WR_REQ
      WR_REQ   req_valid+write; accept -> WR_WAIT
      WR_WAIT  rsp_valid (posted-write ack): ch_upd strobe (channel steps
               SAR/DAR/DMATCR, sets TE on the last unit); burst -> IDLE
               (priority re-resolved EVERY unit boundary: a higher-priority
               channel preempts between units, fig 11.14, but o_BUS_HOLD
               keeps the CPU off); cycle-steal -> GAP
      GAP      one request-free cycle so the arb owner returns to the CPU
               (the fig 11.12 cycle-steal boundary), then IDLE

    Ending laws (p.374): enables are checked at grant only - clearing
    DE/DME mid-unit lets the WRITE of the pair complete (law d), the
    channel then stops with TE unset. DMAC address error / NMIF arrive
    in phase 6; rsp_fault is ignored until then.
*/

//request sources (phase 3): auto = level while enabled (p.347); CMT = the
//compare-match pulse latched until served (p.348). Other RS codes inert.
logic   [3:0]   pend;                       //CMT request latch per channel
logic   [3:0]   ch_en, ch_req, ch_tm_v, ch_rs_cmt;
always_comb begin
    for(int c = 0; c < 4; c++) begin
        logic rs_auto;
        rs_auto      = (ch_chcr[c][11:8] == 4'b0100);
        ch_rs_cmt[c] = (ch_chcr[c][11:8] == 4'b1111);
        //enable = DE & ~TE & DME & ~NMIF & ~AE (p.342)
        ch_en[c]     = ch_chcr[c][0] & ~ch_chcr[c][1] & dme & ~nmif & ~ae;
        ch_req[c]    = ch_en[c] & (rs_auto | (ch_rs_cmt[c] & pend[c]));
        ch_tm_v[c]   = ch_chcr[c][5];       //TM: burst flag per channel
    end
end

//fixed channel priority (p.349); PR=11 is round-robin (phase 5) - until
//then it resolves like the reset order
logic   [1:0]   win;
always_comb begin
    unique case(pr)
        2'b01:   win = ch_req[0] ? 2'd0 : ch_req[2] ? 2'd2 : ch_req[3] ? 2'd3 : 2'd1;
        2'b10:   win = ch_req[2] ? 2'd2 : ch_req[0] ? 2'd0 : ch_req[1] ? 2'd1 : 2'd3;
        default: win = ch_req[0] ? 2'd0 : ch_req[1] ? 2'd1 : ch_req[2] ? 2'd2 : 2'd3;
    endcase
end
wire            win_v = |ch_req;

//sequencer state ("seq"): the unit pipeline above
localparam logic [2:0] S_IDLE = 3'd0, S_RD_REQ = 3'd1, S_RD_WAIT = 3'd2,
                       S_WR_REQ = 3'd3, S_WR_WAIT = 3'd4, S_GAP = 3'd5;
logic   [2:0]   seq;
logic   [1:0]   grant_q;                    //granted channel (registered mux select)
logic   [1:0]   sarlo_q;                    //granted SAR[1:0]: read-lane pick
logic   [1:0]   size_q;                     //granted TS size
logic   [31:0]  addr_q;                     //read address, then write address
logic   [31:0]  wdata_q;                    //lane-replicated write data
logic   [3:0]   wstrb_q;                    //DAR-lane strobes

//granted-channel views (4:1 muxes, registered grant_q select)
wire    [31:0]  dar_g = ch_dar[grant_q];
wire            tm_g  = ch_tm_v[grant_q];

//read-lane extract: right-justify the SAR-addressed datum (big-endian,
//(3-a)*8 = {~a,000}), then replicate - wstrb picks the DAR lane, so no
//DAR-dependent shift is ever needed
wire    [31:0]  rd_sh = I_BUS.rsp_rdata >> {~sarlo_q, 3'b000};
logic   [31:0]  wr_rep;
logic   [3:0]   wr_stb;
always_comb begin
    unique case(size_q)
        2'd0: begin                         //byte
            wr_rep = {4{rd_sh[7:0]}};
            wr_stb = 4'b1000 >> dar_g[1:0];
        end
        2'd1: begin                         //word
            wr_rep = {2{sarlo_q[1] ? I_BUS.rsp_rdata[15:0] : I_BUS.rsp_rdata[31:16]}};
            wr_stb = dar_g[1] ? 4'b0011 : 4'b1100;
        end
        default: begin                      //longword
            wr_rep = I_BUS.rsp_rdata;
            wr_stb = 4'b1111;
        end
    endcase
end

wire            unit_done = (seq == S_WR_WAIT) && I_BUS.rsp_valid;

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        seq     <= S_IDLE;
        grant_q <= 2'd0;
        sarlo_q <= 2'd0;
        size_q  <= 2'd0;
        addr_q  <= 32'd0;
        wdata_q <= 32'd0;
        wstrb_q <= 4'd0;
        pend    <= 4'd0;
    end
    else begin if(i_CEN) begin
        unique case(seq)
            S_IDLE: begin
                if(win_v) begin             //start-up: latch the winner's unit
                    grant_q <= win;
                    addr_q  <= ch_sar[win];
                    sarlo_q <= ch_sar[win][1:0];
                    size_q  <= {ch_chcr[win][4], ch_chcr[win][3]};
                    seq     <= S_RD_REQ;
                end
            end
            S_RD_REQ:  if(I_BUS.req_ready) seq <= S_RD_WAIT;
            S_RD_WAIT: begin
                if(I_BUS.rsp_valid) begin   //buffer the datum, turn the pair around
                    wdata_q <= wr_rep;
                    wstrb_q <= wr_stb;
                    addr_q  <= dar_g;
                    seq     <= S_WR_REQ;
                end
            end
            S_WR_REQ:  if(I_BUS.req_ready) seq <= S_WR_WAIT;
            S_WR_WAIT: begin
                if(I_BUS.rsp_valid)         //burst re-arbitrates at once (fig 11.14);
                    seq <= tm_g ? S_IDLE : S_GAP;   //cycle-steal yields a CPU boundary
            end
            default: seq <= S_IDLE;         //S_GAP: one request-free cycle
        endcase

        //CMT pending: match sets (outranks the clears), a cycle-steal grant
        //withdraws at the FIRST transfer, burst at the LAST (p.348)
        for(int c = 0; c < 4; c++) begin
            if(cmt_fire && ch_rs_cmt[c])                             pend[c] <= 1'b1;
            else if(seq == S_IDLE && win_v && win == c[1:0] && !ch_tm_v[c]) pend[c] <= 1'b0;
            else if(ch_upd[c] && ch_tcr[c] == 24'd1)                 pend[c] <= 1'b0;
        end
    end end
end

//unit-completion strobes into the channel iteration datapaths
always_comb begin
    for(int c = 0; c < 4; c++) ch_upd[c] = unit_done && (grant_q == c[1:0]);
end

//bus master drive: request fields straight from registers - one flat level
//into the arb's 2:1 (the reqn/addr 5 ns class stays shallow)
assign  I_BUS.req_valid   = (seq == S_RD_REQ) || (seq == S_WR_REQ);
assign  I_BUS.req_write   = (seq == S_WR_REQ);
assign  I_BUS.req_size    = size_q;
assign  I_BUS.req_burst   = 1'b0;
assign  I_BUS.req_addr    = addr_q;
assign  I_BUS.req_wdata   = wdata_q;
assign  I_BUS.req_wstrb   = (seq == S_WR_REQ) ? wstrb_q : 4'd0;
assign  I_BUS.req_lock    = 1'b0;
assign  I_BUS.req_dack    = 1'b0;           //DACK/single-address tags arrive in phase 4
assign  I_BUS.req_dack_ch = 1'b0;
assign  I_BUS.req_dack_al = 1'b0;
assign  I_BUS.req_saddr   = 1'b0;
assign  I_BUS.rsp_ready   = (seq == S_RD_WAIT) || (seq == S_WR_WAIT);

//bus hold: through the unit (the R->W pair is indivisible vs the CPU,
//fig 11.12 tier 1) and across burst units (CPU locked out, fig 11.13/11.14)
assign  o_BUS_HOLD = ((seq != S_IDLE) && (seq != S_GAP)) || (|(ch_req & ch_tm_v));



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
