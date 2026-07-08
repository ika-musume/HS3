`default_nettype wire

/*
    DMAC channel register quad (SH7709S section 11, pp.331-342).

    One instance per channel: Fig 11.1's block repeated four times is
    SARn/DARn/DMATCRn/CHCRn plus its iteration datapath (address/count
    update - arrives with the transfer phases). The channel feature
    asymmetry of pp.336-342 is parameterized:
      HAS_EXT      ch0/1: external request bits RL/AM/AL (18:16), DS (6)
      HAS_RELOAD   ch2:   source address reload bit RO (19)
      HAS_INDIRECT ch3:   indirect addressing bit DI (20)
    Absent-feature bits: write invalid, read 0 (p.336).

    Write rules (table 11.2 notes, p.332): TE (CHCR[1]) is write-0-only -
    a hardware set outranks a same-edge clear, write-1 holds; DMATCR
    bits 31:24 read 0 / write-ignored; 16-bit partial access keeps the
    untouched half (the parent delivers lane-aligned data + a byte mask).
    CHCR clears on power-on AND manual reset; SAR/DAR/DMATCR are
    "undefined" at reset on silicon - implemented as reset-to-0, tests
    never rely on it.
*/

module dmac_channel #(
    parameter               CH_ID        = 0,       //channel number (debug/comments only)
    parameter               HAS_EXT      = 1'b0,    //ch0/1: DREQ/DACK control bits exist
    parameter               HAS_RELOAD   = 1'b0,    //ch2: RO bit exists
    parameter               HAS_INDIRECT = 1'b0     //ch3: DI bit exists
) (
    /* CLOCK AND RESET - clears on any reset flavor (p.332) */
    input   wire            i_RST_n,
    input   wire            i_CLK,
    input   wire            i_CEN,

    /* REGISTER ACCESS - one-cycle strobes from the parent's P-bus decode */
    input   wire            i_WR_SAR,
    input   wire            i_WR_DAR,
    input   wire            i_WR_TCR,       //TCR = DMATCR, the 24-bit transfer count register
    input   wire            i_WR_CHCR,
    input   wire    [31:0]  i_WDATA,        //pre-aligned to register bit lanes (big-endian)
    input   wire    [3:0]   i_WMASK,        //byte-lane mask, [3] = bits 31:24

    /* TRANSFER ENGINE HOOKS - tied off until the transfer phases */
    input   wire            i_TE_SET,       //sequencer: DMATCR count completed

    /* REGISTER READ-BACK */
    output  wire    [31:0]  o_SAR,
    output  wire    [31:0]  o_DAR,
    output  wire    [23:0]  o_TCR,
    output  wire    [31:0]  o_CHCR
);

///////////////////////////////////////////////////////////
//////  Register Storage
////

logic   [31:0]  sar;                        //next source address during transfer (p.333)
logic   [31:0]  dar;                        //next destination address during transfer (p.334)
logic   [23:0]  tcr;                        //remaining transfer count; 0 = 16M max (p.335)
//CHCR fields (pp.336-342); absent-feature bits stay 0 forever
logic           di;                         //CHCR[20]: ch3 indirect address mode
logic           ro;                         //CHCR[19]: ch2 source address reload
logic           rl;                         //CHCR[18]: DRAK polarity (1 = active-high)
logic           am;                         //CHCR[17]: DACK in read(0)/write(1) dual cycle
logic           al;                         //CHCR[16]: DACK polarity (1 = active-high)
logic   [1:0]   dm, sm;                     //dest/source address mode: fixed/inc/dec
logic   [3:0]   rs;                         //resource select (request source, p.339)
logic           ds;                         //CHCR[6]: DREQ low-level(0)/falling-edge(1)
logic           tm;                         //CHCR[5]: cycle-steal(0)/burst(1)
logic   [1:0]   ts;                         //transmit size: byte/word/long/16-byte
logic           ie, te, de;                 //interrupt enable / transfer end / enable



///////////////////////////////////////////////////////////
//////  Register Writes
////

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        sar <= 32'd0;
        dar <= 32'd0;
        tcr <= 24'd0;
        {di, ro, rl, am, al} <= 5'd0;
        dm  <= 2'd0;
        sm  <= 2'd0;
        rs  <= 4'd0;
        {ds, tm} <= 2'd0;
        ts  <= 2'd0;
        {ie, te, de} <= 3'd0;
    end
    else begin if(i_CEN) begin
        //byte-lane writes: a 16-bit access keeps the untouched half (p.332 note 2)
        for(int b = 0; b < 4; b++) begin
            if(i_WR_SAR && i_WMASK[b]) sar[b*8 +: 8] <= i_WDATA[b*8 +: 8];
            if(i_WR_DAR && i_WMASK[b]) dar[b*8 +: 8] <= i_WDATA[b*8 +: 8];
        end
        for(int b = 0; b < 3; b++) begin    //DMATCR[31:24] write-ignored (p.332 note 3)
            if(i_WR_TCR && i_WMASK[b]) tcr[b*8 +: 8] <= i_WDATA[b*8 +: 8];
        end

        //absent-feature bits load constant 0 (write invalid, read 0, p.336):
        //gating the VALUE, not the assignment, keeps a real flop D input -
        //an if(PARAM)-guarded assign makes Quartus infer a reset-only latch
        if(i_WR_CHCR) begin
            if(i_WMASK[2]) begin            //bits 23:16 - the channel-exclusive controls
                di <= HAS_INDIRECT ? i_WDATA[20] : 1'b0;
                ro <= HAS_RELOAD   ? i_WDATA[19] : 1'b0;
                rl <= HAS_EXT      ? i_WDATA[18] : 1'b0;
                am <= HAS_EXT      ? i_WDATA[17] : 1'b0;
                al <= HAS_EXT      ? i_WDATA[16] : 1'b0;
            end
            if(i_WMASK[1]) begin            //bits 15:8
                dm <= i_WDATA[15:14];
                sm <= i_WDATA[13:12];
                rs <= i_WDATA[11:8];
            end
            if(i_WMASK[0]) begin            //bits 7:0 (TE handled below)
                ds <= HAS_EXT ? i_WDATA[6] : 1'b0;
                tm <= i_WDATA[5];
                ts <= i_WDATA[4:3];
                ie <= i_WDATA[2];
                de <= i_WDATA[0];
            end
        end

        //TE: hardware set outranks a same-edge write; write-1 never sets (p.341)
        if(i_TE_SET)                     te <= 1'b1;
        else if(i_WR_CHCR && i_WMASK[0]) te <= te & i_WDATA[1];
    end end
end



///////////////////////////////////////////////////////////
//////  Read-Back
////

assign  o_SAR  = sar;
assign  o_DAR  = dar;
assign  o_TCR  = tcr;
assign  o_CHCR = {11'd0, di, ro, rl, am, al, dm, sm, rs, 1'b0, ds, tm, ts, ie, te, de};

endmodule

`default_nettype none
