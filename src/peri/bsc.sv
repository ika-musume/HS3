`default_nettype wire

/*
    BSC - bus state controller (SH7709S section 10, pp.223-304).

    Slaves the splitter's external I-bus-1 leg and serves four worlds:

      1. its own register file (0xFFFFFF50-74, Appendix B: I bus) + the SDMR
         address windows (0xFFFFD000/0xFFFFE000, p.252);
      2. the ORIGINAL SDRAM interface - areas 2/3 when BCR1.DRAMTP selects
         synchronous DRAM (p.234). Command timing is register-programmed
         (MCR/WCR2) and runs on the 50 MHz bus enable i_BCEN, reproducing the
         real chip's natural latency (figs 10.14-10.28). Burst-read/single-
         write, BL=1: a cache line moves as 4 pipelined READ/WRIT commands.
      3. the ORDINARY MEMORY / BURST ROM controller for every other area
         (p.268, p.304): the access is held on the SHARED physical bus (and
         mirrored on the generic port) for a full register-programmed bus
         cycle - WCR2 first-access waits, the burst pitch of BCR1-enabled
         burst ROM areas, and i_WAIT_n stretching - then read data is
         sampled live from the D bus (i_D_I), like real chip pins. A
         handshake controller may instead complete any access EARLY by
         pulsing i_MEM_RSP_VALID (ORed in, data from i_MEM_RDATA), so tying
         the handshake low (unused) works with only i_WAIT_n; the fast path
         also keeps the IPC parity constants intact.
      4. the P-BUS register tier (Appendix B "P" bus: TMU 0xFFFFFE90-B8,
         RTC 0xFFFFFEC0-DE, PFC/ports 0x04000100-137) - an in-BSC bridge
         with the ibus_bridge contract (IDLE->ACCESS->RESP, right-justified
         writes, lane-replicated reads) driving the REG_TMU/REG_RTC/REG_PORT
         IBus_2 legs. Handshake latency is allowed on this tier only (user
         latency law); P-window accesses never touch the external pins.

    Front-end/register file tick on i_CEN (100 MHz core rate); only the
    PCB-facing SDRAM engine + refresh timer tick on i_BCEN. Same clock, so
    the flag exchange is plain registers - no synchronizers (user decision).

    Reset domains (p.228 note, p.297): ALL BSC registers and the SDRAM
    engine/refresh reset on power-on reset only - refresh keeps running
    through a manual reset. Only the front-end handshake clears on every
    reset flavor so a dying transaction cannot wedge the bus.

    DEVIATIONS (vs real silicon):
      - no PCMCIA (A5PCM/A6PCM/PCR are bookkeeping); no MCS pins (MCSCR0-7
        bookkeeping); no standby coupling. BREQ/BACK ARE implemented: grant
        drains the bus, PALLs open bank-active rows, releases the shared
        control/address pads via o_BUS_OE (CKE stays driven; a self-refresh
        park keeps its CKE-low state through a release).
      - BS_n on SDRAM reads aligns with the READ commands, not the Td data
        cycles of fig 10.14 (cosmetic; ordinary cycles assert it at T1).
      - AMX = 0111 only (512K x 32-bit x 4-bank, table 10.13) - the target
        device (MT48LC2M32B2). Other AMX values are not decoded.
      - reads drive DQM all-low (whole longword fetched; CPU extracts bytes).
      - fill beats leave in sequential word order (the cache requests word 0
        first); the real chip wraps from the missed word. Latency identical.
      - one dispatch NOP between an op leaving E_IDLE and its first command
        (the real chip overlaps dispatch with the previous Tpc tail).
      - WCR1 inter-access idles are NOT enforced: they exist to avoid data
        bus turnaround conflicts, and the generic port has split read/write
        data - no shared bus to protect. WCR2 waits ARE enforced (below).
      - 8-bit port widths are unsupported (served as 16-bit); the SDRAM path
        stays 32-bit regardless of BCR2 (AMX 0111 device).
      - i_MEM_READY and i_MEM_RSP_VALID are equivalent external completions
        (both ORed with the timed path); accept pacing is always internal.
        The generic port carries no data - the physical D pins do.
      - area 1 (internal I/O) and area 7 (reserved) answer locally (read 0,
        posted write) instead of emitting external cycles.
      - the generic port mirrors ALL external areas: BSC-owned accesses
        (SDRAM areas, dummy 1/7) appear as one-cycle accept strobes for
        shadowing; the external controller masks them by its own memory map.
      - CMF/OVF clear on a keyed write-0 (real CMF clears at the next refresh
        after write-0 - simplified).
*/

module bsc #(
    parameter       BIG_ENDIAN = 1'b1   //reflected in BCR1.ENDIAN (MD5 pin on silicon)
) (
    /* CLOCK AND RESET */
    input   wire            i_POR_n,    //registers + SDRAM engine + refresh (survives manual reset)
    input   wire            i_RST_n,    //front-end handshake only (all-flavor reset)
    input   wire            i_CLK,
    input   wire            i_CEN,      //core enable - front-end / register file / generic leg
    (* direct_enable *)
    input   wire            i_BCEN,     //bus enable (50 MHz) - SDRAM engine + refresh timer

    /* INTERFACES */
    IBus_1.slave            I_BUS,      //from the splitter's external leg
    IBus_2.master           REG_TMU,    //P bus: TMU window 0xFFFFFE90-B8
    IBus_2.master           REG_RTC,    //P bus: RTC window 0xFFFFFEC0-DE
    IBus_2.master           REG_PORT,   //P bus: PFC/port window 0x04000100-137

    /* BSC PHYSICAL PINS - the real chip's external bus, table 10.1
       (pp.226-227). Ordinary memory / burst ROM and SDRAM SHARE these,
       exactly as on silicon: RD/WR doubles as the SDRAM WE command bit and
       WE3-WE0 double as DQMUU-DQMLL. The data bus is split unidirectional
       (o_D_O/o_D_OE/i_D_I); the true inout lives at the board level.
       PCMCIA pins (CE2A/B, ICIORD/ICIOWR, IOIS16) and the MCS0-7 mask-ROM
       selects are omitted; BREQ/BACK are present but inert. */
    output  wire    [25:0]  o_A,            //shared: static ordinary / muxed SDRAM row-col
    output  wire    [31:0]  o_D_O,
    output  wire            o_D_OE,
    input   wire    [31:0]  i_D_I,
    output  wire            o_BS_n,         //bus cycle start
    output  wire            o_CS0_n,
    output  wire            o_CS2_n,
    output  wire            o_CS3_n,
    output  wire            o_CS4_n,
    output  wire            o_CS5_n,
    output  wire            o_CS6_n,
    output  wire            o_RD_WR,        //bus direction / SDRAM WE command bit
    output  wire            o_RAS3L_n,      //lower 32MB row strobe
    output  wire            o_RAS3U_n,      //upper 32MB row strobe
    output  wire            o_CASL_n,
    output  wire            o_CASU_n,
    output  wire    [3:0]   o_WE_n,         //WE3-WE0 write strobes / DQMUU-DQMLL
    output  wire            o_RD_n,         //ordinary read strobe
    input   wire            i_WAIT_n,       //sampled after the WCR2 waits when enabled
    input   wire            i_MD4,          //area-0 bus width straps (table 10.4:
    input   wire            i_MD3,          //10=16-bit, 11=32-bit; 8-bit unsupported)
    output  wire            o_CKE,
    input   wire            i_BREQ_n,       //bus release request (2FF-synced)
    output  wire            o_BACK_n,       //grant: shared bus released
    output  wire            o_BUS_OE,       //board-level pad enable for the shared
                                            //control/address group (D uses o_D_OE;
                                            //CKE stays driven through a release)

    /* GENERIC MEMORY PORT - address/control view only: ALL data rides the
       physical D pins (user rule). Every external access is mirrored here;
       ordinary accesses are held for their timed bus cycle. EITHER
       i_MEM_READY or i_MEM_RSP_VALID completes an ordinary access early -
       always as one full 32-bit D-bus transfer, bypassing the width split -
       so tying both low leaves i_WAIT_n in sole control. */
    output  wire            o_MEM_REQ,
    output  wire            o_MEM_WRITE,
    output  wire            o_MEM_BURST,    //beat of a 16-byte line transfer
    output  wire    [1:0]   o_MEM_SIZE,
    output  wire    [28:0]  o_MEM_ADDR,     //physical (A31-29 shadow stripped, p.232)
    output  wire    [6:0]   o_MEM_CS_n,     //area strobes; bit n = area n (bit 1 never asserts)
    output  wire    [3:0]   o_MEM_WSTRB,    //32-bit-lane byte enables (control view)
    input   wire            i_MEM_READY,
    input   wire            i_MEM_RSP_VALID,
    input   wire            i_MEM_FAULT,
    output  wire            o_MEM_RSP_READY,

    /* REFRESH TIMER INTERRUPTS (table 6.4 REF entries) */
    output  wire            o_RCMI_REQ,     //compare match  (INTEVT 0x580)
    output  wire            o_ROVI_REQ      //count overflow (INTEVT 0x5A0)
);



///////////////////////////////////////////////////////////
//////  Register File (power-on reset only, p.228)
////

logic   [15:0]  bcr1;               //memory type select; init 0x0000 (p.233)
logic   [15:0]  bcr2;               //area bus width, bookkeeping; init 0x3FF0 (p.239)
logic   [15:0]  wcr1;               //inter-access idles, exported only; init 0x3FF3 (p.240)
logic   [15:0]  wcr2;               //waits + SDRAM CAS latency; init 0xFFFF (p.241)
logic   [15:0]  mcr;                //SDRAM timing; init 0x0000 (p.245)
logic   [15:0]  pcr;                //PCMCIA, bookkeeping only; init 0x0000 (p.248)
logic   [7:0]   rtcsr;              //{CMF,CMIE,CKS[2:0],OVF,OVIE,LMTS} (p.253)
logic   [7:0]   rtcnt;              //refresh timer counter (p.255)
logic   [7:0]   rtcor;              //refresh time constant (p.256)
logic   [9:0]   rfcr;               //refresh count (p.256)
logic   [15:0]  mcscr [0:7];        //MCS0-7 pin control, bookkeeping (pp.258-259)

//decoded engine timing knobs (the natural-latency law, MCR pp.245-247)
wire            a2_sdram  = (bcr1[4:2] == 3'b011);      //DRAMTP=011: both areas SDRAM
wire            a3_sdram  = !bcr1[4] && bcr1[3];        //DRAMTP=010 or 011
wire    [2:0]   t_tpc     = {1'b0, mcr[15:14]} + 3'd1;              //precharge spacing 1-4
wire    [3:0]   t_tpc_slf = {2'b00, mcr[15:14]} * 3 + 4'd2;         //self-refresh exit 2/5/8/11
wire    [2:0]   t_rcd     = {1'b0, mcr[13:12]} + 3'd1;              //RAS-CAS spacing 1-4
wire    [2:0]   t_trwl    = {1'b0, mcr[11:10]} + 3'd1;              //write recovery 1-3
wire    [2:0]   t_tras    = {1'b0, mcr[9:8]}   + 3'd2;              //refresh lockout 2-5
wire            rasd      = mcr[7];                                 //bank active mode
wire            rfsh      = mcr[2];
wire            rmode     = mcr[1];
//CAS latency per area (WCR2 A3W/A2W, p.243: 00/01=1, 10=2, 11=3)
wire    [1:0]   cl_a3     = (wcr2[6:5] == 2'b00) ? 2'd1 : wcr2[6:5];
wire    [1:0]   cl_a2     = (wcr2[4:3] == 2'b00) ? 2'd1 : wcr2[4:3];

///////////////////////////////////////////////////////////
//////  Front-End Decode (core rate, zero added beats)
////

/*
    Route classes on the live request address:
      REG    P4 0xFFFFFF50-7F register window
      SDMR   P4 0xFFFFD000-DFFF (area 2) / 0xFFFFE000-EFFF (area 3)
      SDRAM  areas 2/3 when DRAMTP selects synchronous DRAM
      PBUS   P4 0xFFFFFE90-BF (TMU) / 0xFFFFFEC0-DF (RTC) /
             area 1 0x04000100-13F (PFC/ports)
      LOCAL  area 1 / area 7 / any other unclaimed P4 address (read 0, posted)
      GEN    everything else -> the ordinary/burst-ROM bus controller
*/

wire    [31:0]  fa       = I_BUS.req_addr;
wire            fe_p4    = (fa[31:29] == 3'b111);
wire            fe_reg   = (fa[31:8] == 24'hFFFF_FF) &&
                           (fa[7:4] == 4'h5 || fa[7:4] == 4'h6 || fa[7:4] == 4'h7);
wire            fe_sdmr  = (fa[31:12] == 20'hFFFFD) || (fa[31:12] == 20'hFFFFE);
wire    [2:0]   fe_area  = fa[28:26];
wire            fe_sdram = !fe_p4 && ((fe_area == 3'd2 && a2_sdram) ||
                                      (fe_area == 3'd3 && a3_sdram));
//P-bus register windows (Appendix B): TMU/RTC on P4, PFC/ports in area-1 space
wire            fe_tmu   = (fa[31:8] == 24'hFFFF_FE) &&
                           (fa[7:4] == 4'h9 || fa[7:4] == 4'hA || fa[7:4] == 4'hB);
wire            fe_rtc   = (fa[31:8] == 24'hFFFF_FE) &&
                           (fa[7:4] == 4'hC || fa[7:4] == 4'hD);
wire            fe_port  = !fe_p4 && (fe_area == 3'd1) && (fa[25:6] == 20'h0_0004);
wire            fe_pbus  = fe_tmu || fe_rtc || fe_port;
wire            fe_dummy = (fe_p4 && !fe_reg && !fe_sdmr && !fe_tmu && !fe_rtc) ||
                           (!fe_p4 && ((fe_area == 3'd1 && !fe_port) || fe_area == 3'd7));
wire            fe_local = fe_reg || fe_dummy;
wire            fe_gen   = !fe_p4 && !fe_sdram && !fe_dummy && !fe_port;
wire            fe_eng   = fe_sdram || fe_sdmr;         //engine-owned classes

wire            bus_held;           //BREQ requested or granted (defined below)

//owner of the outstanding response (splitter owner_brg pattern)
localparam [1:0] OWN_GEN = 2'd0, OWN_LOC = 2'd1, OWN_SDR = 2'd2, OWN_PBS = 2'd3;
logic   [1:0]   owner_q;

//SDRAM burst bookkeeping: a fill/drain arrives as 4 line-aligned beats; the
//head starts the engine, continuations ride the running op (line buffers)
logic           b_rd_act;           //burst read in flight (engine fetches the line)
logic           b_wr_act;           //burst write in flight (line buffer drains to pins)
wire            fe_b_head = I_BUS.req_burst && (fa[3:2] == 2'b00);
wire            fe_b_cont = fe_sdram && ((b_rd_act && !I_BUS.req_write) ||
                                         (b_wr_act &&  I_BUS.req_write)) && (fa[3:2] != 2'b00);

//engine request slot (single outstanding; consumed by the engine at a BCEN edge)
logic           eng_go;
wire            eng_busy;           //engine FSM outside E_IDLE
logic           self_active;        //self-refresh entered; requests stall until exit

//local (register/dummy) one-beat registered response
logic           loc_rsp_v;
logic   [31:0]  loc_rsp_d;

//SDRAM response trackers
logic           sd_rd_wait;         //a read beat awaits its line-buffer word
logic   [1:0]   sd_rd_beat;
logic           sd_wr_ack;          //posted-write/SDMR ack pending
logic   [3:0]   wv;                 //read line-buffer word valid (engine sets, head accept clears)
logic   [31:0]  rd_buf  [0:3];      //engine -> front-end read line buffer
logic   [31:0]  wr_buf  [0:3];      //front-end -> engine write line buffer
logic   [3:0]   wr_strb [0:3];
logic   [3:0]   wr_v;

//TAS atomicity (p.320): the bus is never released between a locked pair's
//read and write. fe_lock_hold tracks a pair on the ordinary/generic path
//(e_lock_hold is its SDRAM-engine twin). While a pair is open a pending
//BREQ must not stall accepts either (a fetch can sit ahead of the pair's
//write on the single-outstanding bus - blocking it deadlocks the release):
//the bus runs normally until the legal release point after the write.
logic           fe_lock_hold;
wire            bus_blk      = bus_held && !(fe_lock_hold || e_lock_hold);

//ready per class. A head/single engine op needs the whole engine idle; burst
//continuations only need the previous beat's response consumed.
wire            sd_ready  = fe_b_cont ? (!sd_rd_wait && !sd_wr_ack) :
                            (!eng_go && !eng_busy && !self_active && !bus_blk &&
                             !sd_rd_wait && !sd_wr_ack);
wire            loc_ready = !loc_rsp_v;
wire            pbs_ready;          //P-bus bridge idle (defined in its section)

//the SHARED external bus admits one cycle at a time: ordinary accepts wait
//for the engine (a posted SDRAM write/MRS/refresh still owns the pins) and
//the engine waits for an open ordinary cycle (dispatch gate in E_IDLE)
assign  I_BUS.req_ready = fe_gen  ? !(ord_busy || eng_busy || eng_go || bus_blk) :
                          fe_eng  ? sd_ready  :
                          fe_pbus ? pbs_ready : loc_ready;

wire            fe_acc     = I_BUS.req_valid && I_BUS.req_ready;
wire            fe_acc_eng = fe_acc && fe_eng;

//front-end locked pair: a locked read opens the hold, its write closes it
//at the accept edge (ord_busy then keeps the bus through the write cycle);
//a pair killed by a bus fault on the read unlatches at that response
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) fe_lock_hold <= 1'b0;
    else begin if(i_CEN) begin
        if(fe_acc && fe_gen && I_BUS.req_lock)
            fe_lock_hold <= !I_BUS.req_write;
        else if(fe_rsp_done && owner_q == OWN_GEN && I_BUS.rsp_fault)
            fe_lock_hold <= 1'b0;
    end end
end



///////////////////////////////////////////////////////////
//////  Generic Leg Pass-Through (the IPC-parity path)
////

/*
    ALL external areas are exposed on the port (user rule: the external
    controller masks by address + its own map). BSC-owned areas (SDRAM 2/3,
    dummy 1/7) appear as a ONE-CYCLE strobe at their internal accept edge -
    one pulse = one committed access, observation only. Ordinary/burst-ROM
    accesses are HELD on the port (latched fields, level o_MEM_REQ) for the
    whole register-timed bus cycle so a raw memory can decode them like a
    real external bus; a handshake controller instead pulses i_MEM_RSP_VALID
    to complete early (the IPC-parity fast path).
*/

//ordinary bus cycle in flight: the port carries the latched access
logic           ord_busy;
logic           ord_done;           //timed-path completion (registered, one rsp)
logic   [31:0]  ord_addr, ord_wdata, ord_data;
logic           ord_write, ord_burst;
logic   [1:0]   ord_size;
logic   [3:0]   ord_wstrb;
logic   [2:0]   ord_area;

assign  o_MEM_REQ       = ord_busy ? 1'b1            :
                          (I_BUS.req_valid & ~fe_p4 & (fe_gen | I_BUS.req_ready));
assign  o_MEM_WRITE     = ord_busy ? ord_write       : I_BUS.req_write;
assign  o_MEM_BURST     = ord_busy ? ord_burst       : I_BUS.req_burst;
assign  o_MEM_SIZE      = ord_busy ? ord_size        : I_BUS.req_size;
assign  o_MEM_ADDR      = ord_busy ? ord_addr[28:0]  : fa[28:0];
assign  o_MEM_WSTRB     = ord_busy ? ord_wstrb       : I_BUS.req_wstrb;
assign  o_MEM_RSP_READY = I_BUS.rsp_ready & (owner_q == OWN_GEN);

wire    [2:0]   cs_area = ord_busy ? ord_area : fe_area;
genvar gi;
generate for(gi = 0; gi < 7; gi = gi + 1) begin : g_cs
    assign o_MEM_CS_n[gi] = ~(o_MEM_REQ && cs_area == gi[2:0]);
end endgenerate

//response mux back to the cache: the generic owner completes on the external
//handshake (READY or RSP_VALID, one full 32-bit D-bus transfer - the parity
//fast path) OR the width-aware timed bus cycle (ord_data assembly)
wire            gen_ext_done = i_MEM_RSP_VALID | i_MEM_READY;
wire            pbs_rsp_v;          //P-bus bridge response (defined in its section)
logic   [31:0]  pbs_rdata_q;
assign  I_BUS.rsp_valid = (owner_q == OWN_GEN) ? (gen_ext_done | ord_done) :
                          (owner_q == OWN_LOC) ? loc_rsp_v       :
                          (owner_q == OWN_PBS) ? pbs_rsp_v       :
                          (sd_rd_wait ? wv[sd_rd_beat] : sd_wr_ack);
assign  I_BUS.rsp_rdata = (owner_q == OWN_GEN) ? (gen_ext_done ? i_D_I : ord_data) :
                          (owner_q == OWN_LOC) ? loc_rsp_d   :
                          (owner_q == OWN_PBS) ? pbs_rdata_q : rd_buf[sd_rd_beat];
assign  I_BUS.rsp_fault = (owner_q == OWN_GEN) ? (gen_ext_done & i_MEM_FAULT) : 1'b0;

wire            fe_rsp_done = I_BUS.rsp_valid && I_BUS.rsp_ready;



///////////////////////////////////////////////////////////
//////  Ordinary Memory / Burst ROM Controller (pp.268, 304)
////

/*
    Bus cycles tick on i_BCEN (the 50 MHz CKIO view). An access completes
    after 1 + WCR2-first-wait cycles, then stalls while i_WAIT_n is low if
    the pin is enabled for that area (any nonzero wait setting; a 0-wait
    area ignores the pin, pp.241-244). Burst-ROM continuation beats (line
    fill beats 1-3 of a BCR1-enabled area, p.304) use the shorter burst
    pitch instead of the first-access waits. Read data samples i_MEM_RDATA
    live at the completing bus edge, like pins of a real asynchronous bus.
*/

//WCR2 3-bit encodings: first-access waits and burst pitch (states-1), p.241
function automatic logic [3:0] w3_first(input logic [2:0] c);
    case(c)
        3'd0: w3_first = 4'd0;  3'd1: w3_first = 4'd1;
        3'd2: w3_first = 4'd2;  3'd3: w3_first = 4'd3;
        3'd4: w3_first = 4'd4;  3'd5: w3_first = 4'd6;
        3'd6: w3_first = 4'd8;  default: w3_first = 4'd10;
    endcase
endfunction

function automatic logic [3:0] w3_pitch(input logic [2:0] c);
    case(c)
        3'd0: w3_pitch = 4'd1;  3'd1: w3_pitch = 4'd1;
        3'd2: w3_pitch = 4'd2;  3'd3: w3_pitch = 4'd3;
        3'd4: w3_pitch = 4'd3;  3'd5: w3_pitch = 4'd5;
        3'd6: w3_pitch = 4'd7;  default: w3_pitch = 4'd9;
    endcase
endfunction

//live per-area timing view of the incoming request (captured at accept)
logic   [2:0]   ord_w3;             //3-bit WCR2 code of the addressed area
logic   [3:0]   ord_first;          //first-access wait states
logic           ord_pin_en;         //WAIT pin sampled for this area
logic           ord_bst_en;         //burst ROM enabled for this area (BCR1)
always_comb begin
    case(fe_area)
        3'd0:    begin ord_w3 = wcr2[2:0];         ord_bst_en = (bcr1[10:9] != 2'd0); end
        3'd2:    begin ord_w3 = {1'b0, wcr2[4:3]}; ord_bst_en = 1'b0; end
        3'd3:    begin ord_w3 = {1'b0, wcr2[6:5]}; ord_bst_en = 1'b0; end
        3'd4:    begin ord_w3 = wcr2[9:7];         ord_bst_en = 1'b0; end
        3'd5:    begin ord_w3 = wcr2[12:10];       ord_bst_en = (bcr1[8:7] != 2'd0); end
        default: begin ord_w3 = wcr2[15:13];       ord_bst_en = (bcr1[6:5] != 2'd0); end
    endcase
    //areas 2/3 as ordinary memory use the plain 0-3 wait code (p.243)
    ord_first  = (fe_area == 3'd2 || fe_area == 3'd3) ? {2'd0, ord_w3[1:0]} : w3_first(ord_w3);
    ord_pin_en = (ord_w3 != 3'd0);
end

//a burst-ROM continuation beat rides the open line: pitch timing (p.304)
wire            ord_cont = I_BUS.req_burst && ord_bst_en && (fa[3:2] != 2'b00);

//per-area bus width: BCR2 AnSZ for areas 2-6, MD pins for area 0 (p.231).
//11 = 32-bit; anything narrower is served as 16-bit on D15-D0 (8-bit ports
//are unsupported); a longword then takes TWO full bus cycles, MS half first
logic           ord_w16_c;
always_comb begin
    logic [1:0] a_sz;
    case(fe_area)
        3'd0:    a_sz = {i_MD4, i_MD3};
        3'd2:    a_sz = bcr2[5:4];
        3'd3:    a_sz = bcr2[7:6];
        3'd4:    a_sz = bcr2[9:8];
        3'd5:    a_sz = bcr2[11:10];
        default: a_sz = bcr2[13:12];
    endcase
    ord_w16_c = (a_sz != 2'b11);
end

logic   [3:0]   ord_cnt;            //remaining wait states of this bus cycle
logic   [3:0]   ord_cnt2;           //wait count of the second 16-bit sub-cycle
logic           ord_pin_q;
logic           ord_bs_n;           //BS strobe: low for the first cycle of each sub
logic           ord_w16;            //this access runs on a 16-bit port (D15-D0)
logic           ord_a1;             //current half: 0 = MS (lanes 31:16), 1 = LS
logic           ord_second;         //a second sub-cycle is still owed
logic           ord_run;            //pins asserted: bus cycle is ON the bus-clock grid

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        ord_busy <= 1'b0;
        ord_done <= 1'b0;
        ord_cnt  <= 4'd0;
        ord_cnt2 <= 4'd0;
        ord_pin_q<= 1'b0;
        ord_bs_n <= 1'b1;
        ord_w16  <= 1'b0;
        ord_a1   <= 1'b0;
        ord_second <= 1'b0;
        ord_run  <= 1'b0;
    end
    else begin
        if(i_CEN && fe_acc && fe_gen) begin             //request latched at the accept edge;
            ord_busy  <= 1'b1;                          //pins assert on the bus-clock grid
            ord_done  <= 1'b0;                          //(an off-grid accept waits <=1 core
            ord_run   <= i_BCEN;                        //cycle - real CKIO cycles are aligned)
            ord_cnt   <= ord_cont ? w3_pitch(ord_w3) : ord_first;
            ord_cnt2  <= ord_bst_en ? w3_pitch(ord_w3) : ord_first;
            ord_pin_q <= ord_pin_en;
            ord_bs_n  <= 1'b0;
            ord_w16   <= ord_w16_c;
            //a longword on a 16-bit port splits into two bus cycles (MS first);
            //narrower accesses run one cycle on the half their address selects
            ord_a1    <= (ord_w16_c && I_BUS.req_size == 2'd2) ? 1'b0 : fa[1];
            ord_second<= ord_w16_c && (I_BUS.req_size == 2'd2);
            ord_addr  <= fa;
            ord_write <= I_BUS.req_write;
            ord_burst <= I_BUS.req_burst;
            ord_size  <= I_BUS.req_size;
            ord_wstrb <= I_BUS.req_wstrb;
            ord_wdata <= I_BUS.req_wdata;
            ord_area  <= fe_area;
        end
        else if(i_CEN && fe_rsp_done && owner_q == OWN_GEN) begin
            ord_busy <= 1'b0;                           //either completion path closes it
            ord_done <= 1'b0;
            ord_run  <= 1'b0;
        end
        else if(i_BCEN && ord_busy && !ord_run) begin
            ord_run  <= 1'b1;                           //grid-align an off-grid accept
        end
        else if(i_BCEN && ord_busy && !ord_done) begin
            ord_bs_n <= 1'b1;                           //BS covers each sub's first cycle
            if(ord_cnt != 4'd0)                  ord_cnt  <= ord_cnt - 4'd1;
            else if(!ord_pin_q || i_WAIT_n) begin       //WAIT stretches enabled areas
                if(ord_w16) begin                       //16-bit port: D15-D0 per half
                    if(!ord_a1) ord_data[31:16] <= i_D_I[15:0];
                    else        ord_data[15:0]  <= i_D_I[15:0];
                end
                else ord_data <= i_D_I;                 //32-bit port: one live sample
                if(ord_second) begin                    //advance to the LS half
                    ord_second <= 1'b0;
                    ord_a1     <= 1'b1;
                    ord_cnt    <= ord_cnt2;             //a fresh bus cycle, fresh waits
                    ord_bs_n   <= 1'b0;
                end
                else ord_done <= 1'b1;
            end
        end
    end
end



///////////////////////////////////////////////////////////
//////  Front-End Sequencing (core rate)
////

//register-file read word, muxed on the live address, captured at the accept edge
logic   [15:0]  reg_rd_w;
always_comb begin
    if(fa[7:4] == 4'h5) reg_rd_w = mcscr[fa[3:1]];
    else case(fa[7:1])
        7'h30:   reg_rd_w = bcr1;               //0xFFFFFF60
        7'h31:   reg_rd_w = bcr2;               //0xFFFFFF62
        7'h32:   reg_rd_w = wcr1;               //0xFFFFFF64
        7'h33:   reg_rd_w = wcr2;               //0xFFFFFF66
        7'h34:   reg_rd_w = mcr;                //0xFFFFFF68
        7'h36:   reg_rd_w = pcr;                //0xFFFFFF6C
        7'h37:   reg_rd_w = {8'd0, rtcsr};      //0xFFFFFF6E
        7'h38:   reg_rd_w = {8'd0, rtcnt};      //0xFFFFFF70
        7'h39:   reg_rd_w = {8'd0, rtcor};      //0xFFFFFF72
        7'h3A:   reg_rd_w = {6'd0, rfcr};       //0xFFFFFF74
        default: reg_rd_w = 16'd0;              //reserved offsets read 0
    endcase
end

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        owner_q    <= OWN_GEN;
        loc_rsp_v  <= 1'b0;
        loc_rsp_d  <= 32'd0;
        sd_rd_wait <= 1'b0;
        sd_rd_beat <= 2'd0;
        sd_wr_ack  <= 1'b0;
        b_rd_act   <= 1'b0;
        b_wr_act   <= 1'b0;
    end
    else begin if(i_CEN) begin
        if(fe_acc) begin
            owner_q <= fe_gen ? OWN_GEN : fe_local ? OWN_LOC : fe_pbus ? OWN_PBS : OWN_SDR;

            if(fe_local) begin                  //register/dummy: one-beat response
                loc_rsp_v <= 1'b1;
                loc_rsp_d <= fe_reg ? {2{reg_rd_w}} : 32'd0;
            end

            if(fe_eng) begin
                if(I_BUS.req_write || fe_sdmr) sd_wr_ack  <= 1'b1;  //posted
                else begin
                    sd_rd_wait <= 1'b1;
                    sd_rd_beat <= fa[3:2];
                end
                if(fe_sdram && !fe_sdmr && I_BUS.req_burst && fe_b_head) begin
                    b_rd_act <= !I_BUS.req_write;
                    b_wr_act <=  I_BUS.req_write;
                end
                //last drain beat closes the write burst
                if(b_wr_act && fa[3:2] == 2'b11) b_wr_act <= 1'b0;
            end
        end

        if(fe_rsp_done) begin
            loc_rsp_v <= 1'b0;
            sd_wr_ack <= 1'b0;
            if(sd_rd_wait) begin
                sd_rd_wait <= 1'b0;
                //last fill beat closes the read burst
                if(b_rd_act && sd_rd_beat == 2'd3) b_rd_act <= 1'b0;
            end
        end
    end end
end

//engine op slot: set at the accept edge of a head/single op, cleared when the
//engine leaves E_IDLE with it. Continuation write beats only top up the buffer.
logic           eng_op_write, eng_op_burst, eng_op_mrs, eng_op_lock, eng_cs3;
logic   [31:0]  eng_addr;

//the cache's drain is interruptible (it revisits S_IDLE between beats): a
//blocked ordinary request makes the engine YIELD mid-burst-write, so a
//resumed drain beat must START a fresh write op from its own beat index
wire            eng_wr_pend  = (eng_go && eng_op_write) ||
                               (e_write && (est == E_BA_DISP || est == E_PRE_WAIT ||
                                            est == E_ACTV    || est == E_RCD || est == E_WR));
wire            fe_eng_start = fe_acc_eng && (!fe_b_cont ||
                                              (I_BUS.req_write && !eng_wr_pend));
wire            eng_start_tk;                   //engine takes the op (BCEN edge)

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) eng_go <= 1'b0;
    else begin if(i_CEN) begin
        if(fe_eng_start)      eng_go <= 1'b1;
        else if(eng_start_tk) eng_go <= 1'b0;   //i_BCEN implies i_CEN
    end end
end

always_ff @(posedge i_CLK) begin if(i_CEN) begin
    if(fe_eng_start) begin
        eng_op_write <= I_BUS.req_write && !fe_sdmr;
        eng_op_burst <= I_BUS.req_burst && !fe_sdmr;
        eng_op_mrs   <= fe_sdmr;
        eng_op_lock  <= I_BUS.req_lock;
        eng_cs3      <= fe_sdmr ? (fa[13:12] == 2'b10) : (fe_area == 3'd3);  //0xFFFFExxx = area 3
        eng_addr     <= fa;
    end
end end

//write line buffer: a beat lands in slot addr[3:2] (singles use their own slot);
//cleared when the engine finishes the write op
wire            eng_wr_done;
always_ff @(posedge i_CLK) begin
    if(i_CEN && fe_acc_eng && I_BUS.req_write && !fe_sdmr) begin
        wr_buf [fa[3:2]] <= I_BUS.req_wdata;
        wr_strb[fa[3:2]] <= I_BUS.req_wstrb;
    end
end
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) wr_v <= 4'd0;
    else begin if(i_CEN) begin
        if(fe_acc_eng && I_BUS.req_write && !fe_sdmr) wr_v[fa[3:2]] <= 1'b1;
        else if(eng_wr_done)                          wr_v <= 4'd0;
    end end
end



///////////////////////////////////////////////////////////
//////  P Bus Bridge (sessions 3-4: TMU / RTC / PFC-port register tier)
////

/*
    The P-bus peripherals live behind the BSC in the block diagram (Fig 1.1
    p.6; Appendix B marks their registers "P"). Same contract as the I-bus-2
    BRIDGE: IDLE -> ACCESS -> RESP, write payloads right-justified by the
    strobes, read values lane-replicated by size. Read latency 2 cycles
    accept-to-response - handshake latency is allowed on this tier only
    (user latency law). Front-end reset domain: a dying access must not
    wedge the bus; the slaves keep their own reset rules.
*/

localparam logic [1:0] P_IDLE = 2'd0, P_ACC = 2'd1, P_RSP = 2'd2;
localparam logic [1:0] PW_TMU = 2'd0, PW_RTC = 2'd1, PW_PRT = 2'd2;

logic   [1:0]   pbs_state;
logic   [1:0]   pbs_win_q;          //captured window select (PW_*)
logic   [7:0]   pbs_addr_q;         //byte address within the window
logic           pbs_we_q;
logic   [1:0]   pbs_size_q;
logic   [31:0]  pbs_wdata_q;        //right-justified write payload

assign  pbs_ready = (pbs_state == P_IDLE);
assign  pbs_rsp_v = (pbs_state == P_RSP);

//right-justify off the strobes (ibus_bridge pattern: the strobed lane IS the
//payload lane in either endianness; misaligned accesses fault in MA)
logic   [31:0]  pbs_wdata_rj;
always_comb begin
    unique case(I_BUS.req_wstrb)
        4'b1000: pbs_wdata_rj = {24'd0, I_BUS.req_wdata[31:24]};
        4'b0100: pbs_wdata_rj = {24'd0, I_BUS.req_wdata[23:16]};
        4'b0010: pbs_wdata_rj = {24'd0, I_BUS.req_wdata[15:8]};
        4'b0001: pbs_wdata_rj = {24'd0, I_BUS.req_wdata[7:0]};
        4'b1100: pbs_wdata_rj = {16'd0, I_BUS.req_wdata[31:16]};
        4'b0011: pbs_wdata_rj = {16'd0, I_BUS.req_wdata[15:0]};
        default: pbs_wdata_rj = I_BUS.req_wdata;                //long (or read: don't-care)
    endcase
end

//selected slave read, replicated onto the lanes so the pipe's load aligner
//picks the correct byte/halfword from any naturally aligned offset
wire    [31:0]  pbs_sel_rdata = (pbs_win_q == PW_TMU) ? REG_TMU.rdata :
                                (pbs_win_q == PW_RTC) ? REG_RTC.rdata : REG_PORT.rdata;
logic   [31:0]  pbs_rdata_rep;
always_comb begin
    unique case(pbs_size_q)
        2'd0:    pbs_rdata_rep = {4{pbs_sel_rdata[7:0]}};
        2'd1:    pbs_rdata_rep = {2{pbs_sel_rdata[15:0]}};
        default: pbs_rdata_rep = pbs_sel_rdata;
    endcase
end

always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) begin
        pbs_state   <= P_IDLE;
        pbs_win_q   <= PW_TMU;
        pbs_addr_q  <= 8'd0;
        pbs_we_q    <= 1'b0;
        pbs_size_q  <= 2'd0;
        pbs_wdata_q <= 32'd0;
        pbs_rdata_q <= 32'd0;
    end
    else begin if(i_CEN) begin
        unique case(pbs_state)
            P_IDLE: begin
                if(fe_acc && fe_pbus) begin                     //accept edge: latch everything
                    pbs_win_q   <= fe_tmu ? PW_TMU : fe_rtc ? PW_RTC : PW_PRT;
                    pbs_addr_q  <= fa[7:0];
                    pbs_we_q    <= I_BUS.req_write;
                    pbs_size_q  <= I_BUS.req_size;
                    pbs_wdata_q <= pbs_wdata_rj;
                    pbs_state   <= P_ACC;
                end
            end
            P_ACC: begin                                        //slave strobe: sample the read
                pbs_rdata_q <= pbs_rdata_rep;
                pbs_state   <= P_RSP;
            end
            default: begin                                      //P_RSP
                if(I_BUS.rsp_ready) pbs_state <= P_IDLE;
            end
        endcase
    end end
end

//IBus_2 drive: shared payload fans to all slaves; only the captured window
//sees its strobe
assign  REG_TMU.stb    = (pbs_state == P_ACC) && (pbs_win_q == PW_TMU);
assign  REG_RTC.stb    = (pbs_state == P_ACC) && (pbs_win_q == PW_RTC);
assign  REG_PORT.stb   = (pbs_state == P_ACC) && (pbs_win_q == PW_PRT);

assign  REG_TMU.we     = pbs_we_q;
assign  REG_TMU.size   = pbs_size_q;
assign  REG_TMU.addr   = pbs_addr_q;
assign  REG_TMU.wdata  = pbs_wdata_q;
assign  REG_RTC.we     = pbs_we_q;
assign  REG_RTC.size   = pbs_size_q;
assign  REG_RTC.addr   = pbs_addr_q;
assign  REG_RTC.wdata  = pbs_wdata_q;
assign  REG_PORT.we    = pbs_we_q;
assign  REG_PORT.size  = pbs_size_q;
assign  REG_PORT.addr  = pbs_addr_q;
assign  REG_PORT.wdata = pbs_wdata_q;



///////////////////////////////////////////////////////////
//////  SDRAM Engine (i_BCEN domain - the natural-latency machine)
////

/*
    Commands on {CS,RAS,CAS,WE} (p.276); one command per 20 ns bus cycle.
    Pin registers update at BCEN edges; the PCB clock rises mid-cycle (180
    degrees), giving 10 ns setup/hold either side. Spacing counts (TPC after
    a precharge, RCD after ACTV) are command-to-command distances: value 1
    means back-to-back commands, so wait states = value - 1. Duration counts
    (Tpc/Trwl tails, refresh lockout, self-refresh exit) hold the engine for
    exactly that many cycles.

    Auto-precharge mode (RASD=0): ACTV -> rcd -> READ.../WRIT... with
    auto-precharge on the last beat, then the Tpc tail (figs 10.14-10.18).
    Bank-active mode (RASD=1): commands without precharge; per-bank open-row
    table; PRE -> tpc -> ACTV on a row miss (figs 10.19-10.24); one Tnop
    before a row-hit READ when CL=1 (DQM two-cycle lead, p.290).
*/

typedef enum logic [4:0] {
    E_IDLE,
    E_MRS_PALL, E_MRS_WAIT, E_MRS_SET,  E_MRS_MRD,      //fig 10.28
    E_BA_DISP,  E_PRE_WAIT,                             //bank-active row lookup / precharge gap
    E_ACTV,     E_RCD,                                  //row open + RAS-CAS gap (also the Tnop)
    E_RD,       E_RD_DRAIN, E_RD_TPC,                   //READ beats + CL landing + Tpc tail
    E_WR,       E_WR_TRWL,  E_WR_TPC,                   //WRIT beats + Trwl + Tpc tails
    E_WR_YIELD,                                         //interrupted burst: Trwl then PRE
    E_BRQ_PALL, E_BRQ_WAIT,                             //close banks before a bus grant
    E_REF_PALL, E_REF_WAIT, E_REF_CMD,  E_REF_LOCK,     //fig 10.26
    E_SLF_PALL, E_SLF_WAIT, E_SLF_CMD,  E_SLF,  E_SLF_EXIT  //fig 10.27
} eng_state_t;

//shared-bus pin registers of the SDRAM engine (merged with the ordinary
//controller's drive at the physical pin mux below)
logic           sd_cs2_n, sd_cs3_n;
logic           sd_rasl_n, sd_rasu_n, sd_casl_n, sd_casu_n;
logic           sd_cmdwe_n;         //the shared RD/WR pin, SDRAM command view
logic   [25:0]  sd_a;               //chip A25-A0 view (device A10:A0 sits on A12:A2)
logic   [3:0]   sd_dqm;
logic           sd_bs_n, sd_cke;
logic   [31:0]  sd_dq_o;
logic           sd_dq_oe;

eng_state_t     est;
logic   [3:0]   ecnt;               //shared wait counter
logic   [1:0]   ebeat;              //command beat within a burst
logic   [2:0]   rd_need;            //data landings still expected
logic   [2:0]   wr_recov;           //bank-active write recovery (tWR) before a precharge
logic           e_write, e_burst, e_cs3, e_lock_hold;
logic   [31:0]  e_addr;
logic           rst_z;              //front-end reset, sampled for the write-abort escape

//read-latency pipeline: slot n = a READ issued n+1 bus cycles ago; the slot
//at CL-1 leaving the pipe means DQ carries that beat's data at this edge
logic   [2:0]   rdp_v;
logic   [1:0]   rdp_b [0:2];
wire    [1:0]   e_cl    = e_cs3 ? cl_a3 : cl_a2;
wire            rd_lat  = rdp_v[e_cl - 2'd1];
wire    [1:0]   rd_latb = rdp_b[e_cl - 2'd1];

//bank-active open-row table (device: 4 banks)
logic   [3:0]   ba_v;
logic   [10:0]  ba_row [0:3];
wire    [1:0]   e_bank   = e_addr[22:21];
wire    [10:0]  e_row    = e_addr[20:10];
wire            row_hit  = ba_v[e_bank] && (ba_row[e_bank] == e_row);
wire            row_conf = ba_v[e_bank] && (ba_row[e_bank] != e_row);

//bus arbitration (p.320): BREQ is granted only with the bus drained (engine
//idle or parked in self-refresh, no ordinary cycle, no queued op) and all
//bank-active rows closed (E_BRQ_PALL runs first so the next master - and
//the Micron model - meets precharged banks). While requested or granted, no
//new external cycle is accepted; refresh requests stay latched and run on
//regain. Register/local accesses keep working - only the pins are released.
logic           breq_z, breq_zz;    //2FF sync of the async pin
logic           bus_rel;
always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) begin
        breq_z  <= 1'b0;
        breq_zz <= 1'b0;
    end
    else begin if(i_CEN) begin
        breq_z  <= ~i_BREQ_n;
        breq_zz <= breq_z;
    end end
end
wire            brq      = breq_zz;
assign          bus_held = brq | bus_rel;

always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) bus_rel <= 1'b0;
    else begin if(i_BCEN) begin
        if(!brq)                                       bus_rel <= 1'b0;
        else if((est == E_IDLE || est == E_SLF) &&
                !ord_busy && !eng_go && ba_v == 4'd0 &&
                !e_lock_hold && !fe_lock_hold)         bus_rel <= 1'b1;  //never split a TAS pair (p.320)
    end end
end

assign  o_BACK_n = ~bus_rel;
assign  o_BUS_OE = ~bus_rel;

//refresh arbitration: a pending request completes, then refresh wins over a
//NEW op; a locked RMW pair (TAS) is never split by a refresh
logic           ref_req;
wire            ref_ok = ref_req && rfsh && !rmode && !e_lock_hold;
wire            self_req = rfsh && rmode;               //MCR.RMODE level (p.300)

assign  eng_start_tk = (est == E_IDLE) && i_BCEN && eng_go && !ref_ok && !self_req;
//E_SLF is a PARKED state: the SDRAM sits in self-refresh on CKE alone and
//the shared bus is free for ordinary cycles (a new SDRAM op is still held
//off by self_active). Everything else counts as bus ownership.
assign  eng_busy     = (est != E_IDLE) && (est != E_SLF);

wire    [1:0]   wr_slot   = e_burst ? ebeat : e_addr[3:2];
assign  eng_wr_done = (est == E_WR) && i_BCEN && wr_v[wr_slot] &&
                      (!e_burst || ebeat == 2'd3);

always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) begin
        est   <= E_IDLE;
        ecnt  <= 4'd0;
        ebeat <= 2'd0;
        rd_need <= 3'd0;
        wr_recov <= 3'd0;
        e_write <= 1'b0; e_burst <= 1'b0; e_cs3 <= 1'b0; e_lock_hold <= 1'b0;
        e_addr  <= 32'd0;
        rst_z   <= 1'b0;
        rdp_v <= 3'd0;
        rdp_b[0] <= 2'd0; rdp_b[1] <= 2'd0; rdp_b[2] <= 2'd0;
        ba_v  <= 4'd0;
        ba_row[0] <= 11'd0; ba_row[1] <= 11'd0; ba_row[2] <= 11'd0; ba_row[3] <= 11'd0;
        sd_cs2_n  <= 1'b1; sd_cs3_n  <= 1'b1;
        sd_rasl_n <= 1'b1; sd_rasu_n <= 1'b1;
        sd_casl_n <= 1'b1; sd_casu_n <= 1'b1;
        sd_cmdwe_n <= 1'b1;
        sd_a      <= 26'd0;
        sd_dqm    <= 4'b1111;
        sd_bs_n   <= 1'b1;
        sd_cke    <= 1'b1;
        sd_dq_o   <= 32'd0; sd_dq_oe <= 1'b0;
    end
    else begin if(i_BCEN) begin
        rst_z <= i_RST_n;

        //every cycle defaults to NOP/deselect; op arms below override.
        //WE/DQM idles HIGH (shared with the ordinary write strobes); a read
        //op holds all lanes low from the row open (2-cycle DQM lead, p.290)
        sd_cs2_n  <= 1'b1; sd_cs3_n  <= 1'b1;
        sd_rasl_n <= 1'b1; sd_rasu_n <= 1'b1;
        sd_casl_n <= 1'b1; sd_casu_n <= 1'b1;
        sd_cmdwe_n <= 1'b1;
        sd_dqm    <= (!e_write && (est == E_ACTV || est == E_RCD ||
                                   est == E_RD   || est == E_RD_DRAIN)) ? 4'b0000 : 4'b1111;
        sd_bs_n   <= 1'b1;
        sd_dq_oe  <= 1'b0;

        //read-latency pipeline always shifts; E_RD refills slot 0
        rdp_v    <= {rdp_v[1:0], 1'b0};
        rdp_b[1] <= rdp_b[0];
        rdp_b[2] <= rdp_b[1];

        //bank-active write recovery drains every cycle (E_WR reloads it)
        if(wr_recov != 3'd0) wr_recov <= wr_recov - 3'd1;

        case(est)
        E_IDLE: begin
            if(ord_busy) begin end                      //an ordinary cycle owns the bus
            else if(brq && ba_v != 4'd0) est <= E_BRQ_PALL; //close rows, then grant
            else if(ref_ok && !bus_held)   est <= E_REF_PALL;
            else if(self_req && !bus_held) est <= E_SLF_PALL;
            else if(eng_go) begin
                e_write <= eng_op_write;
                e_burst <= eng_op_burst;
                e_cs3   <= eng_cs3;
                e_addr  <= eng_addr;
                //a locked read opens the refresh-deferral window (TAS pair)
                if(eng_op_lock && !eng_op_write) e_lock_hold <= 1'b1;
                ebeat   <= eng_addr[3:2];           //bursts resume from their own beat
                rd_need <= eng_op_burst ? 3'd4 : 3'd1;
                if(eng_op_mrs)  est <= E_MRS_PALL;
                else if(rasd)   est <= E_BA_DISP;
                else            est <= E_ACTV;
            end
        end

        /* mode register set: PALL -> tpc gap -> MRS -> 4 cycles (fig 10.28) */
        E_MRS_PALL: begin
            if(wr_recov == 3'd0) begin                  //tWR guard (bank-active writes)
                sd_cs2_n <= ~a2_sdram; sd_cs3_n <= ~a3_sdram;   //PALL, all devices
                sd_rasl_n <= 1'b0; sd_rasu_n <= 1'b0;           //U+L together (p.276)
                sd_cmdwe_n <= 1'b0;
                sd_a[12] <= 1'b1;                               //chip A12 = device A10
                ba_v <= 4'd0;
                if(t_tpc == 3'd1) est <= E_MRS_SET;
                else begin ecnt <= {1'b0, t_tpc} - 4'd1; est <= E_MRS_WAIT; end
            end
        end
        E_MRS_WAIT: begin
            if(ecnt <= 4'd1) est <= E_MRS_SET;
            else             ecnt <= ecnt - 4'd1;
        end
        E_MRS_SET: begin
            sd_cs2_n <= e_cs3; sd_cs3_n <= ~e_cs3;
            sd_rasl_n <= 1'b0; sd_rasu_n <= 1'b0;       //MRS drives U+L (p.276)
            sd_casl_n <= 1'b0; sd_casu_n <= 1'b0;
            sd_cmdwe_n <= 1'b0;
            sd_a     <= e_addr[25:0];                   //mode value rides A12:A2 (p.252)
            ecnt <= 4'd4;                               //TMw1-4 covers tMRD (fig 10.28)
            est  <= E_MRS_MRD;
        end
        E_MRS_MRD: begin
            if(ecnt <= 4'd1) est <= E_IDLE;
            else             ecnt <= ecnt - 4'd1;
        end

        /* bank-active dispatch (RASD=1, pp.289-290): row lookup, then either the
           command beats directly (hit), or PRE -> tpc gap -> ACTV (conflict) */
        E_BA_DISP: begin
            if(row_hit) begin
                if(!e_write && e_cl == 2'd1) begin      //Tnop: DQM two-cycle lead
                    ecnt <= 4'd1;
                    est  <= E_RCD;
                end
                else est <= e_write ? E_WR : E_RD;
            end
            else if(row_conf) begin
                if(wr_recov == 3'd0) begin              //tWR guard before the precharge
                    sd_cs2_n <= e_cs3; sd_cs3_n <= ~e_cs3;
                    if(e_addr[25]) sd_rasu_n <= 1'b0;       //PRE this bank
                    else           sd_rasl_n <= 1'b0;
                    sd_cmdwe_n  <= 1'b0;
                    sd_a[12]    <= 1'b0;
                    sd_a[14:13] <= e_bank;
                    ba_v[e_bank] <= 1'b0;
                    if(t_tpc == 3'd1) est <= E_ACTV;
                    else begin ecnt <= {1'b0, t_tpc} - 4'd1; est <= E_PRE_WAIT; end
                end
            end
            else est <= E_ACTV;                         //bank idle: activate
        end
        E_PRE_WAIT: begin
            if(ecnt <= 4'd1) est <= E_ACTV;
            else             ecnt <= ecnt - 4'd1;
        end

        /* row activate + RAS-CAS gap (Tr, Trw; p.281) */
        E_ACTV: begin
            sd_cs2_n <= e_cs3; sd_cs3_n <= ~e_cs3;
            if(e_addr[25]) sd_rasu_n <= 1'b0;           //ACTV (RAS3L/U by 32MB half)
            else           sd_rasl_n <= 1'b0;
            sd_a     <= {e_addr[25:15], e_bank, e_row, e_addr[1:0]};    //row on A12:A2
            if(rasd) begin
                ba_v[e_bank]   <= 1'b1;
                ba_row[e_bank] <= e_row;
            end
            if(t_rcd == 3'd1) est <= e_write ? E_WR : E_RD;
            else begin ecnt <= {1'b0, t_rcd} - 4'd1; est <= E_RCD; end
        end
        E_RCD: begin
            if(ecnt <= 4'd1) est <= e_write ? E_WR : E_RD;
            else             ecnt <= ecnt - 4'd1;
        end

        /* READ/READA beats, one per cycle (Tc1-Tc4; READA unless bank-active) */
        E_RD: begin
            sd_cs2_n <= e_cs3; sd_cs3_n <= ~e_cs3;
            if(e_addr[25]) sd_casu_n <= 1'b0;           //READ / READA
            else           sd_casl_n <= 1'b0;
            //column phase: chip A12 = auto-precharge flag, A9:A2 = column
            sd_a     <= {e_addr[25:15], e_bank, (!rasd && (!e_burst || ebeat == 2'd3)),
                         e_addr[11:10], e_addr[9:4], ebeat, e_addr[1:0]};
            sd_bs_n  <= 1'b0;
            rdp_v[0]   <= 1'b1;
            rdp_b[0]   <= ebeat;
            if(!e_burst || ebeat == 2'd3) est <= E_RD_DRAIN;
            else                          ebeat <= ebeat + 2'd1;
        end
        E_RD_DRAIN: begin                               //exit at the last CL landing edge
            if(rd_need == 3'd1 && rd_lat) begin
                if(rasd) est <= E_IDLE;                 //no precharge tail in bank-active
                else begin ecnt <= {1'b0, t_tpc}; est <= E_RD_TPC; end
            end
        end
        E_RD_TPC: begin
            if(ecnt <= 4'd1) est <= E_IDLE;
            else             ecnt <= ecnt - 4'd1;
        end

        /* WRIT/WRITA beats: data rides the command cycle (p.285). A beat whose
           posted data has not landed yet is a NOP stall (stretches, never breaks) */
        E_WR: begin
            if(!rst_z || (!wr_v[wr_slot] && I_BUS.req_valid && fe_gen)) begin
                //beats will never arrive (manual reset) or a blocked ordinary
                //request needs the bus: close the interrupted burst. Bank-
                //active just releases (banks stay open); auto-precharge must
                //Trwl then PRE the bank the plain WRITs left active
                if(rasd) begin
                    wr_recov <= t_trwl;
                    ecnt <= 4'd1;
                    est  <= E_WR_TRWL;
                end
                else begin
                    ecnt <= {1'b0, t_trwl};
                    est  <= E_WR_YIELD;
                end
            end
            else if(wr_v[wr_slot]) begin
                sd_cs2_n <= e_cs3; sd_cs3_n <= ~e_cs3;
                if(e_addr[25]) sd_casu_n <= 1'b0;       //WRIT / WRITA
                else           sd_casl_n <= 1'b0;
                sd_cmdwe_n <= 1'b0;
                sd_a     <= {e_addr[25:15], e_bank, (!rasd && (!e_burst || ebeat == 2'd3)),
                             e_addr[11:10], e_addr[9:4], ebeat, e_addr[1:0]};
                sd_dq_o  <= wr_buf [wr_slot];
                sd_dqm   <= ~wr_strb[wr_slot];
                sd_dq_oe <= 1'b1;
                sd_bs_n  <= 1'b0;
                if(!e_burst || ebeat == 2'd3) begin
                    e_lock_hold <= 1'b0;                //locked pair completed
                    //bank-active: no Trwl/Tpc tail (p.289), but the bus stays
                    //owned for the WRIT command's own cycle (one E_WR_TRWL
                    //beat), else the pin mux flips mid-command; tWR before a
                    //precharge is guarded by wr_recov instead
                    if(rasd) wr_recov <= t_trwl;
                    ecnt <= rasd ? 4'd1 : {1'b0, t_trwl};
                    est  <= E_WR_TRWL;
                end
                else ebeat <= ebeat + 2'd1;
            end
        end
        E_WR_TRWL: begin
            if(ecnt <= 4'd1) begin
                if(rasd) est <= E_IDLE;                 //ownership beat served
                else begin
                    ecnt <= {1'b0, t_tpc};
                    est  <= E_WR_TPC;
                end
            end
            else ecnt <= ecnt - 4'd1;
        end
        E_WR_TPC: begin
            if(ecnt <= 4'd1) est <= E_IDLE;
            else             ecnt <= ecnt - 4'd1;
        end
        E_WR_YIELD: begin                               //close an interrupted AP burst
            if(ecnt <= 4'd1) begin
                sd_cs2_n <= e_cs3; sd_cs3_n <= ~e_cs3;
                if(e_addr[25]) sd_rasu_n <= 1'b0;       //PRE the bank the WRITs opened
                else           sd_rasl_n <= 1'b0;
                sd_cmdwe_n  <= 1'b0;
                sd_a[12]    <= 1'b0;
                sd_a[14:13] <= e_bank;
                ecnt <= {1'b0, t_tpc};
                est  <= E_WR_TPC;
            end
            else ecnt <= ecnt - 4'd1;
        end

        /* bus-grant precharge: bank-active rows must close before another
           master (or the model) touches the SDRAM */
        E_BRQ_PALL: begin
            if(wr_recov == 3'd0) begin                  //tWR guard as everywhere
                sd_cs2_n <= ~a2_sdram; sd_cs3_n <= ~a3_sdram;
                sd_rasl_n <= 1'b0; sd_rasu_n <= 1'b0;   //PALL, all devices
                sd_cmdwe_n <= 1'b0;
                sd_a[12] <= 1'b1;
                ba_v <= 4'd0;
                if(t_tpc == 3'd1) est <= E_IDLE;
                else begin ecnt <= {1'b0, t_tpc} - 4'd1; est <= E_BRQ_WAIT; end
            end
        end
        E_BRQ_WAIT: begin
            if(ecnt <= 4'd1) est <= E_IDLE;
            else             ecnt <= ecnt - 4'd1;
        end

        /* auto-refresh: PALL -> tpc gap -> REF -> tras+tpc lockout (fig 10.26) */
        E_REF_PALL: begin
            if(wr_recov == 3'd0) begin                  //tWR guard (bank-active writes)
                sd_cs2_n <= ~a2_sdram; sd_cs3_n <= ~a3_sdram;
                sd_rasl_n <= 1'b0; sd_rasu_n <= 1'b0;
                sd_cmdwe_n <= 1'b0;
                sd_a[12] <= 1'b1;
                ba_v <= 4'd0;                           //refresh closes all banks (p.290)
                if(t_tpc == 3'd1) est <= E_REF_CMD;
                else begin ecnt <= {1'b0, t_tpc} - 4'd1; est <= E_REF_WAIT; end
            end
        end
        E_REF_WAIT: begin
            if(ecnt <= 4'd1) est <= E_REF_CMD;
            else             ecnt <= ecnt - 4'd1;
        end
        E_REF_CMD: begin
            sd_cs2_n <= ~a2_sdram; sd_cs3_n <= ~a3_sdram;
            sd_rasl_n <= 1'b0; sd_rasu_n <= 1'b0;       //REF drives U+L (p.276)
            sd_casl_n <= 1'b0; sd_casu_n <= 1'b0;
            ecnt <= {1'b0, t_tras} + {1'b0, t_tpc} - 4'd1;
            est  <= E_REF_LOCK;
        end
        E_REF_LOCK: begin                               //no command for TRAS+TPC (p.297)
            if(ecnt <= 4'd1) est <= E_IDLE;
            else             ecnt <= ecnt - 4'd1;
        end

        /* self-refresh: REF entry with CKE low, held until RMODE clears (p.300) */
        E_SLF_PALL: begin
            if(wr_recov == 3'd0) begin                  //tWR guard (bank-active writes)
                sd_cs2_n <= ~a2_sdram; sd_cs3_n <= ~a3_sdram;
                sd_rasl_n <= 1'b0; sd_rasu_n <= 1'b0;
                sd_cmdwe_n <= 1'b0;
                sd_a[12] <= 1'b1;
                ba_v <= 4'd0;
                if(t_tpc == 3'd1) est <= E_SLF_CMD;
                else begin ecnt <= {1'b0, t_tpc} - 4'd1; est <= E_SLF_WAIT; end
            end
        end
        E_SLF_WAIT: begin
            if(ecnt <= 4'd1) est <= E_SLF_CMD;
            else             ecnt <= ecnt - 4'd1;
        end
        E_SLF_CMD: begin
            sd_cs2_n <= ~a2_sdram; sd_cs3_n <= ~a3_sdram;
            sd_rasl_n <= 1'b0; sd_rasu_n <= 1'b0;       //SELF = REF with CKE low
            sd_casl_n <= 1'b0; sd_casu_n <= 1'b0;
            sd_cke   <= 1'b0;
            est <= E_SLF;
        end
        E_SLF: begin
            if(!self_req) begin                         //software cleared RMODE
                sd_cke <= 1'b1;
                ecnt <= t_tpc_slf;                      //exit wait 2/5/8/11 (p.245)
                est  <= E_SLF_EXIT;
            end
            else sd_cke <= 1'b0;
        end
        E_SLF_EXIT: begin
            if(ecnt <= 4'd1) est <= E_IDLE;
            else             ecnt <= ecnt - 4'd1;
        end

        default: est <= E_IDLE;
        endcase

        //read-data landing: the slot leaving the CL pipeline carries this edge's DQ
        if(rd_lat) rd_need <= rd_need - 3'd1;


    end end
end

always_ff @(posedge i_CLK) begin
    if(i_BCEN && rd_lat) rd_buf[rd_latb] <= i_D_I;
end

//word-valid handoff: engine sets per landed beat; the head accept of the next
//read clears (the engine cannot land words before its first command)
always_ff @(posedge i_CLK or negedge i_RST_n) begin
    if(!i_RST_n) wv <= 4'd0;
    else begin
        if(i_CEN && fe_acc_eng && !I_BUS.req_write && !fe_sdmr && !fe_b_cont) wv <= 4'd0;
        else if(i_BCEN && rd_lat) wv[rd_latb] <= 1'b1;
    end
end



///////////////////////////////////////////////////////////
//////  Refresh Timer (i_BCEN domain, CKIO-based; pp.253-257)
////

//prescaler taps: CKS 001=/4 010=/16 011=/64 100=/256 101=/1024 110=/2048 111=/4096
logic   [11:0]  presc;
logic           presc_tick;
always_comb begin
    case(rtcsr[5:3])
        3'b001:  presc_tick = (presc[1:0]  == 2'b11);
        3'b010:  presc_tick = (presc[3:0]  == 4'hF);
        3'b011:  presc_tick = (presc[5:0]  == 6'h3F);
        3'b100:  presc_tick = (presc[7:0]  == 8'hFF);
        3'b101:  presc_tick = (presc[9:0]  == 10'h3FF);
        3'b110:  presc_tick = (presc[10:0] == 11'h7FF);
        3'b111:  presc_tick = (presc[11:0] == 12'hFFF);
        default: presc_tick = 1'b0;             //clock input disabled
    endcase
end

always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) presc <= 12'd0;
    else begin if(i_BCEN) begin
        presc <= presc + 12'd1;
    end end
end

wire            rt_match  = presc_tick && (rtcnt == rtcor);
wire            ref_done  = (est == E_REF_CMD);         //the cycle the REF issues
wire    [9:0]   rfcr_lim  = rtcsr[0] ? 10'd511 : 10'd1023;  //LMTS (p.255)



///////////////////////////////////////////////////////////
//////  Register Writes (core rate; POR-only reset, p.228)
////

/*
    All registers are 16-bit word-access; the strobed half of the 32-bit bus is
    extracted right-justified (bridge pattern). The refresh group demands the
    write keys of fig 10.5: RTCSR/RTCNT/RTCOR = 0xA5 upper byte, RFCR =
    6'b101001 upper bits; wrong key or size is silently ignored.
*/

wire    [15:0]  wr_w     = I_BUS.req_wstrb[3] ? I_BUS.req_wdata[31:16] : I_BUS.req_wdata[15:0];
wire            wr_reg   = fe_acc && fe_reg && I_BUS.req_write && (I_BUS.req_size == 2'd1);
wire            key_a5   = (wr_w[15:8] == 8'hA5);
wire            key_rfcr = (wr_w[15:10] == 6'b101001);

wire            wr_rtcsr = wr_reg && (fa[7:1] == 7'h37) && key_a5;
wire            wr_rtcnt = wr_reg && (fa[7:1] == 7'h38) && key_a5;
wire            wr_rtcor = wr_reg && (fa[7:1] == 7'h39) && key_a5;
wire            wr_rfcr  = wr_reg && (fa[7:1] == 7'h3A) && key_rfcr;

always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) begin
        bcr1  <= {4'd0, BIG_ENDIAN, 11'd0};     //H'0000 + the ENDIAN pin reflection (p.233)
        bcr2  <= 16'h3FF0;
        wcr1  <= 16'h3FF3;
        wcr2  <= 16'hFFFF;
        mcr   <= 16'h0000;
        pcr   <= 16'h0000;
        rtcor <= 8'd0;
        mcscr[0] <= 16'd0; mcscr[1] <= 16'd0; mcscr[2] <= 16'd0; mcscr[3] <= 16'd0;
        mcscr[4] <= 16'd0; mcscr[5] <= 16'd0; mcscr[6] <= 16'd0; mcscr[7] <= 16'd0;
    end
    else begin if(i_CEN) begin
        if(wr_reg) begin
            if(fa[7:4] == 4'h5) mcscr[fa[3:1]] <= wr_w;
            else case(fa[7:1])
                7'h30: bcr1 <= {wr_w[15:12], BIG_ENDIAN, wr_w[10:0]};   //ENDIAN read-only
                7'h31: bcr2 <= wr_w & 16'h3FF0;         //bits 15,14,3-0 reserved (p.239)
                7'h32: wcr1 <= wr_w;
                7'h33: wcr2 <= wr_w;
                7'h34: mcr  <= wr_w & 16'hFFFE;
                7'h36: pcr  <= wr_w;
                default: ;                              //keyed group handled below
            endcase
        end
        if(wr_rtcor) rtcor <= wr_w[7:0];
    end end
end

//RTCNT: keyed software write beats the timer tick (WDT precedent)
always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) rtcnt <= 8'd0;
    else begin
        if(i_CEN && wr_rtcnt)   rtcnt <= wr_w[7:0];
        else if(i_BCEN) begin
            if(rt_match)        rtcnt <= 8'd0;
            else if(presc_tick) rtcnt <= rtcnt + 8'd1;
        end
    end
end

//RTCSR: CMF/OVF set by hardware, cleared only by a keyed write-0 (p.253)
always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) rtcsr <= 8'd0;
    else begin
        if(i_CEN && wr_rtcsr) begin
            rtcsr[6:3] <= wr_w[6:3];
            rtcsr[1:0] <= wr_w[1:0];
            rtcsr[7]   <= rtcsr[7] & wr_w[7];           //write-1 holds, write-0 clears
            rtcsr[2]   <= rtcsr[2] & wr_w[2];
        end
        else if(i_BCEN) begin
            if(rt_match)                     rtcsr[7] <= 1'b1;      //CMF
            if(ref_done && rfcr == rfcr_lim) rtcsr[2] <= 1'b1;      //OVF
        end
    end
end

//RFCR counts refresh cycles (cleared by a keyed write)
always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) rfcr <= 10'd0;
    else begin
        if(i_CEN && wr_rfcr)        rfcr <= wr_w[9:0];
        else if(i_BCEN && ref_done) rfcr <= rfcr + 10'd1;
    end
end

//refresh request: latched at compare match, cleared when the REF cycle runs
always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) ref_req <= 1'b0;
    else begin if(i_BCEN) begin
        if(rt_match && rfsh && !rmode) ref_req <= 1'b1;
        else if(ref_done)              ref_req <= 1'b0;
    end end
end

//self-refresh entry tracker for the front-end request stall
always_ff @(posedge i_CLK or negedge i_POR_n) begin
    if(!i_POR_n) self_active <= 1'b0;
    else begin if(i_BCEN) begin
        if(est == E_SLF_CMD)                       self_active <= 1'b1;
        else if(est == E_SLF_EXIT && ecnt <= 4'd1) self_active <= 1'b0;
    end end
end

assign  o_RCMI_REQ = rtcsr[7] & rtcsr[6];       //CMF & CMIE
assign  o_ROVI_REQ = rtcsr[2] & rtcsr[1];       //OVF & OVIE



///////////////////////////////////////////////////////////
//////  Physical Pin Merge (the shared external bus, table 10.1)
////

/*
    Ordinary bus cycles (ord_busy, latched in the front-end) and SDRAM engine
    cycles (sd_* regs, BCEN domain) never overlap - one outstanding
    transaction - so every shared pin is a plain 2:1 mux of registers.
    Idle levels match silicon: strobes high, address holds, D released.
*/

//pins flip to the ordinary fields only from the grid-aligned launch edge
//(ord_run); the accept-time ord_busy keeps all ownership interlocks
wire    ord_pins  = ord_busy && ord_run;

assign  o_A       = ord_pins ? {ord_addr[25:2], ord_a1, ord_addr[0]} : sd_a;
//16-bit ports live on D15-D0 (fig 10.13): the selected half is driven there
assign  o_D_O     = !ord_pins ? sd_dq_o :
                    !ord_w16  ? ord_wdata :
                    {2{ord_a1 ? ord_wdata[15:0] : ord_wdata[31:16]}};
assign  o_D_OE    = ord_pins ? ord_write      : sd_dq_oe;   //write data held all cycle
assign  o_BS_n    = ord_pins ? ord_bs_n       : sd_bs_n;
assign  o_CS0_n   = ~(ord_pins && ord_area == 3'd0);
assign  o_CS2_n   = ord_pins ? (ord_area != 3'd2) : sd_cs2_n;
assign  o_CS3_n   = ord_pins ? (ord_area != 3'd3) : sd_cs3_n;
assign  o_CS4_n   = ~(ord_pins && ord_area == 3'd4);
assign  o_CS5_n   = ~(ord_pins && ord_area == 3'd5);
assign  o_CS6_n   = ~(ord_pins && ord_area == 3'd6);
assign  o_RD_WR   = ord_pins ? ~ord_write     : sd_cmdwe_n; //low = write cycle
assign  o_RAS3L_n = ord_pins ? 1'b1           : sd_rasl_n;
assign  o_RAS3U_n = ord_pins ? 1'b1           : sd_rasu_n;
assign  o_CASL_n  = ord_pins ? 1'b1           : sd_casl_n;
assign  o_CASU_n  = ord_pins ? 1'b1           : sd_casu_n;
assign  o_WE_n    = !ord_pins ? sd_dqm :
                    !ord_write ? 4'b1111 :
                    !ord_w16   ? ~ord_wstrb :
                    {2'b11, ord_a1 ? ~ord_wstrb[1:0] : ~ord_wstrb[3:2]};    //WE1/WE0 lanes
assign  o_RD_n    = ~(ord_pins && !ord_write);              //ordinary read strobe
assign  o_CKE     = sd_cke;

endmodule

`default_nettype none
