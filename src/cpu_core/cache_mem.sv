`default_nettype none

/*
    Cache memory primitives, kept in dedicated modules so synthesis maps them
    cleanly onto Cyclone V block RAM. All arrays carry an explicit ramstyle
    directive (M10K) and the synchronous registered-read pattern Quartus needs
    to infer BRAM. V (valid) and U (dirty) live in their own DOUBLE-depth VU
    RAM (with the LRU) so CCR.CF flushes by swapping to a pre-cleared shadow
    half in one cycle (p.106; the real SH3 flush is 1-2 cycles) - the tag
    array itself is never cleared: V=0 makes a stale tag unreachable. The
    controller scrubs the retired half in the background.

    The data array is split into one bank per way (cache_data_bank_wt x4) so all
    four ways are read in parallel and the hit way is selected after the tag
    compare - this avoids any tag-RAM -> data-RAM combinational chain. Each bank
    is a SIMPLE DUAL-PORT (1R1W) M10K: one read and one independent write address.

    SINGLE-CLOCK write-through (new-data) bypass: both ports now capture on the
    SAME edge, so a write and a read-address capture to one cell collide - the
    M10K mixed-port RDW returns OLD data (Cyclone V has no mixed-port new-data
    mode in silicon; altsyncram offers only OLD_DATA/DONT_CARE there). The
    dclk design dodged this by time-sharing (write cen_p, read cen_n). Here an
    EXPLICIT soft bypass restores write-before-read order: the collision compare
    and the write word are REGISTERED at the capture edge, and o_DO muxes the
    held write data over the RAM q per lane. One 2:1 after the RAM output;
    identical netlist in simulation and synthesis (no sim/synth split needed).

    RDW contract (ibara bug #3, 2026-07-30): the DATA bank consumes the RAM q
    on a collision - the UN-strobed lanes of a sub-word store (goldens D/E) -
    so its mixed-port RDW must be OLD_DATA in silicon. no_rw_check fitted
    DONT_CARE and silicon served garbage on exactly those lanes (sim models
    old-data, so no simulator can ever see it); dropping the attribute only
    surfaced warning 276027 (template inferred as DUAL-CLOCK RAM, RDW
    undefined) and still fitted DONT_CARE - Quartus inference cannot express
    OLD_DATA. The bank therefore instantiates altsyncram directly with
    read_during_write_mode_mixed_ports=OLD_DATA (Verilator keeps the
    behavioral twin - identical semantics on every consumed lane). Tag/LRU
    keep inference + no_rw_check: their bypass is full-entry, the colliding
    q is provably dead there.
*/

/* verilator lint_off DECLFILENAME */

///////////////////////////////////////////////////////////
//////  Data bank - 1R1W + per-byte write-through bypass, one per way (4 kB)
////

module cache_data_bank_wt (
    input   wire            i_CLK,
    input   wire            i_EN,       //single architectural clock enable
    input   wire    [9:0]   i_RADDR,    //read  port {index[7:0], word[1:0]} = 1024 longwords
    input   wire            i_WE,       //write port enable (this way's bank)
    input   wire    [3:0]   i_BWE,      //per-byte lane enable; lane b <-> i_DI[8b+7:8b]
    input   wire    [9:0]   i_WADDR,    //write port {index[7:0], word[1:0]}
    input   wire    [31:0]  i_DI,
    output  wire    [31:0]  o_DO
);

logic   [31:0]  rd_q;       //RAM read word (OLD data on a collision - contracted)
logic   [3:0]   byp_q;      //per-lane collision: this edge wrote the cell being read
logic   [31:0]  di_q;       //held write word for the bypass lanes

`ifdef VERILATOR
//Behavioral twin of the altsyncram below. Old-data on collision is now the
//CONTRACTED silicon behavior, so sim and silicon agree on every consumed lane.
logic [3:0][7:0] ram [0:1023];      //byte lanes; lane b <-> i_DI[8b+7:8b]
always_ff @(posedge i_CLK) if(i_EN) begin
    if(i_WE) begin                              //write port - independent address
        if(i_BWE[0]) ram[i_WADDR][0] <= i_DI[ 7: 0];
        if(i_BWE[1]) ram[i_WADDR][1] <= i_DI[15: 8];
        if(i_BWE[2]) ram[i_WADDR][2] <= i_DI[23:16];
        if(i_BWE[3]) ram[i_WADDR][3] <= i_DI[31:24];
    end
    rd_q  <= ram[i_RADDR];                      //read port - registered address/old data
end
`else
//Direct altsyncram: the ONLY way to pin mixed-port RDW = OLD_DATA (inference
//classes this template dual-clock, warning 276027, and fits DONT_CARE).
//Masked (un-strobed) lanes read old data on a collision - the consumed case.
altsyncram #(
    .operation_mode                     ("DUAL_PORT"),
    .width_a                            (32),
    .widthad_a                          (10),
    .numwords_a                         (1024),
    .width_byteena_a                    (4),
    .byte_size                          (8),
    .width_b                            (32),
    .widthad_b                          (10),
    .numwords_b                         (1024),
    .address_reg_b                      ("CLOCK0"),
    .outdata_reg_b                      ("UNREGISTERED"),
    .read_during_write_mode_mixed_ports ("OLD_DATA"),
    .ram_block_type                     ("M10K"),
    .intended_device_family             ("Cyclone V"),
    .lpm_type                           ("altsyncram")
) u_ram (
    .clock0     (i_CLK),
    .clocken0   (i_EN),                 //gates BOTH the write and the read-address capture
    .wren_a     (i_WE),
    .address_a  (i_WADDR),
    .data_a     (i_DI),
    .byteena_a  (i_BWE),
    .address_b  (i_RADDR),
    .q_b        (rd_q)
);
`endif

always_ff @(posedge i_CLK) if(i_EN) begin
    byp_q <= {4{i_WE && (i_WADDR == i_RADDR)}} & i_BWE;   //same-edge RDW, per strobed lane
    di_q  <= i_DI;
end

`ifdef HS3_RDW_HOSTILE_CACHE
//tb-only ADVERSARIAL silicon model (never synthesized): replays the PRE-FIX netlist
//contract (no_rw_check -> mixed-port RDW = DONT_CARE), where silicon guarantees
//NOTHING for the collision lanes the bypass does not cover. Invert them so any
//consumer goes loudly wrong in sim - the un-strobed lanes of a sub-word store.
logic   [3:0]   col_q;      //collision lanes NOT covered by the write-through bypass
logic   [31:0]  col_cnt = 0;//exposed-collision read cycles (diagnostic, printed at exit)
always_ff @(posedge i_CLK) if(i_EN) begin
    col_q <= {4{i_WE && (i_WADDR == i_RADDR)}} & ~i_BWE;
    if(|col_q) col_cnt <= col_cnt + 32'd1;
end
final $display("[rdw_hostile] %m: %0d exposed-collision read cycles", col_cnt);
wire    [31:0]  rd_eff = { col_q[3] ? ~rd_q[31:24] : rd_q[31:24],
                           col_q[2] ? ~rd_q[23:16] : rd_q[23:16],
                           col_q[1] ? ~rd_q[15: 8] : rd_q[15: 8],
                           col_q[0] ? ~rd_q[ 7: 0] : rd_q[ 7: 0] };
`else
wire    [31:0]  rd_eff = rd_q;
`endif

//Write-through compose: bypassed lanes take the held write byte, others the RAM q.
assign  o_DO[ 7: 0] = byp_q[0] ? di_q[ 7: 0] : rd_eff[ 7: 0];
assign  o_DO[15: 8] = byp_q[1] ? di_q[15: 8] : rd_eff[15: 8];
assign  o_DO[23:16] = byp_q[2] ? di_q[23:16] : rd_eff[23:16];
assign  o_DO[31:24] = byp_q[3] ? di_q[31:24] : rd_eff[31:24];

endmodule


///////////////////////////////////////////////////////////
//////  Tag RAM - 1R1W + write-through bypass (256 x 19 = tag[18:0])
////

/*
    M10K, MEASURED choice: an MLAB variant (async read + fabric rdaddr_q) was built
    and fitted - the 256-deep composition needs 8-deep MLAB banking plus a wide
    output mux, and the depth-mux levels + inter-LAB routing cost MORE than the
    M10K's ~2.3 ns tCO. 32-deep arrays are the profitable MLAB shape; 256-deep is
    not. V/U moved to the shadowed VU RAM below; the bypass covers the fill-
    validate tag write against a same-set lookup captured at the same edge.
*/

module cache_tag_ram_wt (
    input   wire            i_CLK,
    input   wire            i_EN,
    input   wire    [7:0]   i_RADDR,
    input   wire            i_WE,
    input   wire    [7:0]   i_WADDR,
    input   wire    [18:0]  i_DI,
    output  wire    [18:0]  o_DO
);

(* ramstyle = "M10K, no_rw_check" *) logic [18:0] ram [0:255];

logic   [18:0]  rd_q;
logic           byp_q;
logic   [18:0]  di_q;

always_ff @(posedge i_CLK) if(i_EN) begin
    if(i_WE) ram[i_WADDR] <= i_DI;
    rd_q  <= ram[i_RADDR];
    byp_q <= i_WE && (i_WADDR == i_RADDR);
    di_q  <= i_DI;
end

assign  o_DO = byp_q ? di_q : rd_q;

endmodule


///////////////////////////////////////////////////////////
//////  VU RAM - 1R1W + write-through bypass (512 x 2 = {V, U}, one per way)
////

/*
    VU = {V (valid), U (dirty)} of one way, DOUBLE depth: addr MSB is the
    active-bank select (shadow-swap CCR.CF flush, see cache.sv). The bypass
    guards the store-hit U write: it must be visible to the very next lookup
    of the same set, else a stale-clean victim would skip its write-back
    (lost store). Scrub writes hit the other bank - the 9-bit address compare
    keeps them out of the bypass automatically.
*/

module cache_vu_ram_wt (
    input   wire            i_CLK,
    input   wire            i_EN,
    input   wire    [8:0]   i_RADDR,    //{bank, set}
    input   wire            i_WE,
    input   wire    [8:0]   i_WADDR,
    input   wire    [1:0]   i_DI,       //{V, U}
    output  wire    [1:0]   o_DO
);

(* ramstyle = "M10K, no_rw_check" *) logic [1:0] ram [0:511];

logic   [1:0]   rd_q;
logic           byp_q;
logic   [1:0]   di_q;

always_ff @(posedge i_CLK) if(i_EN) begin
    if(i_WE) ram[i_WADDR] <= i_DI;
    rd_q  <= ram[i_RADDR];
    byp_q <= i_WE && (i_WADDR == i_RADDR);
    di_q  <= i_DI;
end

assign  o_DO = byp_q ? di_q : rd_q;

endmodule


///////////////////////////////////////////////////////////
//////  LRU RAM - 1R1W + write-through bypass (512 x 6, 6-bit pseudo-LRU; p.104)
////

/*
    The bypass keeps replacement decisions bit-exact with the dclk ordering: a
    same-set access right after an MRU update must see the updated pseudo-LRU,
    else victim choices (and thus external write-back traffic) would diverge.
    DOUBLE depth like the VU RAM: addr MSB is the shadow-swap bank select.
*/

module cache_lru_ram_wt (
    input   wire            i_CLK,
    input   wire            i_EN,
    input   wire    [8:0]   i_RADDR,    //{bank, set}
    input   wire            i_WE,
    input   wire    [8:0]   i_WADDR,
    input   wire    [5:0]   i_DI,
    output  wire    [5:0]   o_DO
);

(* ramstyle = "M10K, no_rw_check" *) logic [5:0] ram [0:511];

logic   [5:0]   rd_q;
logic           byp_q;
logic   [5:0]   di_q;

always_ff @(posedge i_CLK) if(i_EN) begin
    if(i_WE) ram[i_WADDR] <= i_DI;
    rd_q  <= ram[i_RADDR];
    byp_q <= i_WE && (i_WADDR == i_RADDR);
    di_q  <= i_DI;
end

assign  o_DO = byp_q ? di_q : rd_q;

endmodule

/* verilator lint_on DECLFILENAME */

`default_nettype none
