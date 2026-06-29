// Unit tests for top_tdm<> — see header comment in buffer.hpp / buffer_cell.hpp for protocol.
//
// Read buffer protocol (discovered from RTL):
//   buf_r2 has 32 cells.  With active_mode=0 (1 port = 4 beats), there are 8
//   "drain groups" of 4 cells each.  fetch_valid_const=true causes cells to
//   continuously prefetch: cells 0..3 fetch rport_addr_i[24..27]; cells 4..31
//   fetch addr=0.  After reset, cells immediately fill with whatever is in the
//   bank at those addresses (initially zeros = pre-write stale data).
//
//   To read back WRITTEN data, do_read_group() uses a 3-phase protocol:
//     Phase 1 (stale drain): assert req=1 on all N_PARA buses; count exactly
//       FRAME_GRPS*N_PARA rvalid events to drain one complete window → triggers
//       window reset → cells refetch with current rport_addr (= target addrs).
//     Phase 2 (fresh capture): poll for fresh group-0 rvalids; these contain
//       the post-write data for target addrs.
//     Phase 3 (invariant restore): drain remaining FRAME_GRPS-1 groups so
//       rd_ptr wraps back to 0 for the next call.
//
//   INVARIANT: rd_ptr=0 at entry to every do_read_group() call.
//   Maintained by: do_reset() at test start + Phase 3 at call end.
//
//   Address constraint: addrs[] passed to do_read_group must be CONSECUTIVE
//   (addrs[m] = addrs[0] + m*ADDR_STEP) because the TDM mapper uses only
//   cell[0]'s address and fills cell[w] with addrs[0] + w*BYTES_PER_ROW.

#include "top_tdm.hpp"
#include <cstdio>
#include <systemc.h>

using DUT    = top_tdm<>;
using data_t = DUT::data_t;

static constexpr int      NR_TOTAL   = DUT::NUM_RPORT_PORTS; // 36
static constexpr int      NW_TOTAL   = DUT::NUM_WPORT_PORTS; // 32
static constexpr int      N_BUF      = DUT::NUM_TOTAL_BUF;   // 9
static constexpr uint32_t FULL_BE    = 0xFFFF;
static constexpr int      kTimeout   = 1000;

static constexpr int WBUS   = DUT::WR2_BASE;  // 24
static constexpr int RBUS   = DUT::RD2_BASE;  // 24
static constexpr int N_PARA = DUT::WR2_PORTS; // 4
static constexpr int WR3B   = DUT::WR3_BASE;  // 28

static constexpr int      FRAME_SIZE = 32;
static constexpr int      FRAME_GRPS = FRAME_SIZE / N_PARA; // 8
static constexpr uint64_t ADDR_STEP  = 0x10;

static int g_pass = 0;
static int g_fail = 0;

static void CHECK(bool cond, const char *label) {
    if (cond) { ++g_pass; std::printf("  PASS  %s\n", label); }
    else       { ++g_fail; std::printf("  FAIL  %s\n", label); }
}

static data_t make_row(uint32_t v) {
    sc_bv<32> w(v);
    data_t d;
    d.range(31,0)=w; d.range(63,32)=w; d.range(95,64)=w; d.range(127,96)=w;
    return d;
}

SC_MODULE(tb) {
    sc_clock        clk{"clk", 10, SC_NS};
    sc_signal<bool> rst_n{"rst_n"};

    sc_signal<bool>     rport_req[NR_TOTAL];
    sc_signal<uint64_t> rport_addr[NR_TOTAL];
    sc_signal<bool>     rport_we[NR_TOTAL];
    sc_signal<uint32_t> rport_be[NR_TOTAL];
    sc_signal<data_t>   rport_wdata[NR_TOTAL];
    sc_signal<bool>     rport_gnt[NR_TOTAL];
    sc_signal<bool>     rport_rvalid[NR_TOTAL];
    sc_signal<data_t>   rport_rdata[NR_TOTAL];

    sc_signal<bool>     wport_req[NW_TOTAL];
    sc_signal<uint64_t> wport_addr[NW_TOTAL];
    sc_signal<bool>     wport_we[NW_TOTAL];
    sc_signal<uint32_t> wport_be[NW_TOTAL];
    sc_signal<data_t>   wport_wdata[NW_TOTAL];
    sc_signal<bool>     wport_gnt[NW_TOTAL];
    sc_signal<bool>     wport_rvalid[NW_TOTAL];
    sc_signal<data_t>   wport_rdata[NW_TOTAL];

    sc_signal<uint32_t> buf_mode[N_BUF];
    DUT *dut;

    SC_HAS_PROCESS(tb);
    tb(sc_module_name nm) : sc_module(nm) {
        dut = new DUT("dut");
        dut->clk_i(clk); dut->rst_ni(rst_n);
        for (int m = 0; m < NR_TOTAL; ++m) {
            dut->rport_req_i[m](rport_req[m]);
            dut->rport_addr_i[m](rport_addr[m]);
            dut->rport_we_i[m](rport_we[m]);
            dut->rport_be_i[m](rport_be[m]);
            dut->rport_wdata_i[m](rport_wdata[m]);
            dut->rport_gnt_o[m](rport_gnt[m]);
            dut->rport_rvalid_o[m](rport_rvalid[m]);
            dut->rport_rdata_o[m](rport_rdata[m]);
        }
        for (int m = 0; m < NW_TOTAL; ++m) {
            dut->wport_req_i[m](wport_req[m]);
            dut->wport_addr_i[m](wport_addr[m]);
            dut->wport_we_i[m](wport_we[m]);
            dut->wport_be_i[m](wport_be[m]);
            dut->wport_wdata_i[m](wport_wdata[m]);
            dut->wport_gnt_o[m](wport_gnt[m]);
            dut->wport_rvalid_o[m](wport_rvalid[m]);
            dut->wport_rdata_o[m](wport_rdata[m]);
        }
        for (int i = 0; i < N_BUF; ++i)
            dut->buf_active_mode_i[i](buf_mode[i]);
        SC_THREAD(run);
    }
    ~tb() { delete dut; }

    void tick() { wait(clk.posedge_event()); wait(1, SC_NS); }

    void idle_rport(int m) {
        rport_req[m].write(false); rport_we[m].write(false);
        rport_be[m].write(0);      rport_addr[m].write(0);
        rport_wdata[m].write(data_t(0));
    }
    void idle_wport(int m) {
        wport_req[m].write(false); wport_we[m].write(false);
        wport_be[m].write(0);      wport_addr[m].write(0);
        wport_wdata[m].write(data_t(0));
    }

    void do_reset() {
        rst_n.write(false);
        for (int m = 0; m < NR_TOTAL; ++m) idle_rport(m);
        for (int m = 0; m < NW_TOTAL; ++m) idle_wport(m);
        for (int i = 0; i < N_BUF; ++i)    buf_mode[i].write(0);
        tick(); tick();
        rst_n.write(true);
        tick();
    }

    // Write FRAME_SIZE entries; rvalids come as a sustained burst.
    bool do_write_frame(int base_bus, uint64_t base_addr, const data_t data[FRAME_SIZE]) {
        int gnt_per_bus[N_PARA] = {};
        int rv_total = 0, curr_group = 0;

        auto drive_group = [&](int g) {
            for (int m = 0; m < N_PARA; ++m) {
                wport_req[base_bus+m].write(true);
                wport_addr[base_bus+m].write(base_addr+static_cast<uint64_t>(g*N_PARA+m)*ADDR_STEP);
                wport_we[base_bus+m].write(true);
                wport_be[base_bus+m].write(FULL_BE);
                wport_wdata[base_bus+m].write(data[g*N_PARA+m]);
            }
        };
        drive_group(0);

        for (int iter = 0; iter < FRAME_SIZE*kTimeout && rv_total < FRAME_SIZE; ++iter) {
            tick();
            bool ag = true;
            for (int m = 0; m < N_PARA; ++m) {
                if (gnt_per_bus[m] <= curr_group) {
                    if (wport_gnt[base_bus+m].read()) { ++gnt_per_bus[m]; idle_wport(base_bus+m); }
                    else ag = false;
                }
            }
            if (ag && curr_group < FRAME_GRPS-1) { ++curr_group; drive_group(curr_group); }
            for (int m = 0; m < N_PARA; ++m)
                if (wport_rvalid[base_bus+m].read()) ++rv_total;
        }
        for (int m = 0; m < N_PARA; ++m) idle_wport(base_bus+m);
        return rv_total == FRAME_SIZE;
    }

    // 3-phase read: stale drain → fresh capture → invariant restore.
    // addrs[] MUST be consecutive: addrs[m] = addrs[0] + m*ADDR_STEP.
    // INVARIANT: rd_ptr=0 at entry AND exit.
    bool do_read_group(int base_bus, const uint64_t addrs[N_PARA], data_t out[N_PARA]) {
        for (int m = 0; m < N_PARA; ++m) {
            rport_addr[base_bus+m].write(addrs[m]);
            rport_req[base_bus+m].write(true);
            rport_we[base_bus+m].write(false);
            rport_be[base_bus+m].write(FULL_BE);
        }

        // Phase 1: drain one complete stale window (FRAME_GRPS*N_PARA rvalid events).
        int stale_rv = 0;
        for (int iter = 0; stale_rv < FRAME_GRPS*N_PARA && iter < FRAME_GRPS*kTimeout; ++iter) {
            tick();
            for (int m = 0; m < N_PARA; ++m)
                if (rport_rvalid[base_bus+m].read()) ++stale_rv;
        }
        if (stale_rv < FRAME_GRPS*N_PARA) {
            for (int m = 0; m < N_PARA; ++m) idle_rport(base_bus+m);
            return false;
        }
        // Window reset fired; cells now refetch target addrs from bank.

        // Phase 2: capture fresh group-0 rvalids (post-write data).
        bool done[N_PARA] = {};
        int  got = 0;
        for (int iter = 0; got < N_PARA && iter < N_PARA*kTimeout; ++iter) {
            tick();
            for (int m = 0; m < N_PARA; ++m) {
                if (!done[m] && rport_rvalid[base_bus+m].read()) {
                    out[m] = rport_rdata[base_bus+m].read();
                    done[m] = true;
                    ++got;
                }
            }
        }

        // Phase 3: drain remaining FRAME_GRPS-1 groups → rd_ptr=0 invariant.
        int rem_rv = 0;
        for (int iter = 0; rem_rv < (FRAME_GRPS-1)*N_PARA && iter < FRAME_GRPS*kTimeout; ++iter) {
            tick();
            for (int m = 0; m < N_PARA; ++m)
                if (rport_rvalid[base_bus+m].read()) ++rem_rv;
        }

        for (int m = 0; m < N_PARA; ++m) idle_rport(base_bus+m);
        return got == N_PARA;
    }

    static void make_addrs(uint64_t out[N_PARA], uint64_t base) {
        for (int m = 0; m < N_PARA; ++m)
            out[m] = base + static_cast<uint64_t>(m)*ADDR_STEP;
    }
    static void fill_frame(data_t out[FRAME_SIZE], data_t val) {
        for (int i = 0; i < FRAME_SIZE; ++i) out[i] = val;
    }
    static void seq_frame(data_t out[FRAME_SIZE], uint32_t base_val) {
        for (int i = 0; i < FRAME_SIZE; ++i)
            out[i] = make_row(base_val + static_cast<uint32_t>(i));
    }

    void run() {
        // ── T01 ──────────────────────────────────────────────────────────
        std::puts("\n=== T01: Reset — all output ports deasserted ===");
        do_reset();
        bool all_rg=true,all_rr=true,all_wg=true,all_wr=true;
        for (int m=0;m<NR_TOTAL;++m){all_rg&=!rport_gnt[m].read();all_rr&=!rport_rvalid[m].read();}
        for (int m=0;m<NW_TOTAL;++m){all_wg&=!wport_gnt[m].read();all_wr&=!wport_rvalid[m].read();}
        CHECK(all_rg,"T01a rport_gnt_o[*]=0 after reset");
        CHECK(all_rr,"T01b rport_rvalid_o[*]=0 after reset");
        CHECK(all_wg,"T01c wport_gnt_o[*]=0 after reset");
        CHECK(all_wr,"T01d wport_rvalid_o[*]=0 after reset");

        // ── T02 ──────────────────────────────────────────────────────────
        std::puts("\n=== T02: Write frame then read — data integrity ===");
        do_reset();
        data_t t02_frame[FRAME_SIZE]; seq_frame(t02_frame, 0xBEEF0000U);
        CHECK(do_write_frame(WBUS, 0x00, t02_frame), "T02a write frame completed");
        uint64_t t02_addrs[N_PARA]; make_addrs(t02_addrs, 0x00);
        data_t   t02_out[N_PARA];
        CHECK(do_read_group(RBUS, t02_addrs, t02_out), "T02b all N_PARA reads completed");
        bool t02_ok = true;
        for (int m=0;m<N_PARA;++m) {
            if (!(t02_out[m]==t02_frame[m])) {
                std::printf("  FAIL  T02: mismatch m=%d exp=0x%08x got=0x%08x\n",m,
                    (uint32_t)t02_frame[m].range(31,0).to_uint(),
                    (uint32_t)t02_out[m].range(31,0).to_uint());
                t02_ok=false; ++g_fail;
            } else { ++g_pass; }
        }
        if (t02_ok) std::puts("  PASS  T02c first N_PARA read-backs match written data");

        // ── T03 ──────────────────────────────────────────────────────────
        std::puts("\n=== T03: Multi-read same address — all N_PARA buses ===");
        do_reset();
        data_t t03_frame[FRAME_SIZE];
        fill_frame(t03_frame, make_row(0xCAFEBABEU));
        CHECK(do_write_frame(WBUS, 0x00, t03_frame), "T03 setup write frame");
        // All entries equal 0xcafebabe; cell[m] gets addr[0]+m*ADDR_STEP → same value.
        uint64_t t03_addrs[N_PARA]; for(int m=0;m<N_PARA;++m) t03_addrs[m]=0x00;
        data_t   t03_out[N_PARA];
        CHECK(do_read_group(RBUS, t03_addrs, t03_out), "T03a reads completed");
        bool t03_ok = true;
        for (int m=0;m<N_PARA;++m) t03_ok &= (t03_out[m]==t03_frame[0]);
        CHECK(t03_ok, "T03b all same-address read data values correct");

        // ── T04 ──────────────────────────────────────────────────────────
        std::puts("\n=== T04: Multi-read distinct consecutive addresses ===");
        do_reset();
        data_t t04_frame[FRAME_SIZE];
        for (int i=0;i<FRAME_SIZE;++i)
            t04_frame[i]=make_row(0x10000000U*static_cast<uint32_t>((i%N_PARA)+1));
        CHECK(do_write_frame(WBUS, 0x00, t04_frame), "T04 setup write frame");
        uint64_t t04_addrs[N_PARA]; make_addrs(t04_addrs, 0x00);
        data_t   t04_out[N_PARA];
        CHECK(do_read_group(RBUS, t04_addrs, t04_out), "T04a reads completed");
        bool t04_ok = true;
        for (int m=0;m<N_PARA;++m) t04_ok &= (t04_out[m]==t04_frame[m]);
        CHECK(t04_ok, "T04b all distinct-address read data values correct");

        // ── T05 ──────────────────────────────────────────────────────────
        std::puts("\n=== T05: 64-address sequential write-then-read ===");
        do_reset();
        static constexpr int N_FRAMES5 = 2;
        static constexpr int N_ADDRS5  = N_FRAMES5 * FRAME_SIZE; // 64
        data_t t05_data[N_ADDRS5];
        for (int i=0;i<N_ADDRS5;++i)
            t05_data[i]=make_row(0x01010101U*static_cast<uint32_t>(i+1));
        for (int f=0;f<N_FRAMES5;++f)
            CHECK(do_write_frame(WBUS, static_cast<uint64_t>(f)*FRAME_SIZE*ADDR_STEP,
                                 t05_data+f*FRAME_SIZE), "T05 write frame");

        bool t05_ok = true;
        static constexpr int N_GROUPS5 = N_ADDRS5 / N_PARA; // 16
        for (int g=0;g<N_GROUPS5;++g) {
            uint64_t a5[N_PARA];
            make_addrs(a5, static_cast<uint64_t>(g)*N_PARA*ADDR_STEP);
            data_t o5[N_PARA];
            if (!do_read_group(RBUS, a5, o5)) {
                std::printf("  FAIL  T05: read group %d timed out\n", g);
                t05_ok=false; g_fail+=N_PARA; continue;
            }
            for (int m=0;m<N_PARA;++m) {
                if (!(o5[m]==t05_data[g*N_PARA+m])) {
                    std::printf("  FAIL  T05: mismatch i=%d addr=0x%03llx\n",
                        g*N_PARA+m,(unsigned long long)a5[m]);
                    t05_ok=false; ++g_fail;
                } else { ++g_pass; }
            }
        }
        if (t05_ok) std::puts("  PASS  T05 all 64 addresses read back correctly");

        // ── T06 ──────────────────────────────────────────────────────────
        std::puts("\n=== T06: Overwrite — second write to same addresses ===");
        do_reset();
        data_t t06_A[FRAME_SIZE], t06_B[FRAME_SIZE];
        fill_frame(t06_A, make_row(0xDEADBEEFU));
        fill_frame(t06_B, make_row(0x01234567U));
        CHECK(do_write_frame(WBUS, 0x00, t06_A), "T06a first write frame");
        CHECK(do_write_frame(WBUS, 0x00, t06_B), "T06b second write frame (overwrite)");
        uint64_t t06_addrs[N_PARA]; make_addrs(t06_addrs, 0x00);
        data_t   t06_out[N_PARA];
        CHECK(do_read_group(RBUS, t06_addrs, t06_out), "T06c read-back completed");
        bool t06_ok = true;
        for (int m=0;m<N_PARA;++m) {
            if (!(t06_out[m]==t06_B[m])) {
                std::printf("  FAIL  T06: overwrite not reflected at bus %d\n",m);
                t06_ok=false; ++g_fail;
            } else { ++g_pass; }
        }
        if (t06_ok) std::puts("  PASS  T06d all addresses return second (overwrite) value");

        // ── T07 ──────────────────────────────────────────────────────────
        std::puts("\n=== T07: Alternating bit patterns 0xAA/0x55 ===");
        do_reset();
        data_t t07_aa[FRAME_SIZE], t07_55[FRAME_SIZE];
        fill_frame(t07_aa, make_row(0xAAAAAAAAU));
        fill_frame(t07_55, make_row(0x55555555U));
        CHECK(do_write_frame(WBUS, 0x000,                t07_aa), "T07a write 0xAA frame");
        CHECK(do_write_frame(WBUS, FRAME_SIZE*ADDR_STEP, t07_55), "T07b write 0x55 frame");
        uint64_t t07_aa_a[N_PARA]; make_addrs(t07_aa_a, 0x000);
        uint64_t t07_55_a[N_PARA]; make_addrs(t07_55_a, FRAME_SIZE*ADDR_STEP);
        data_t   t07_oo[N_PARA], t07_o5[N_PARA];
        CHECK(do_read_group(RBUS, t07_aa_a, t07_oo), "T07c 0xAA reads completed");
        CHECK(do_read_group(RBUS, t07_55_a, t07_o5), "T07d 0x55 reads completed");
        bool t07_ok = true;
        for (int m=0;m<N_PARA;++m) {
            if (!(t07_oo[m]==t07_aa[0])){std::printf("  FAIL  T07: 0xAA mismatch m=%d\n",m);t07_ok=false;++g_fail;}else{++g_pass;}
            if (!(t07_o5[m]==t07_55[0])){std::printf("  FAIL  T07: 0x55 mismatch m=%d\n",m);t07_ok=false;++g_fail;}else{++g_pass;}
        }
        if (t07_ok) std::puts("  PASS  T07e all alternating-pattern reads correct");

        // ── T08 ──────────────────────────────────────────────────────────
        std::puts("\n=== T08: Cross-address independence — frame B does not corrupt frame A ===");
        do_reset();
        static constexpr uint64_t BASE_A8 = 0x000;
        static constexpr uint64_t BASE_B8 = static_cast<uint64_t>(FRAME_SIZE)*ADDR_STEP;
        data_t t08_A[FRAME_SIZE], t08_B[FRAME_SIZE];
        seq_frame(t08_A, 0xA0000001U); seq_frame(t08_B, 0xB0000001U);
        CHECK(do_write_frame(WBUS, BASE_A8, t08_A), "T08a write frame A");
        CHECK(do_write_frame(WBUS, BASE_B8, t08_B), "T08b write frame B");
        uint64_t t08_addrs[N_PARA]; make_addrs(t08_addrs, BASE_A8);
        data_t   t08_out[N_PARA];
        CHECK(do_read_group(RBUS, t08_addrs, t08_out), "T08c frame-A read-back");
        bool t08_ok = true;
        for (int m=0;m<N_PARA;++m) {
            if (!(t08_out[m]==t08_A[m])) {
                std::printf("  FAIL  T08: frame-A addr 0x%03llx corrupted\n",
                    (unsigned long long)(BASE_A8+m*ADDR_STEP));
                t08_ok=false; ++g_fail;
            } else { ++g_pass; }
        }
        if (t08_ok) std::puts("  PASS  T08d frame-A data unchanged after frame-B write");

        // ── T09 ──────────────────────────────────────────────────────────
        std::puts("\n=== T09: Port group isolation — WR2/RD2 does not leak to other groups ===");
        do_reset();
        data_t t09_frame[FRAME_SIZE];
        fill_frame(t09_frame, make_row(0xFEDCBA98U));

        // Write via WR2 while monitoring other wport groups for leakage
        int  g9[N_PARA]={}, rv9=0, grp9=0;
        bool w9_no_leak=true;
        auto drv9=[&](int g){
            for(int m=0;m<N_PARA;++m){
                wport_req[WBUS+m].write(true);
                wport_addr[WBUS+m].write(static_cast<uint64_t>(g*N_PARA+m)*ADDR_STEP);
                wport_we[WBUS+m].write(true); wport_be[WBUS+m].write(FULL_BE);
                wport_wdata[WBUS+m].write(t09_frame[g*N_PARA+m]);
            }
        };
        drv9(0);
        for (int iter=0;iter<FRAME_SIZE*kTimeout&&rv9<FRAME_SIZE;++iter){
            tick();
            bool ag=true;
            for(int m=0;m<N_PARA;++m){
                if(g9[m]<=grp9){if(wport_gnt[WBUS+m].read()){++g9[m];idle_wport(WBUS+m);}else ag=false;}
            }
            if(ag&&grp9<FRAME_GRPS-1){++grp9;drv9(grp9);}
            for(int m=0;m<N_PARA;++m) if(wport_rvalid[WBUS+m].read()) ++rv9;
            for(int m=0;m<WBUS;++m)         w9_no_leak&=!wport_rvalid[m].read()&&!wport_gnt[m].read();
            for(int m=WBUS+N_PARA;m<NW_TOTAL;++m) w9_no_leak&=!wport_rvalid[m].read()&&!wport_gnt[m].read();
        }
        for(int m=0;m<N_PARA;++m) idle_wport(WBUS+m);
        CHECK(rv9==FRAME_SIZE, "T09a WR2 frame write completed");
        CHECK(w9_no_leak,      "T09b no other wport group saw rvalid/gnt during WR2 write");

        // Read via all 4 RD2 buses (3-phase) while monitoring other rport groups
        uint64_t t09_addrs[N_PARA]; make_addrs(t09_addrs, 0x00);
        data_t   t09_out[N_PARA];
        bool     r09_no_leak = true;

        auto chk_riso=[&](){
            for(int m=0;m<RBUS;++m)
                r09_no_leak&=!rport_rvalid[m].read()&&!rport_gnt[m].read();
            for(int m=RBUS+N_PARA;m<NR_TOTAL;++m)
                r09_no_leak&=!rport_rvalid[m].read()&&!rport_gnt[m].read();
        };

        for(int m=0;m<N_PARA;++m){
            rport_addr[RBUS+m].write(t09_addrs[m]);
            rport_req[RBUS+m].write(true);
            rport_we[RBUS+m].write(false);
            rport_be[RBUS+m].write(FULL_BE);
        }

        int stale9=0;
        for(int iter=0;stale9<FRAME_GRPS*N_PARA&&iter<FRAME_GRPS*kTimeout;++iter){
            tick();
            for(int m=0;m<N_PARA;++m) if(rport_rvalid[RBUS+m].read()) ++stale9;
            chk_riso();
        }
        CHECK(stale9>=FRAME_GRPS*N_PARA, "T09c RD2 stale window drained");

        bool r9done[N_PARA]={}; int r9got=0;
        for(int iter=0;r9got<N_PARA&&iter<N_PARA*kTimeout;++iter){
            tick();
            for(int m=0;m<N_PARA;++m)
                if(!r9done[m]&&rport_rvalid[RBUS+m].read()){
                    t09_out[m]=rport_rdata[RBUS+m].read();r9done[m]=true;++r9got;
                }
            chk_riso();
        }
        CHECK(r9got==N_PARA, "T09d RD2 group read completed");

        int rem9=0;
        for(int iter=0;rem9<(FRAME_GRPS-1)*N_PARA&&iter<FRAME_GRPS*kTimeout;++iter){
            tick();
            for(int m=0;m<N_PARA;++m) if(rport_rvalid[RBUS+m].read()) ++rem9;
            chk_riso();
        }
        for(int m=0;m<N_PARA;++m) idle_rport(RBUS+m);
        CHECK(r09_no_leak, "T09e no other rport group saw rvalid/gnt during RD2 read");

        // ── T10 ──────────────────────────────────────────────────────────
        std::puts("\n=== T10: Two simultaneous write frames (WR2 + WR3) ===");
        do_reset();
        static constexpr uint64_t T10_BASE2 = 0x000;
        static constexpr uint64_t T10_BASE3 = static_cast<uint64_t>(FRAME_SIZE)*ADDR_STEP;
        data_t t10_d2[FRAME_SIZE], t10_d3[FRAME_SIZE];
        seq_frame(t10_d2, 0x20000001U); seq_frame(t10_d3, 0x30000001U);

        int gnt2[N_PARA]={}, gnt3[N_PARA]={}, rv2=0, rv3=0, grp2=0, grp3=0;
        auto drv2=[&](int g){for(int m=0;m<N_PARA;++m){
            wport_req[WBUS+m].write(true);
            wport_addr[WBUS+m].write(T10_BASE2+static_cast<uint64_t>(g*N_PARA+m)*ADDR_STEP);
            wport_we[WBUS+m].write(true);wport_be[WBUS+m].write(FULL_BE);
            wport_wdata[WBUS+m].write(t10_d2[g*N_PARA+m]);}};
        auto drv3=[&](int g){for(int m=0;m<N_PARA;++m){
            wport_req[WR3B+m].write(true);
            wport_addr[WR3B+m].write(T10_BASE3+static_cast<uint64_t>(g*N_PARA+m)*ADDR_STEP);
            wport_we[WR3B+m].write(true);wport_be[WR3B+m].write(FULL_BE);
            wport_wdata[WR3B+m].write(t10_d3[g*N_PARA+m]);}};
        drv2(0); drv3(0);
        for(int iter=0;iter<2*FRAME_SIZE*kTimeout&&(rv2<FRAME_SIZE||rv3<FRAME_SIZE);++iter){
            tick();
            bool a2=true,a3=true;
            for(int m=0;m<N_PARA;++m){
                if(gnt2[m]<=grp2){if(wport_gnt[WBUS+m].read()){++gnt2[m];idle_wport(WBUS+m);}else a2=false;}
                if(gnt3[m]<=grp3){if(wport_gnt[WR3B+m].read()){++gnt3[m];idle_wport(WR3B+m);}else a3=false;}
            }
            if(a2&&grp2<FRAME_GRPS-1){++grp2;drv2(grp2);}
            if(a3&&grp3<FRAME_GRPS-1){++grp3;drv3(grp3);}
            for(int m=0;m<N_PARA;++m){
                if(wport_rvalid[WBUS+m].read())++rv2;
                if(wport_rvalid[WR3B+m].read())++rv3;
            }
        }
        for(int m=0;m<N_PARA;++m){idle_wport(WBUS+m);idle_wport(WR3B+m);}
        CHECK(rv2==FRAME_SIZE,"T10a WR2 frame writes completed");
        CHECK(rv3==FRAME_SIZE,"T10b WR3 frame writes completed");

        uint64_t t10_a2[N_PARA]; make_addrs(t10_a2, T10_BASE2);
        uint64_t t10_a3[N_PARA]; make_addrs(t10_a3, T10_BASE3);
        data_t   t10_o2[N_PARA], t10_o3[N_PARA];
        CHECK(do_read_group(RBUS, t10_a2, t10_o2), "T10c WR2 read-back completed");
        CHECK(do_read_group(RBUS, t10_a3, t10_o3), "T10d WR3 read-back completed");
        bool t10_ok=true;
        for(int m=0;m<N_PARA;++m){
            if(!(t10_o2[m]==t10_d2[m])){std::printf("  FAIL  T10: WR2 mismatch m=%d\n",m);t10_ok=false;++g_fail;}else{++g_pass;}
            if(!(t10_o3[m]==t10_d3[m])){std::printf("  FAIL  T10: WR3 mismatch m=%d\n",m);t10_ok=false;++g_fail;}else{++g_pass;}
        }
        if(t10_ok) std::puts("  PASS  T10e all 8 addresses from both groups correct");

        // ─────────────────────────────────────────────────────────────────
        std::puts("\n=== Summary ===");
        std::printf("  passed: %d\n  failed: %d\n", g_pass, g_fail);
        sc_stop();
    }
};

int sc_main(int, char **) {
    tb t{"tb"};
    sc_start();
    if (g_fail > 0) { std::fprintf(stderr,"\n%d test(s) FAILED\n",g_fail); return 1; }
    std::puts("\nAll tests passed.");
    return 0;
}
