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
//   addrs[] entries are independent per lane — tdm.hpp maps each TDM slot's
//   address separately (per-slot g_addr_i[NUM_WORD]), so addrs[] need not be
//   consecutive; duplicates and addr=0 (NOP) entries are both valid.

#include "top_tdm.hpp"
#include "unit_test_common.hpp"
#include <cstdio>
#include <systemc.h>

using DUT    = top_tdm<>;
using data_t = DUT::data_t;

static constexpr int      NR_TOTAL = DUT::NUM_RPORT_PORTS; // 36
static constexpr int      NW_TOTAL = DUT::NUM_WPORT_PORTS; // 32
static constexpr int      N_BUF    = DUT::NUM_TOTAL_BUF;   // 9
static constexpr uint32_t FULL_BE  = 0xFFFF;
static constexpr int      kTimeout = 1000;

static constexpr int WBUS   = DUT::WR2_BASE;  // 24
static constexpr int RBUS   = DUT::RD2_BASE;  // 24
static constexpr int N_PARA = DUT::WR2_PORTS; // 4
static constexpr int WR3B   = DUT::WR3_BASE;  // 28

static constexpr int      FRAME_SIZE = 32;
static constexpr int      FRAME_GRPS = FRAME_SIZE / N_PARA; // 8
static constexpr uint64_t ADDR_STEP  = 0x10;
// addr=0 is reserved as NOP sentinel; real test data starts at 0x200.
static constexpr uint64_t kBase = 0x200;

SC_MODULE(tb) {
    sc_clock        clk{"clk", 10, SC_NS};
    sc_signal<bool> rst_n{"rst_n"};

    obi_signal_bundle<data_t> rport[NR_TOTAL];
    obi_signal_bundle<data_t> wport[NW_TOTAL];

    sc_signal<uint32_t> buf_mode[N_BUF];
    sc_signal<uint64_t> buf_map_r[N_BUF];
    sc_signal<uint64_t> buf_map_c[N_BUF];
    sc_signal<uint64_t> buf_map_l[N_BUF];
    sc_signal<uint64_t> buf_map_sm[N_BUF];

    // Read buffers now always prefetch a whole window from a per-buffer
    // lookahead bus (fetch_addr_i is no longer wired to the live port
    // address — see top_tdm.hpp) — this hand-driven testbench has no
    // trace/task concept, so it just stages whatever addresses the current
    // test wants read next via set_fetch() below. fetch_valid stays
    // permanently true (buf_r0..r3 only; buf_r4/DMA has no dedicated valid
    // gate in the DUT, same as before) since this harness has no start_cycle
    // fencing to race against — see agu.hpp's lookahead_ready() for the
    // fencing case this gate exists for elsewhere.
    static constexpr int NUM_RD_BUF_TB = 5;
    sc_signal<uint64_t>  fetch[NUM_RD_BUF_TB][FRAME_SIZE];
    sc_signal<bool>      fetch_valid[NUM_RD_BUF_TB - 1];
    DUT                 *dut;

    SC_HAS_PROCESS(tb);
    tb(sc_module_name nm) : sc_module(nm) {
        dut = new DUT("dut");
        dut->clk_i(clk);
        dut->rst_ni(rst_n);
        bind_obi_group(dut->rport_req_i, dut->rport_addr_i, dut->rport_we_i, dut->rport_be_i,
                       dut->rport_wdata_i, dut->rport_gnt_o, dut->rport_rvalid_o,
                       dut->rport_rdata_o, rport);
        bind_obi_group(dut->wport_req_i, dut->wport_addr_i, dut->wport_we_i, dut->wport_be_i,
                       dut->wport_wdata_i, dut->wport_gnt_o, dut->wport_rvalid_o,
                       dut->wport_rdata_o, wport);
        for (int w = 0; w < FRAME_SIZE; ++w) {
            dut->rd0_lookahead_i[w](fetch[0][w]);
            dut->rd1_lookahead_i[w](fetch[1][w]);
            dut->rd2_lookahead_i[w](fetch[2][w]);
            dut->rd3_lookahead_i[w](fetch[3][w]);
            dut->rd4_lookahead_i[w](fetch[4][w]);
        }
        dut->rd0_lookahead_valid_i(fetch_valid[0]);
        dut->rd1_lookahead_valid_i(fetch_valid[1]);
        dut->rd2_lookahead_valid_i(fetch_valid[2]);
        dut->rd3_lookahead_valid_i(fetch_valid[3]);
        for (int i = 0; i < N_BUF; ++i) {
            dut->buf_active_mode_i[i](buf_mode[i]);
            dut->buf_map_r_i[i](buf_map_r[i]);
            dut->buf_map_c_i[i](buf_map_c[i]);
            dut->buf_map_l_i[i](buf_map_l[i]);
            dut->buf_map_store_mode_i[i](buf_map_sm[i]);
        }
        SC_THREAD(run);
    }
    ~tb() {
        delete dut;
    }

    void idle_rport(int m) {
        rport[m].req.write(false);
        rport[m].we.write(false);
        rport[m].be.write(0);
        rport[m].addr.write(0);
        rport[m].wdata.write(data_t(0));
    }
    void idle_wport(int m) {
        wport[m].req.write(false);
        wport[m].we.write(false);
        wport[m].be.write(0);
        wport[m].addr.write(0);
        wport[m].wdata.write(data_t(0));
    }

    void do_reset() {
        rst_n.write(false);
        for (int m = 0; m < NR_TOTAL; ++m)
            idle_rport(m);
        for (int m = 0; m < NW_TOTAL; ++m)
            idle_wport(m);
        for (int i = 0; i < N_BUF; ++i) {
            buf_mode[i].write(0);
            buf_map_r[i].write(4);
            buf_map_c[i].write(4);
            buf_map_l[i].write(8);
            buf_map_sm[i].write(0);
        }
        for (int b = 0; b < NUM_RD_BUF_TB; ++b)
            for (int w = 0; w < FRAME_SIZE; ++w)
                fetch[b][w].write(0);
        for (int b = 0; b < NUM_RD_BUF_TB - 1; ++b)
            fetch_valid[b].write(true);
        tick(clk);
        tick(clk);
        rst_n.write(true);
        tick(clk);
    }

    // Maps a read port base_bus (0, DUT::RD1_BASE, RBUS/DUT::RD2_BASE,
    // DUT::RD3_BASE, or DUT::RD4_BASE) to its buffer index, then stages
    // [addrs[0..n), zero-padded to FRAME_SIZE] into that buffer's lookahead
    // bus. Cells only latch this the next time their window resets (see
    // buffer_cell.hpp), same timing requirement the old rport_addr_i-wired
    // fetch_addr_i had — call this alongside (or before) writing the target
    // addresses to rport[...].addr, which now only drives the port-side
    // req/gnt/rvalid handshake.
    void set_fetch(int base_bus, const uint64_t *addrs, int n) {
        static const int kBases[NUM_RD_BUF_TB] = {0, DUT::RD1_BASE, RBUS, DUT::RD3_BASE,
                                                  DUT::RD4_BASE};
        int              buf_idx               = -1;
        for (int i = 0; i < NUM_RD_BUF_TB; ++i)
            if (kBases[i] == base_bus) {
                buf_idx = i;
                break;
            }
        if (buf_idx < 0)
            SC_REPORT_FATAL("tb", "set_fetch: unrecognized base_bus");
        for (int w = 0; w < n; ++w)
            fetch[buf_idx][w].write(addrs[w]);
        for (int w = n; w < FRAME_SIZE; ++w)
            fetch[buf_idx][w].write(0);
    }

    // Write FRAME_SIZE entries; rvalids come as a sustained burst.
    // addr_step lets callers space entries by something other than ADDR_STEP
    // (e.g. a same-bank-conflict stride); out_cycles, if non-null, receives
    // the number of ticks taken. Both default to preserve every existing
    // caller's behavior.
    bool do_write_frame(int base_bus, uint64_t base_addr, const data_t data[FRAME_SIZE],
                        uint32_t be = FULL_BE, uint64_t addr_step = ADDR_STEP,
                        int *out_cycles = nullptr) {
        int gnt_per_bus[N_PARA] = {};
        int rv_total = 0, curr_group = 0, n_cycles = 0;

        auto drive_group = [&](int g) {
            for (int m = 0; m < N_PARA; ++m) {
                wport[base_bus + m].req.write(true);
                wport[base_bus + m].addr.write(base_addr +
                                               static_cast<uint64_t>(g * N_PARA + m) * addr_step);
                wport[base_bus + m].we.write(true);
                wport[base_bus + m].be.write(be);
                wport[base_bus + m].wdata.write(data[g * N_PARA + m]);
            }
        };
        drive_group(0);

        for (int iter = 0; iter < FRAME_SIZE * kTimeout && rv_total < FRAME_SIZE; ++iter) {
            tick(clk);
            ++n_cycles;
            bool ag = true;
            for (int m = 0; m < N_PARA; ++m) {
                if (gnt_per_bus[m] <= curr_group) {
                    if (wport[base_bus + m].gnt.read()) {
                        ++gnt_per_bus[m];
                        idle_wport(base_bus + m);
                    } else
                        ag = false;
                }
            }
            if (ag && curr_group < FRAME_GRPS - 1) {
                ++curr_group;
                drive_group(curr_group);
            }
            for (int m = 0; m < N_PARA; ++m)
                if (wport[base_bus + m].rvalid.read())
                    ++rv_total;
        }
        for (int m = 0; m < N_PARA; ++m)
            idle_wport(base_bus + m);
        if (out_cycles)
            *out_cycles = n_cycles;
        return rv_total == FRAME_SIZE;
    }

    // 3-phase read: stale drain → fresh capture → invariant restore.
    // addrs[] MUST be consecutive: addrs[m] = addrs[0] + m*ADDR_STEP.
    // INVARIANT: rd_ptr=0 at entry AND exit.
    bool do_read_group(int base_bus, const uint64_t addrs[N_PARA], data_t out[N_PARA]) {
        set_fetch(base_bus, addrs, N_PARA);
        for (int m = 0; m < N_PARA; ++m) {
            rport[base_bus + m].addr.write(addrs[m]);
            rport[base_bus + m].req.write(true);
            rport[base_bus + m].we.write(false);
            rport[base_bus + m].be.write(FULL_BE);
        }

        // Phase 1: drain one complete stale window (FRAME_GRPS*N_PARA rvalid events).
        int stale_rv = 0;
        for (int iter = 0; stale_rv < FRAME_GRPS * N_PARA && iter < FRAME_GRPS * kTimeout; ++iter) {
            tick(clk);
            for (int m = 0; m < N_PARA; ++m)
                if (rport[base_bus + m].rvalid.read())
                    ++stale_rv;
        }
        if (stale_rv < FRAME_GRPS * N_PARA) {
            for (int m = 0; m < N_PARA; ++m)
                idle_rport(base_bus + m);
            return false;
        }
        // Window reset fired; cells now refetch target addrs from bank.

        // Phase 2: capture fresh group-0 rvalids (post-write data).
        bool done[N_PARA] = {};
        int  got          = 0;
        for (int iter = 0; got < N_PARA && iter < N_PARA * kTimeout; ++iter) {
            tick(clk);
            for (int m = 0; m < N_PARA; ++m) {
                if (!done[m] && rport[base_bus + m].rvalid.read()) {
                    out[m]  = rport[base_bus + m].rdata.read();
                    done[m] = true;
                    ++got;
                }
            }
        }

        // Phase 3: drain remaining FRAME_GRPS-1 groups → rd_ptr=0 invariant.
        int rem_rv = 0;
        for (int iter = 0; rem_rv < (FRAME_GRPS - 1) * N_PARA && iter < FRAME_GRPS * kTimeout;
             ++iter) {
            tick(clk);
            for (int m = 0; m < N_PARA; ++m)
                if (rport[base_bus + m].rvalid.read())
                    ++rem_rv;
        }

        for (int m = 0; m < N_PARA; ++m)
            idle_rport(base_bus + m);
        return got == N_PARA;
    }

    static void make_addrs(uint64_t out[N_PARA], uint64_t base) {
        for (int m = 0; m < N_PARA; ++m)
            out[m] = base + static_cast<uint64_t>(m) * ADDR_STEP;
    }
    static void fill_frame(data_t out[FRAME_SIZE], data_t val) {
        for (int i = 0; i < FRAME_SIZE; ++i)
            out[i] = val;
    }
    static void seq_frame(data_t out[FRAME_SIZE], uint32_t base_val) {
        for (int i = 0; i < FRAME_SIZE; ++i)
            out[i] = make_row<data_t>(base_val + static_cast<uint32_t>(i));
    }

    void run() {
        // ── T01 ──────────────────────────────────────────────────────────
        std::puts("\n=== T01: Reset — all output ports deasserted ===");
        do_reset();
        bool all_rg = true, all_rr = true, all_wg = true, all_wr = true;
        for (int m = 0; m < NR_TOTAL; ++m) {
            all_rg &= !rport[m].gnt.read();
            all_rr &= !rport[m].rvalid.read();
        }
        for (int m = 0; m < NW_TOTAL; ++m) {
            all_wg &= !wport[m].gnt.read();
            all_wr &= !wport[m].rvalid.read();
        }
        CHECK(all_rg, "T01a rport_gnt_o[*]=0 after reset");
        CHECK(all_rr, "T01b rport_rvalid_o[*]=0 after reset");
        CHECK(all_wg, "T01c wport_gnt_o[*]=0 after reset");
        CHECK(all_wr, "T01d wport_rvalid_o[*]=0 after reset");

        // ── T02 ──────────────────────────────────────────────────────────
        std::puts("\n=== T02: Write frame then read — data integrity ===");
        do_reset();
        data_t t02_frame[FRAME_SIZE];
        seq_frame(t02_frame, 0xBEEF0000U);
        CHECK(do_write_frame(WBUS, kBase, t02_frame), "T02a write frame completed");
        uint64_t t02_addrs[N_PARA];
        make_addrs(t02_addrs, kBase);
        data_t t02_out[N_PARA];
        CHECK(do_read_group(RBUS, t02_addrs, t02_out), "T02b all N_PARA reads completed");
        bool t02_ok = true;
        for (int m = 0; m < N_PARA; ++m) {
            if (!(t02_out[m] == t02_frame[m])) {
                std::printf("  FAIL  T02: mismatch m=%d exp=0x%08x got=0x%08x\n", m,
                            (uint32_t)t02_frame[m].range(31, 0).to_uint(),
                            (uint32_t)t02_out[m].range(31, 0).to_uint());
                t02_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        if (t02_ok)
            std::puts("  PASS  T02c first N_PARA read-backs match written data");

        // ── T03 ──────────────────────────────────────────────────────────
        std::puts("\n=== T03: Multi-read same address — all N_PARA buses ===");
        do_reset();
        data_t t03_frame[FRAME_SIZE];
        fill_frame(t03_frame, make_row<data_t>(0xCAFEBABEU));
        CHECK(do_write_frame(WBUS, kBase, t03_frame), "T03 setup write frame");
        // All entries equal 0xcafebabe; cell[m] gets addr[0]+m*ADDR_STEP → same value.
        uint64_t t03_addrs[N_PARA];
        for (int m = 0; m < N_PARA; ++m)
            t03_addrs[m] = kBase;
        data_t t03_out[N_PARA];
        CHECK(do_read_group(RBUS, t03_addrs, t03_out), "T03a reads completed");
        bool t03_ok = true;
        for (int m = 0; m < N_PARA; ++m)
            t03_ok &= (t03_out[m] == t03_frame[0]);
        CHECK(t03_ok, "T03b all same-address read data values correct");

        // ── T04 ──────────────────────────────────────────────────────────
        std::puts("\n=== T04: Multi-read distinct consecutive addresses ===");
        do_reset();
        data_t t04_frame[FRAME_SIZE];
        for (int i = 0; i < FRAME_SIZE; ++i)
            t04_frame[i] = make_row<data_t>(0x10000000U * static_cast<uint32_t>((i % N_PARA) + 1));
        CHECK(do_write_frame(WBUS, kBase, t04_frame), "T04 setup write frame");
        uint64_t t04_addrs[N_PARA];
        make_addrs(t04_addrs, kBase);
        data_t t04_out[N_PARA];
        CHECK(do_read_group(RBUS, t04_addrs, t04_out), "T04a reads completed");
        bool t04_ok = true;
        for (int m = 0; m < N_PARA; ++m)
            t04_ok &= (t04_out[m] == t04_frame[m]);
        CHECK(t04_ok, "T04b all distinct-address read data values correct");

        // ── T05 ──────────────────────────────────────────────────────────
        std::puts("\n=== T05: 64-address sequential write-then-read ===");
        do_reset();
        static constexpr int N_FRAMES5 = 2;
        static constexpr int N_ADDRS5  = N_FRAMES5 * FRAME_SIZE; // 64
        data_t               t05_data[N_ADDRS5];
        for (int i = 0; i < N_ADDRS5; ++i)
            t05_data[i] = make_row<data_t>(0x01010101U * static_cast<uint32_t>(i + 1));
        for (int f = 0; f < N_FRAMES5; ++f)
            CHECK(do_write_frame(WBUS, kBase + static_cast<uint64_t>(f) * FRAME_SIZE * ADDR_STEP,
                                 t05_data + f * FRAME_SIZE),
                  "T05 write frame");

        bool                 t05_ok    = true;
        static constexpr int N_GROUPS5 = N_ADDRS5 / N_PARA; // 16
        for (int g = 0; g < N_GROUPS5; ++g) {
            uint64_t a5[N_PARA];
            make_addrs(a5, kBase + static_cast<uint64_t>(g) * N_PARA * ADDR_STEP);
            data_t o5[N_PARA];
            if (!do_read_group(RBUS, a5, o5)) {
                std::printf("  FAIL  T05: read group %d timed out\n", g);
                t05_ok = false;
                g_fail += N_PARA;
                continue;
            }
            for (int m = 0; m < N_PARA; ++m) {
                if (!(o5[m] == t05_data[g * N_PARA + m])) {
                    std::printf("  FAIL  T05: mismatch i=%d addr=0x%03llx\n", g * N_PARA + m,
                                (unsigned long long)a5[m]);
                    t05_ok = false;
                    ++g_fail;
                } else {
                    ++g_pass;
                }
            }
        }
        if (t05_ok)
            std::puts("  PASS  T05 all 64 addresses read back correctly");

        // ── T06 ──────────────────────────────────────────────────────────
        std::puts("\n=== T06: Overwrite — second write to same addresses ===");
        do_reset();
        data_t t06_A[FRAME_SIZE], t06_B[FRAME_SIZE];
        fill_frame(t06_A, make_row<data_t>(0xDEADBEEFU));
        fill_frame(t06_B, make_row<data_t>(0x01234567U));
        CHECK(do_write_frame(WBUS, kBase, t06_A), "T06a first write frame");
        CHECK(do_write_frame(WBUS, kBase, t06_B), "T06b second write frame (overwrite)");
        uint64_t t06_addrs[N_PARA];
        make_addrs(t06_addrs, kBase);
        data_t t06_out[N_PARA];
        CHECK(do_read_group(RBUS, t06_addrs, t06_out), "T06c read-back completed");
        bool t06_ok = true;
        for (int m = 0; m < N_PARA; ++m) {
            if (!(t06_out[m] == t06_B[m])) {
                std::printf("  FAIL  T06: overwrite not reflected at bus %d\n", m);
                t06_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        if (t06_ok)
            std::puts("  PASS  T06d all addresses return second (overwrite) value");

        // ── T07 ──────────────────────────────────────────────────────────
        std::puts("\n=== T07: Alternating bit patterns 0xAA/0x55 ===");
        do_reset();
        data_t t07_aa[FRAME_SIZE], t07_55[FRAME_SIZE];
        fill_frame(t07_aa, make_row<data_t>(0xAAAAAAAAU));
        fill_frame(t07_55, make_row<data_t>(0x55555555U));
        CHECK(do_write_frame(WBUS, kBase, t07_aa), "T07a write 0xAA frame");
        CHECK(do_write_frame(WBUS, kBase + FRAME_SIZE * ADDR_STEP, t07_55),
              "T07b write 0x55 frame");
        uint64_t t07_aa_a[N_PARA];
        make_addrs(t07_aa_a, kBase);
        uint64_t t07_55_a[N_PARA];
        make_addrs(t07_55_a, kBase + FRAME_SIZE * ADDR_STEP);
        data_t t07_oo[N_PARA], t07_o5[N_PARA];
        CHECK(do_read_group(RBUS, t07_aa_a, t07_oo), "T07c 0xAA reads completed");
        CHECK(do_read_group(RBUS, t07_55_a, t07_o5), "T07d 0x55 reads completed");
        bool t07_ok = true;
        for (int m = 0; m < N_PARA; ++m) {
            if (!(t07_oo[m] == t07_aa[0])) {
                std::printf("  FAIL  T07: 0xAA mismatch m=%d\n", m);
                t07_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
            if (!(t07_o5[m] == t07_55[0])) {
                std::printf("  FAIL  T07: 0x55 mismatch m=%d\n", m);
                t07_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        if (t07_ok)
            std::puts("  PASS  T07e all alternating-pattern reads correct");

        // ── T08 ──────────────────────────────────────────────────────────
        std::puts("\n=== T08: Cross-address independence — frame B does not corrupt frame A ===");
        do_reset();
        static constexpr uint64_t BASE_A8 = kBase;
        static constexpr uint64_t BASE_B8 = kBase + static_cast<uint64_t>(FRAME_SIZE) * ADDR_STEP;
        data_t                    t08_A[FRAME_SIZE], t08_B[FRAME_SIZE];
        seq_frame(t08_A, 0xA0000001U);
        seq_frame(t08_B, 0xB0000001U);
        CHECK(do_write_frame(WBUS, BASE_A8, t08_A), "T08a write frame A");
        CHECK(do_write_frame(WBUS, BASE_B8, t08_B), "T08b write frame B");
        uint64_t t08_addrs[N_PARA];
        make_addrs(t08_addrs, BASE_A8);
        data_t t08_out[N_PARA];
        CHECK(do_read_group(RBUS, t08_addrs, t08_out), "T08c frame-A read-back");
        bool t08_ok = true;
        for (int m = 0; m < N_PARA; ++m) {
            if (!(t08_out[m] == t08_A[m])) {
                std::printf("  FAIL  T08: frame-A addr 0x%03llx corrupted\n",
                            (unsigned long long)(BASE_A8 + m * ADDR_STEP));
                t08_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        if (t08_ok)
            std::puts("  PASS  T08d frame-A data unchanged after frame-B write");

        // ── T09 ──────────────────────────────────────────────────────────
        std::puts("\n=== T09: Port group isolation — WR2/RD2 does not leak to other groups ===");
        do_reset();
        data_t t09_frame[FRAME_SIZE];
        fill_frame(t09_frame, make_row<data_t>(0xFEDCBA98U));

        // Write via WR2 while monitoring other wport groups for leakage
        int  g9[N_PARA] = {}, rv9 = 0, grp9 = 0;
        bool w9_no_leak = true;
        auto drv9       = [&](int g) {
            for (int m = 0; m < N_PARA; ++m) {
                wport[WBUS + m].req.write(true);
                wport[WBUS + m].addr.write(kBase +
                                                 static_cast<uint64_t>(g * N_PARA + m) * ADDR_STEP);
                wport[WBUS + m].we.write(true);
                wport[WBUS + m].be.write(FULL_BE);
                wport[WBUS + m].wdata.write(t09_frame[g * N_PARA + m]);
            }
        };
        drv9(0);
        for (int iter = 0; iter < FRAME_SIZE * kTimeout && rv9 < FRAME_SIZE; ++iter) {
            tick(clk);
            bool ag = true;
            for (int m = 0; m < N_PARA; ++m) {
                if (g9[m] <= grp9) {
                    if (wport[WBUS + m].gnt.read()) {
                        ++g9[m];
                        idle_wport(WBUS + m);
                    } else
                        ag = false;
                }
            }
            if (ag && grp9 < FRAME_GRPS - 1) {
                ++grp9;
                drv9(grp9);
            }
            for (int m = 0; m < N_PARA; ++m)
                if (wport[WBUS + m].rvalid.read())
                    ++rv9;
            for (int m = 0; m < WBUS; ++m)
                w9_no_leak &= !wport[m].rvalid.read() && !wport[m].gnt.read();
            for (int m = WBUS + N_PARA; m < NW_TOTAL; ++m)
                w9_no_leak &= !wport[m].rvalid.read() && !wport[m].gnt.read();
        }
        for (int m = 0; m < N_PARA; ++m)
            idle_wport(WBUS + m);
        CHECK(rv9 == FRAME_SIZE, "T09a WR2 frame write completed");
        CHECK(w9_no_leak, "T09b no other wport group saw rvalid/gnt during WR2 write");

        // Read via all 4 RD2 buses (3-phase) while monitoring other rport groups
        uint64_t t09_addrs[N_PARA];
        make_addrs(t09_addrs, kBase);
        data_t t09_out[N_PARA];
        bool   r09_no_leak = true;

        auto chk_riso = [&]() {
            for (int m = 0; m < RBUS; ++m)
                r09_no_leak &= !rport[m].rvalid.read() && !rport[m].gnt.read();
            for (int m = RBUS + N_PARA; m < NR_TOTAL; ++m)
                r09_no_leak &= !rport[m].rvalid.read() && !rport[m].gnt.read();
        };

        set_fetch(RBUS, t09_addrs, N_PARA);
        for (int m = 0; m < N_PARA; ++m) {
            rport[RBUS + m].addr.write(t09_addrs[m]);
            rport[RBUS + m].req.write(true);
            rport[RBUS + m].we.write(false);
            rport[RBUS + m].be.write(FULL_BE);
        }

        int stale9 = 0;
        for (int iter = 0; stale9 < FRAME_GRPS * N_PARA && iter < FRAME_GRPS * kTimeout; ++iter) {
            tick(clk);
            for (int m = 0; m < N_PARA; ++m)
                if (rport[RBUS + m].rvalid.read())
                    ++stale9;
            chk_riso();
        }
        CHECK(stale9 >= FRAME_GRPS * N_PARA, "T09c RD2 stale window drained");

        bool r9done[N_PARA] = {};
        int  r9got          = 0;
        for (int iter = 0; r9got < N_PARA && iter < N_PARA * kTimeout; ++iter) {
            tick(clk);
            for (int m = 0; m < N_PARA; ++m)
                if (!r9done[m] && rport[RBUS + m].rvalid.read()) {
                    t09_out[m] = rport[RBUS + m].rdata.read();
                    r9done[m]  = true;
                    ++r9got;
                }
            chk_riso();
        }
        CHECK(r9got == N_PARA, "T09d RD2 group read completed");

        int rem9 = 0;
        for (int iter = 0; rem9 < (FRAME_GRPS - 1) * N_PARA && iter < FRAME_GRPS * kTimeout;
             ++iter) {
            tick(clk);
            for (int m = 0; m < N_PARA; ++m)
                if (rport[RBUS + m].rvalid.read())
                    ++rem9;
            chk_riso();
        }
        for (int m = 0; m < N_PARA; ++m)
            idle_rport(RBUS + m);
        CHECK(r09_no_leak, "T09e no other rport group saw rvalid/gnt during RD2 read");

        // ── T10 ──────────────────────────────────────────────────────────
        std::puts("\n=== T10: Two simultaneous write frames (WR2 + WR3) ===");
        do_reset();
        static constexpr uint64_t T10_BASE2 = kBase;
        static constexpr uint64_t T10_BASE3 = kBase + static_cast<uint64_t>(FRAME_SIZE) * ADDR_STEP;
        data_t                    t10_d2[FRAME_SIZE], t10_d3[FRAME_SIZE];
        seq_frame(t10_d2, 0x20000001U);
        seq_frame(t10_d3, 0x30000001U);

        int  gnt2[N_PARA] = {}, gnt3[N_PARA] = {}, rv2 = 0, rv3 = 0, grp2 = 0, grp3 = 0;
        auto drv2 = [&](int g) {
            for (int m = 0; m < N_PARA; ++m) {
                wport[WBUS + m].req.write(true);
                wport[WBUS + m].addr.write(T10_BASE2 +
                                           static_cast<uint64_t>(g * N_PARA + m) * ADDR_STEP);
                wport[WBUS + m].we.write(true);
                wport[WBUS + m].be.write(FULL_BE);
                wport[WBUS + m].wdata.write(t10_d2[g * N_PARA + m]);
            }
        };
        auto drv3 = [&](int g) {
            for (int m = 0; m < N_PARA; ++m) {
                wport[WR3B + m].req.write(true);
                wport[WR3B + m].addr.write(T10_BASE3 +
                                           static_cast<uint64_t>(g * N_PARA + m) * ADDR_STEP);
                wport[WR3B + m].we.write(true);
                wport[WR3B + m].be.write(FULL_BE);
                wport[WR3B + m].wdata.write(t10_d3[g * N_PARA + m]);
            }
        };
        drv2(0);
        drv3(0);
        for (int iter = 0;
             iter < 2 * FRAME_SIZE * kTimeout && (rv2 < FRAME_SIZE || rv3 < FRAME_SIZE); ++iter) {
            tick(clk);
            bool a2 = true, a3 = true;
            for (int m = 0; m < N_PARA; ++m) {
                if (gnt2[m] <= grp2) {
                    if (wport[WBUS + m].gnt.read()) {
                        ++gnt2[m];
                        idle_wport(WBUS + m);
                    } else
                        a2 = false;
                }
                if (gnt3[m] <= grp3) {
                    if (wport[WR3B + m].gnt.read()) {
                        ++gnt3[m];
                        idle_wport(WR3B + m);
                    } else
                        a3 = false;
                }
            }
            if (a2 && grp2 < FRAME_GRPS - 1) {
                ++grp2;
                drv2(grp2);
            }
            if (a3 && grp3 < FRAME_GRPS - 1) {
                ++grp3;
                drv3(grp3);
            }
            for (int m = 0; m < N_PARA; ++m) {
                if (wport[WBUS + m].rvalid.read())
                    ++rv2;
                if (wport[WR3B + m].rvalid.read())
                    ++rv3;
            }
        }
        for (int m = 0; m < N_PARA; ++m) {
            idle_wport(WBUS + m);
            idle_wport(WR3B + m);
        }
        CHECK(rv2 == FRAME_SIZE, "T10a WR2 frame writes completed");
        CHECK(rv3 == FRAME_SIZE, "T10b WR3 frame writes completed");

        uint64_t t10_a2[N_PARA];
        make_addrs(t10_a2, T10_BASE2);
        uint64_t t10_a3[N_PARA];
        make_addrs(t10_a3, T10_BASE3);
        data_t t10_o2[N_PARA], t10_o3[N_PARA];
        CHECK(do_read_group(RBUS, t10_a2, t10_o2), "T10c WR2 read-back completed");
        CHECK(do_read_group(RBUS, t10_a3, t10_o3), "T10d WR3 read-back completed");
        bool t10_ok = true;
        for (int m = 0; m < N_PARA; ++m) {
            if (!(t10_o2[m] == t10_d2[m])) {
                std::printf("  FAIL  T10: WR2 mismatch m=%d\n", m);
                t10_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
            if (!(t10_o3[m] == t10_d3[m])) {
                std::printf("  FAIL  T10: WR3 mismatch m=%d\n", m);
                t10_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        if (t10_ok)
            std::puts("  PASS  T10e all 8 addresses from both groups correct");

        // ── T11 ──────────────────────────────────────────────────────────
        std::puts("\n=== T11: addr=0 NOP sentinel — reserved address is a no-op ===");
        do_reset();
        data_t t11_frame[FRAME_SIZE];
        seq_frame(t11_frame, 0xDEAD0000U);
        // base=0 makes addrs[0] land exactly on the reserved sentinel address;
        // addrs[1..N_PARA-1] are real writes at ADDR_STEP, 2*ADDR_STEP, ...
        CHECK(do_write_frame(WBUS, 0, t11_frame), "T11a write frame (entry 0 = addr 0) completed");
        uint64_t t11_addrs[N_PARA];
        make_addrs(t11_addrs, 0);
        data_t t11_out[N_PARA];
        CHECK(do_read_group(RBUS, t11_addrs, t11_out), "T11b read-back completed");
        CHECK(t11_out[0] == data_t(0), "T11c addr=0 reads back zero (write to it was a NOP)");
        bool t11_ok = true;
        for (int m = 1; m < N_PARA; ++m) {
            if (!(t11_out[m] == t11_frame[m])) {
                std::printf("  FAIL  T11: mismatch m=%d\n", m);
                t11_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        if (t11_ok)
            std::puts("  PASS  T11d addresses after the sentinel wrote/read back correctly");

        // ── T12 ──────────────────────────────────────────────────────────
        std::puts("\n=== T12: Byte-enable partial write (end-to-end through TDM) ===");
        do_reset();
        data_t t12_base[FRAME_SIZE];
        fill_frame(t12_base, make_row<data_t>(0xAAAAAAAAU));
        CHECK(do_write_frame(WBUS, kBase, t12_base), "T12a full-BE baseline write");
        data_t t12_partial[FRAME_SIZE];
        fill_frame(t12_partial, make_row<data_t>(0x0000BBBBU));
        // be=0x0003 enables only the low 2 bytes of the 128-bit row (bits[15:0]).
        CHECK(do_write_frame(WBUS, kBase, t12_partial, 0x0003),
              "T12b partial-BE (low 2 bytes) overwrite");
        uint64_t t12_addrs[N_PARA];
        make_addrs(t12_addrs, kBase);
        data_t t12_out[N_PARA];
        CHECK(do_read_group(RBUS, t12_addrs, t12_out), "T12c read-back completed");
        bool t12_ok = true;
        for (int m = 0; m < N_PARA; ++m) {
            const uint32_t want = (t12_base[m].range(31, 0).to_uint() & 0xFFFF0000U) |
                                  (t12_partial[m].range(31, 0).to_uint() & 0x0000FFFFU);
            const uint32_t got = t12_out[m].range(31, 0).to_uint();
            if (want != got) {
                std::printf("  FAIL  T12: mismatch m=%d exp=0x%08x got=0x%08x\n", m, want, got);
                t12_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        if (t12_ok)
            std::puts("  PASS  T12d unselected bytes preserved, selected bytes updated");

        // ── T13 ──────────────────────────────────────────────────────────
        std::puts("\n=== T13: Per-buffer distinct TDM mapping config (map_cfg mux) ===");
        do_reset();
        static constexpr int WR3_IDX = DUT::NUM_RD_BUF + 3; // 8
        static constexpr int RD3_IDX = 3;
        // WR2/RD2 keep the default geometry (R=4,C=4,L=8,mode=0) set by do_reset().
        // WR3/RD3 get a distinct geometry, live simultaneously, to exercise the
        // per-buffer map_cfg_comb() mux in top_tdm.hpp (indexed by arb_req_sel).
        buf_map_r[WR3_IDX].write(2);
        buf_map_c[WR3_IDX].write(2);
        buf_map_l[WR3_IDX].write(4);
        buf_map_sm[WR3_IDX].write(4); // Row_Col_Loop
        buf_map_r[RD3_IDX].write(2);
        buf_map_c[RD3_IDX].write(2);
        buf_map_l[RD3_IDX].write(4);
        buf_map_sm[RD3_IDX].write(4);

        static constexpr uint64_t T13_BASE2 = kBase;
        static constexpr uint64_t T13_BASE3 = kBase + static_cast<uint64_t>(FRAME_SIZE) * ADDR_STEP;
        data_t                    t13_d2[FRAME_SIZE], t13_d3[FRAME_SIZE];
        seq_frame(t13_d2, 0x20A00001U);
        seq_frame(t13_d3, 0x30B00001U);
        CHECK(do_write_frame(WBUS, T13_BASE2, t13_d2), "T13a WR2 write (default geometry)");
        CHECK(do_write_frame(WR3B, T13_BASE3, t13_d3), "T13b WR3 write (distinct geometry)");

        uint64_t t13_a2[N_PARA];
        make_addrs(t13_a2, T13_BASE2);
        uint64_t t13_a3[N_PARA];
        make_addrs(t13_a3, T13_BASE3);
        data_t t13_o2[N_PARA], t13_o3[N_PARA];
        CHECK(do_read_group(RBUS, t13_a2, t13_o2), "T13c RD2 read-back (default geometry)");
        CHECK(do_read_group(DUT::RD3_BASE, t13_a3, t13_o3),
              "T13d RD3 read-back (distinct geometry)");
        bool t13_ok = true;
        for (int m = 0; m < N_PARA; ++m) {
            if (!(t13_o2[m] == t13_d2[m])) {
                std::printf("  FAIL  T13: WR2/RD2 mismatch m=%d\n", m);
                t13_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
            if (!(t13_o3[m] == t13_d3[m])) {
                std::printf("  FAIL  T13: WR3/RD3 mismatch m=%d\n", m);
                t13_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        if (t13_ok)
            std::puts("  PASS  T13e both simultaneously-live geometries round-trip correctly");

        // ── T14 ──────────────────────────────────────────────────────────
        std::puts("\n=== T14: Simultaneous read (RD3) + write (WR2) traffic ===");
        do_reset();
        static constexpr uint64_t T14_RD_BASE = kBase;
        static constexpr uint64_t T14_WR_BASE =
            kBase + static_cast<uint64_t>(FRAME_SIZE) * ADDR_STEP;
        data_t t14_rd_frame[FRAME_SIZE];
        seq_frame(t14_rd_frame, 0x40000001U);
        data_t t14_wr_frame[FRAME_SIZE];
        seq_frame(t14_wr_frame, 0x50000001U);
        // Pre-populate the data RD3 will read, before starting the concurrent phase.
        CHECK(do_write_frame(WR3B, T14_RD_BASE, t14_rd_frame),
              "T14 setup: pre-write RD3 data via WR3");

        // Concurrent phase: WR2 writes a fresh frame while RD3 reads back the
        // pre-written one — both traverse the shared arbiter/mux/TDM/crossbar
        // path in the same cycles.
        int  wgnt[N_PARA] = {}, wrv = 0, wgrp = 0;
        auto drv_w = [&](int g) {
            for (int m = 0; m < N_PARA; ++m) {
                wport[WBUS + m].req.write(true);
                wport[WBUS + m].addr.write(T14_WR_BASE +
                                           static_cast<uint64_t>(g * N_PARA + m) * ADDR_STEP);
                wport[WBUS + m].we.write(true);
                wport[WBUS + m].be.write(FULL_BE);
                wport[WBUS + m].wdata.write(t14_wr_frame[g * N_PARA + m]);
            }
        };
        drv_w(0);

        uint64_t t14_rd_addrs[N_PARA];
        make_addrs(t14_rd_addrs, T14_RD_BASE);
        set_fetch(DUT::RD3_BASE, t14_rd_addrs, N_PARA);
        for (int m = 0; m < N_PARA; ++m) {
            rport[DUT::RD3_BASE + m].addr.write(t14_rd_addrs[m]);
            rport[DUT::RD3_BASE + m].req.write(true);
            rport[DUT::RD3_BASE + m].we.write(false);
            rport[DUT::RD3_BASE + m].be.write(FULL_BE);
        }

        int    rstale = 0, rphase = 0, rgot = 0;
        bool   rdone[N_PARA] = {};
        data_t t14_rd_out[N_PARA];
        for (int iter = 0; (wrv < FRAME_SIZE || rphase < 2) && iter < 4 * FRAME_SIZE * kTimeout;
             ++iter) {
            tick(clk);
            bool ag = true;
            for (int m = 0; m < N_PARA; ++m) {
                if (wgnt[m] <= wgrp) {
                    if (wport[WBUS + m].gnt.read()) {
                        ++wgnt[m];
                        idle_wport(WBUS + m);
                    } else
                        ag = false;
                }
            }
            if (ag && wgrp < FRAME_GRPS - 1) {
                ++wgrp;
                drv_w(wgrp);
            }
            for (int m = 0; m < N_PARA; ++m)
                if (wport[WBUS + m].rvalid.read())
                    ++wrv;

            if (rphase == 0) {
                for (int m = 0; m < N_PARA; ++m)
                    if (rport[DUT::RD3_BASE + m].rvalid.read())
                        ++rstale;
                if (rstale >= FRAME_GRPS * N_PARA)
                    rphase = 1;
            } else if (rphase == 1) {
                for (int m = 0; m < N_PARA; ++m)
                    if (!rdone[m] && rport[DUT::RD3_BASE + m].rvalid.read()) {
                        t14_rd_out[m] = rport[DUT::RD3_BASE + m].rdata.read();
                        rdone[m]      = true;
                        ++rgot;
                    }
                if (rgot >= N_PARA)
                    rphase = 2;
            }
        }
        for (int m = 0; m < N_PARA; ++m) {
            idle_wport(WBUS + m);
            idle_rport(DUT::RD3_BASE + m);
        }

        CHECK(wrv == FRAME_SIZE, "T14a WR2 write completed while RD3 concurrently read");
        CHECK(rgot == N_PARA, "T14b RD3 read completed while WR2 concurrently wrote");
        bool t14_ok = true;
        for (int m = 0; m < N_PARA; ++m) {
            if (!(t14_rd_out[m] == t14_rd_frame[m])) {
                std::printf("  FAIL  T14: RD3 mismatch m=%d\n", m);
                t14_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        if (t14_ok)
            std::puts("  PASS  T14c concurrent RD3 data correct (no corruption from WR2 traffic)");

        uint64_t t14_wr_addrs[N_PARA];
        make_addrs(t14_wr_addrs, T14_WR_BASE);
        data_t t14_wr_out[N_PARA];
        CHECK(do_read_group(RBUS, t14_wr_addrs, t14_wr_out), "T14d WR2 data read-back completed");
        bool t14_wr_ok = true;
        for (int m = 0; m < N_PARA; ++m) {
            if (!(t14_wr_out[m] == t14_wr_frame[m])) {
                std::printf("  FAIL  T14: WR2 mismatch m=%d\n", m);
                t14_wr_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        if (t14_wr_ok)
            std::puts("  PASS  T14e WR2 data correct after concurrent read+write traffic");

        // ── T15 ──────────────────────────────────────────────────────────
        std::puts("\n=== T15: Write — 1 real addr padded with 31 NOPs (full 32-slot window) ===");
        do_reset();
        // The write buffer's window is NUM_BANK=32 cells, filled across
        // FRAME_GRPS groups of N_PARA lanes.  Only one (group,lane) pair
        // carries a real address; every other slot presents addr=0 (NOP) and
        // is expected to be silently skipped by the TDM addr=0 fast-path.
        static constexpr int T15_REAL_GROUP = 3;
        static constexpr int T15_REAL_LANE  = 1;
        // bank.hpp memory persists across the whole test binary (only
        // zero-initialised once, not on reset), so use a base range no
        // other test writes to, letting the "never written" check hold.
        static constexpr uint64_t T15_REAL_ADDR =
            kBase + 5 * static_cast<uint64_t>(FRAME_SIZE) * ADDR_STEP;
        const data_t t15_real_data = make_row<data_t>(0x600DF00DU);

        int  t15_gnt[N_PARA] = {};
        int  t15_rv = 0, t15_grp = 0;
        auto drive_t15 = [&](int g) {
            for (int m = 0; m < N_PARA; ++m) {
                const bool is_real = (g == T15_REAL_GROUP && m == T15_REAL_LANE);
                wport[WBUS + m].req.write(true);
                wport[WBUS + m].addr.write(is_real ? T15_REAL_ADDR : 0);
                wport[WBUS + m].we.write(true);
                wport[WBUS + m].be.write(FULL_BE);
                wport[WBUS + m].wdata.write(is_real ? t15_real_data : data_t(0));
            }
        };
        drive_t15(0);
        for (int iter = 0; iter < FRAME_SIZE * kTimeout && t15_rv < FRAME_SIZE; ++iter) {
            tick(clk);
            bool ag = true;
            for (int m = 0; m < N_PARA; ++m) {
                if (t15_gnt[m] <= t15_grp) {
                    if (wport[WBUS + m].gnt.read()) {
                        ++t15_gnt[m];
                        idle_wport(WBUS + m);
                    } else
                        ag = false;
                }
            }
            if (ag && t15_grp < FRAME_GRPS - 1) {
                ++t15_grp;
                drive_t15(t15_grp);
            }
            for (int m = 0; m < N_PARA; ++m)
                if (wport[WBUS + m].rvalid.read())
                    ++t15_rv;
        }
        for (int m = 0; m < N_PARA; ++m)
            idle_wport(WBUS + m);
        CHECK(t15_rv == FRAME_SIZE, "T15a write completes: 31 NOP slots + 1 real addr");

        // Read-side lane index is unrelated to which write-side lane (fill
        // group/lane) issued the address — addressing is purely by value.
        // make_addrs() puts T15_REAL_ADDR at addrs[0]; addrs[1..] are
        // higher, never-written addresses.
        uint64_t t15_addrs[N_PARA];
        make_addrs(t15_addrs, T15_REAL_ADDR);
        data_t t15_out[N_PARA];
        CHECK(do_read_group(RBUS, t15_addrs, t15_out), "T15b read-back completed");
        CHECK(t15_out[0] == t15_real_data, "T15c the single real address reads back correctly");
        bool t15_ok = true;
        for (int m = 1; m < N_PARA; ++m)
            t15_ok &= (t15_out[m] == data_t(0));
        CHECK(t15_ok, "T15d neighboring addresses (never written) remain zero");

        // ── T16 ──────────────────────────────────────────────────────────
        std::puts("\n=== T16: Read — 1 real fetch addr padded with 31 NOP slots (full window) ===");
        do_reset();
        data_t t16_frame[FRAME_SIZE];
        fill_frame(t16_frame, make_row<data_t>(0xFEED1234U));
        CHECK(do_write_frame(WBUS, kBase, t16_frame), "T16 setup: write baseline data via WR2");

        // Read buffer cells 4..31 are permanently tied to addr=0 (NOP) by
        // construction (see top_tdm.hpp bind for RD2). Feeding addr=0 to
        // lanes 1-3 too makes this a genuine 1-real + 31-NOP full window.
        uint64_t t16_addrs[N_PARA] = {kBase, 0, 0, 0};
        data_t   t16_out[N_PARA];
        CHECK(do_read_group(RBUS, t16_addrs, t16_out),
              "T16a read completes: 31 NOP slots + 1 real fetch addr");
        CHECK(t16_out[0] == t16_frame[0], "T16b the single real address reads back correctly");
        bool t16_ok = true;
        for (int m = 1; m < N_PARA; ++m)
            t16_ok &= (t16_out[m] == data_t(0));
        CHECK(t16_ok, "T16c NOP lanes read back zero");

        // ── T17 ──────────────────────────────────────────────────────────
        std::puts("\n=== T17: active_mode=1 (2 port groups / 8 beats) — WR1/RD1 ===");
        do_reset();
        buf_mode[DUT::NUM_RD_BUF + 1].write(1);       // WR1 (buf_w1): 2 active port groups
        buf_mode[1].write(1);                         // RD1 (buf_r1): 2 active port groups
        static constexpr int T17_BEATS  = 2 * N_PARA; // 8
        static constexpr int T17_GROUPS = FRAME_SIZE / T17_BEATS; // 4
        data_t               t17_frame[FRAME_SIZE];
        seq_frame(t17_frame, 0x17000001U);

        int  t17_gnt[T17_BEATS] = {};
        int  t17_rv = 0, t17_grp = 0;
        auto drive_t17 = [&](int g) {
            for (int m = 0; m < T17_BEATS; ++m) {
                wport[DUT::WR1_BASE + m].req.write(true);
                wport[DUT::WR1_BASE + m].addr.write(
                    kBase + static_cast<uint64_t>(g * T17_BEATS + m) * ADDR_STEP);
                wport[DUT::WR1_BASE + m].we.write(true);
                wport[DUT::WR1_BASE + m].be.write(FULL_BE);
                wport[DUT::WR1_BASE + m].wdata.write(t17_frame[g * T17_BEATS + m]);
            }
        };
        drive_t17(0);
        for (int iter = 0; iter < FRAME_SIZE * kTimeout && t17_rv < FRAME_SIZE; ++iter) {
            tick(clk);
            bool ag = true;
            for (int m = 0; m < T17_BEATS; ++m) {
                if (t17_gnt[m] <= t17_grp) {
                    if (wport[DUT::WR1_BASE + m].gnt.read()) {
                        ++t17_gnt[m];
                        idle_wport(DUT::WR1_BASE + m);
                    } else
                        ag = false;
                }
            }
            if (ag && t17_grp < T17_GROUPS - 1) {
                ++t17_grp;
                drive_t17(t17_grp);
            }
            for (int m = 0; m < T17_BEATS; ++m)
                if (wport[DUT::WR1_BASE + m].rvalid.read())
                    ++t17_rv;
        }
        for (int m = 0; m < T17_BEATS; ++m)
            idle_wport(DUT::WR1_BASE + m);
        CHECK(t17_rv == FRAME_SIZE, "T17a WR1 write completes with active_mode=1 (8 beats/group)");

        uint64_t t17_rd_addrs[T17_BEATS];
        for (int m = 0; m < T17_BEATS; ++m)
            t17_rd_addrs[m] = kBase + static_cast<uint64_t>(m) * ADDR_STEP;
        set_fetch(DUT::RD1_BASE, t17_rd_addrs, T17_BEATS);
        for (int m = 0; m < T17_BEATS; ++m) {
            rport[DUT::RD1_BASE + m].addr.write(t17_rd_addrs[m]);
            rport[DUT::RD1_BASE + m].req.write(true);
            rport[DUT::RD1_BASE + m].we.write(false);
            rport[DUT::RD1_BASE + m].be.write(FULL_BE);
        }
        int t17_stale = 0;
        for (int iter = 0; t17_stale < T17_GROUPS * T17_BEATS && iter < T17_GROUPS * kTimeout;
             ++iter) {
            tick(clk);
            for (int m = 0; m < T17_BEATS; ++m)
                if (rport[DUT::RD1_BASE + m].rvalid.read())
                    ++t17_stale;
        }
        CHECK(t17_stale >= T17_GROUPS * T17_BEATS, "T17b RD1 stale window drained (active_mode=1)");

        bool   t17_done[T17_BEATS] = {};
        int    t17_got             = 0;
        data_t t17_out[T17_BEATS];
        for (int iter = 0; t17_got < T17_BEATS && iter < T17_BEATS * kTimeout; ++iter) {
            tick(clk);
            for (int m = 0; m < T17_BEATS; ++m)
                if (!t17_done[m] && rport[DUT::RD1_BASE + m].rvalid.read()) {
                    t17_out[m]  = rport[DUT::RD1_BASE + m].rdata.read();
                    t17_done[m] = true;
                    ++t17_got;
                }
        }
        CHECK(t17_got == T17_BEATS, "T17c RD1 group read completed (active_mode=1)");
        int t17_rem = 0;
        for (int iter = 0; t17_rem < (T17_GROUPS - 1) * T17_BEATS && iter < T17_GROUPS * kTimeout;
             ++iter) {
            tick(clk);
            for (int m = 0; m < T17_BEATS; ++m)
                if (rport[DUT::RD1_BASE + m].rvalid.read())
                    ++t17_rem;
        }
        for (int m = 0; m < T17_BEATS; ++m)
            idle_rport(DUT::RD1_BASE + m);
        bool t17_ok = true;
        for (int m = 0; m < T17_BEATS; ++m) {
            if (!(t17_out[m] == t17_frame[m])) {
                std::printf("  FAIL  T17: mismatch m=%d\n", m);
                t17_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        if (t17_ok)
            std::puts("  PASS  T17d all 8 addresses read back correctly (active_mode=1)");

        // ── T18 ──────────────────────────────────────────────────────────
        std::puts("\n=== T18: active_mode=2 (4 port groups / 16 beats) — WR0/RD0 ===");
        do_reset();
        buf_mode[DUT::NUM_RD_BUF + 0].write(2);       // WR0 (buf_w0): 4 active port groups
        buf_mode[0].write(2);                         // RD0 (buf_r0): 4 active port groups
        static constexpr int T18_BEATS  = 4 * N_PARA; // 16
        static constexpr int T18_GROUPS = FRAME_SIZE / T18_BEATS; // 2
        data_t               t18_frame[FRAME_SIZE];
        seq_frame(t18_frame, 0x18000001U);

        int  t18_gnt[T18_BEATS] = {};
        int  t18_rv = 0, t18_grp = 0;
        auto drive_t18 = [&](int g) {
            for (int m = 0; m < T18_BEATS; ++m) {
                wport[0 + m].req.write(true); // WR0_BASE == 0
                wport[0 + m].addr.write(kBase +
                                        static_cast<uint64_t>(g * T18_BEATS + m) * ADDR_STEP);
                wport[0 + m].we.write(true);
                wport[0 + m].be.write(FULL_BE);
                wport[0 + m].wdata.write(t18_frame[g * T18_BEATS + m]);
            }
        };
        drive_t18(0);
        for (int iter = 0; iter < FRAME_SIZE * kTimeout && t18_rv < FRAME_SIZE; ++iter) {
            tick(clk);
            bool ag = true;
            for (int m = 0; m < T18_BEATS; ++m) {
                if (t18_gnt[m] <= t18_grp) {
                    if (wport[0 + m].gnt.read()) {
                        ++t18_gnt[m];
                        idle_wport(0 + m);
                    } else
                        ag = false;
                }
            }
            if (ag && t18_grp < T18_GROUPS - 1) {
                ++t18_grp;
                drive_t18(t18_grp);
            }
            for (int m = 0; m < T18_BEATS; ++m)
                if (wport[0 + m].rvalid.read())
                    ++t18_rv;
        }
        for (int m = 0; m < T18_BEATS; ++m)
            idle_wport(0 + m);
        CHECK(t18_rv == FRAME_SIZE, "T18a WR0 write completes with active_mode=2 (16 beats/group)");

        uint64_t t18_rd_addrs[T18_BEATS];
        for (int m = 0; m < T18_BEATS; ++m)
            t18_rd_addrs[m] = kBase + static_cast<uint64_t>(m) * ADDR_STEP;
        set_fetch(0, t18_rd_addrs, T18_BEATS);
        for (int m = 0; m < T18_BEATS; ++m) {
            rport[0 + m].addr.write(t18_rd_addrs[m]);
            rport[0 + m].req.write(true);
            rport[0 + m].we.write(false);
            rport[0 + m].be.write(FULL_BE);
        }
        int t18_stale = 0;
        for (int iter = 0; t18_stale < T18_GROUPS * T18_BEATS && iter < T18_GROUPS * kTimeout;
             ++iter) {
            tick(clk);
            for (int m = 0; m < T18_BEATS; ++m)
                if (rport[0 + m].rvalid.read())
                    ++t18_stale;
        }
        CHECK(t18_stale >= T18_GROUPS * T18_BEATS, "T18b RD0 stale window drained (active_mode=2)");

        bool   t18_done[T18_BEATS] = {};
        int    t18_got             = 0;
        data_t t18_out[T18_BEATS];
        for (int iter = 0; t18_got < T18_BEATS && iter < T18_BEATS * kTimeout; ++iter) {
            tick(clk);
            for (int m = 0; m < T18_BEATS; ++m)
                if (!t18_done[m] && rport[0 + m].rvalid.read()) {
                    t18_out[m]  = rport[0 + m].rdata.read();
                    t18_done[m] = true;
                    ++t18_got;
                }
        }
        CHECK(t18_got == T18_BEATS, "T18c RD0 group read completed (active_mode=2)");
        int t18_rem = 0;
        for (int iter = 0; t18_rem < (T18_GROUPS - 1) * T18_BEATS && iter < T18_GROUPS * kTimeout;
             ++iter) {
            tick(clk);
            for (int m = 0; m < T18_BEATS; ++m)
                if (rport[0 + m].rvalid.read())
                    ++t18_rem;
        }
        for (int m = 0; m < T18_BEATS; ++m)
            idle_rport(0 + m);
        bool t18_ok = true;
        for (int m = 0; m < T18_BEATS; ++m) {
            if (!(t18_out[m] == t18_frame[m])) {
                std::printf("  FAIL  T18: mismatch m=%d\n", m);
                t18_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        if (t18_ok)
            std::puts("  PASS  T18d all 16 addresses read back correctly (active_mode=2)");

        // ── T19 ──────────────────────────────────────────────────────────
        std::puts("\n=== T19: Byte-enable be=0x0000 is a true no-op ===");
        do_reset();
        data_t t19_base[FRAME_SIZE];
        fill_frame(t19_base, make_row<data_t>(0x12345678U));
        CHECK(do_write_frame(WBUS, kBase, t19_base), "T19a baseline write");
        data_t t19_junk[FRAME_SIZE];
        fill_frame(t19_junk, make_row<data_t>(0xFFFFFFFFU));
        CHECK(do_write_frame(WBUS, kBase, t19_junk, 0x0000),
              "T19b be=0 write (should change nothing)");
        uint64_t t19_addrs[N_PARA];
        make_addrs(t19_addrs, kBase);
        data_t t19_out[N_PARA];
        CHECK(do_read_group(RBUS, t19_addrs, t19_out), "T19c read-back completed");
        bool t19_ok = true;
        for (int m = 0; m < N_PARA; ++m) {
            if (!(t19_out[m] == t19_base[m])) {
                std::printf("  FAIL  T19: mismatch m=%d\n", m);
                t19_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        if (t19_ok)
            std::puts("  PASS  T19d be=0 write left data unchanged");

        // ── T20 ──────────────────────────────────────────────────────────
        std::puts("\n=== T20: All 9 buffers active simultaneously (full arbiter stress) ===");
        do_reset();
        // Fresh, never-before-used address ranges (bank memory persists across
        // the whole binary — see T15's note); one range per buffer.
        static constexpr uint64_t T20_BASE_R0 =
            kBase + 10 * static_cast<uint64_t>(FRAME_SIZE) * ADDR_STEP;
        static constexpr uint64_t T20_BASE_R1 =
            kBase + 11 * static_cast<uint64_t>(FRAME_SIZE) * ADDR_STEP;
        static constexpr uint64_t T20_BASE_R2 =
            kBase + 12 * static_cast<uint64_t>(FRAME_SIZE) * ADDR_STEP;
        static constexpr uint64_t T20_BASE_R3 =
            kBase + 13 * static_cast<uint64_t>(FRAME_SIZE) * ADDR_STEP;
        static constexpr uint64_t T20_BASE_R4 =
            kBase + 14 * static_cast<uint64_t>(FRAME_SIZE) * ADDR_STEP;
        static constexpr uint64_t T20_BASE_W0 =
            kBase + 15 * static_cast<uint64_t>(FRAME_SIZE) * ADDR_STEP;
        static constexpr uint64_t T20_BASE_W1 =
            kBase + 16 * static_cast<uint64_t>(FRAME_SIZE) * ADDR_STEP;
        static constexpr uint64_t T20_BASE_W2 =
            kBase + 17 * static_cast<uint64_t>(FRAME_SIZE) * ADDR_STEP;
        static constexpr uint64_t T20_BASE_W3 =
            kBase + 18 * static_cast<uint64_t>(FRAME_SIZE) * ADDR_STEP;

        data_t t20_r0[FRAME_SIZE], t20_r1[FRAME_SIZE], t20_r2[FRAME_SIZE], t20_r3[FRAME_SIZE],
            t20_r4[FRAME_SIZE];
        seq_frame(t20_r0, 0xA0000001U);
        seq_frame(t20_r1, 0xA1000001U);
        seq_frame(t20_r2, 0xA2000001U);
        seq_frame(t20_r3, 0xA3000001U);
        seq_frame(t20_r4, 0xA4000001U);
        // Setup (sequential): pre-populate the data each read buffer will fetch.
        CHECK(do_write_frame(0, T20_BASE_R0, t20_r0), "T20 setup: RD0 data via WR0");
        CHECK(do_write_frame(DUT::WR1_BASE, T20_BASE_R1, t20_r1), "T20 setup: RD1 data via WR1");
        CHECK(do_write_frame(WBUS, T20_BASE_R2, t20_r2), "T20 setup: RD2 data via WR2");
        CHECK(do_write_frame(WBUS, T20_BASE_R3, t20_r3), "T20 setup: RD3 data via WR2");
        CHECK(do_write_frame(WR3B, T20_BASE_R4, t20_r4), "T20 setup: RD4 data via WR3");

        data_t t20_w0[FRAME_SIZE], t20_w1[FRAME_SIZE], t20_w2[FRAME_SIZE], t20_w3[FRAME_SIZE];
        seq_frame(t20_w0, 0xB0000001U);
        seq_frame(t20_w1, 0xB1000001U);
        seq_frame(t20_w2, 0xB2000001U);
        seq_frame(t20_w3, 0xB3000001U);

        static const int      WBASES[4]     = {0, DUT::WR1_BASE, WBUS, WR3B};
        static const uint64_t WBASE_ADDR[4] = {T20_BASE_W0, T20_BASE_W1, T20_BASE_W2, T20_BASE_W3};
        data_t               *WDATA[4]      = {t20_w0, t20_w1, t20_w2, t20_w3};

        static const int      RBASES[5] = {0, DUT::RD1_BASE, RBUS, DUT::RD3_BASE, DUT::RD4_BASE};
        static const uint64_t RBASE_ADDR[5] = {T20_BASE_R0, T20_BASE_R1, T20_BASE_R2, T20_BASE_R3,
                                               T20_BASE_R4};
        data_t               *RDATA_EXP[5]  = {t20_r0, t20_r1, t20_r2, t20_r3, t20_r4};

        int  w_gnt[4][N_PARA] = {};
        int  w_rv[4]          = {};
        int  w_grp[4]         = {};
        auto drive_w          = [&](int b, int g) {
            for (int m = 0; m < N_PARA; ++m) {
                wport[WBASES[b] + m].req.write(true);
                wport[WBASES[b] + m].addr.write(WBASE_ADDR[b] +
                                                         static_cast<uint64_t>(g * N_PARA + m) * ADDR_STEP);
                wport[WBASES[b] + m].we.write(true);
                wport[WBASES[b] + m].be.write(FULL_BE);
                wport[WBASES[b] + m].wdata.write(WDATA[b][g * N_PARA + m]);
            }
        };
        for (int b = 0; b < 4; ++b)
            drive_w(b, 0);

        for (int b = 0; b < 5; ++b) {
            uint64_t addrs[N_PARA];
            for (int m = 0; m < N_PARA; ++m)
                addrs[m] = RBASE_ADDR[b] + static_cast<uint64_t>(m) * ADDR_STEP;
            set_fetch(RBASES[b], addrs, N_PARA);
            for (int m = 0; m < N_PARA; ++m) {
                rport[RBASES[b] + m].addr.write(addrs[m]);
                rport[RBASES[b] + m].req.write(true);
                rport[RBASES[b] + m].we.write(false);
                rport[RBASES[b] + m].be.write(FULL_BE);
            }
        }
        int    r_stale[5] = {}, r_phase[5] = {}, r_got[5] = {};
        bool   r_done[5][N_PARA] = {};
        data_t r_out[5][N_PARA];

        bool w_all_done = false, r_all_done = false;
        for (int iter = 0; (!w_all_done || !r_all_done) && iter < 4 * FRAME_SIZE * kTimeout;
             ++iter) {
            tick(clk);
            w_all_done = true;
            for (int b = 0; b < 4; ++b) {
                bool ag = true;
                for (int m = 0; m < N_PARA; ++m) {
                    if (w_gnt[b][m] <= w_grp[b]) {
                        if (wport[WBASES[b] + m].gnt.read()) {
                            ++w_gnt[b][m];
                            idle_wport(WBASES[b] + m);
                        } else
                            ag = false;
                    }
                }
                if (ag && w_grp[b] < FRAME_GRPS - 1) {
                    ++w_grp[b];
                    drive_w(b, w_grp[b]);
                }
                for (int m = 0; m < N_PARA; ++m)
                    if (wport[WBASES[b] + m].rvalid.read())
                        ++w_rv[b];
                if (w_rv[b] < FRAME_SIZE)
                    w_all_done = false;
            }
            r_all_done = true;
            for (int b = 0; b < 5; ++b) {
                if (r_phase[b] == 0) {
                    for (int m = 0; m < N_PARA; ++m)
                        if (rport[RBASES[b] + m].rvalid.read())
                            ++r_stale[b];
                    if (r_stale[b] >= FRAME_GRPS * N_PARA)
                        r_phase[b] = 1;
                } else if (r_phase[b] == 1) {
                    for (int m = 0; m < N_PARA; ++m)
                        if (!r_done[b][m] && rport[RBASES[b] + m].rvalid.read()) {
                            r_out[b][m]  = rport[RBASES[b] + m].rdata.read();
                            r_done[b][m] = true;
                            ++r_got[b];
                        }
                    if (r_got[b] >= N_PARA)
                        r_phase[b] = 2;
                }
                if (r_phase[b] < 2)
                    r_all_done = false;
            }
        }
        for (int b = 0; b < 4; ++b)
            for (int m = 0; m < N_PARA; ++m)
                idle_wport(WBASES[b] + m);

        // Restore the "rd_ptr=0 at do_read_group() entry" invariant (see
        // this file's header comment) for every read buffer T20d's
        // write-verify will drive via do_read_group(). The capture loop
        // above only waits for each buffer's r_phase to reach 2 (its
        // N_PARA target values captured) — not for a full window drain
        // back to rd_ptr=0 — but ports_req[b] is still asserted here (only
        // idle_rport() below deasserts it), so draining continues exactly
        // like do_read_group()'s own "Phase 3: invariant restore". Whether
        // that drain had already finished by the time the loop's overall
        // exit condition (w_all_done && r_all_done) was satisfied was
        // incidental to how fast the arbiter happened to service the other
        // 8 buffers, not something the loop actually enforced — a faster
        // arbiter (or simply a different service order) can legitimately
        // reach r_all_done before every buffer's drain completes, leaving
        // do_read_group() reading from wherever the window's current drain
        // position is instead of the caller's target address.
        for (int b = 0; b < 5; ++b) {
            bool drained = false;
            for (int iter = 0; !drained && iter < FRAME_SIZE * kTimeout; ++iter) {
                tick(clk);
                drained = false;
                switch (b) {
                case 0:
                    drained = dut->buf_r0.snapshot().rd_ptr == 0;
                    break;
                case 1:
                    drained = dut->buf_r1.snapshot().rd_ptr == 0;
                    break;
                case 2:
                    drained = dut->buf_r2.snapshot().rd_ptr == 0;
                    break;
                case 3:
                    drained = dut->buf_r3.snapshot().rd_ptr == 0;
                    break;
                case 4:
                    drained = dut->buf_r4.snapshot().rd_ptr == 0;
                    break;
                }
            }
            // Idle THIS buffer's port immediately, not after checking the
            // rest — otherwise its req stays asserted while we drain the
            // other 4, and it drifts straight back away from rd_ptr=0.
            for (int m = 0; m < N_PARA; ++m)
                idle_rport(RBASES[b] + m);
            char lbl[80];
            std::snprintf(lbl, sizeof(lbl), "T20 read buffer %d redrained to rd_ptr=0", b);
            CHECK(drained, lbl);
        }

        bool t20_w_ok = true;
        for (int b = 0; b < 4; ++b) {
            if (w_rv[b] != FRAME_SIZE) {
                std::printf("  FAIL  T20: write buffer %d incomplete (rv=%d)\n", b, w_rv[b]);
                t20_w_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        CHECK(t20_w_ok, "T20a all 4 write buffers completed concurrently");

        bool t20_r_ok = true;
        for (int b = 0; b < 5; ++b) {
            if (r_got[b] != N_PARA) {
                std::printf("  FAIL  T20: read buffer %d incomplete (got=%d)\n", b, r_got[b]);
                t20_r_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        CHECK(t20_r_ok, "T20b all 5 read buffers completed concurrently");

        bool t20_data_ok = true;
        for (int b = 0; b < 5; ++b)
            for (int m = 0; m < N_PARA; ++m)
                if (!(r_out[b][m] == RDATA_EXP[b][m])) {
                    std::printf("  FAIL  T20: RD%d data mismatch m=%d\n", b, m);
                    t20_data_ok = false;
                    ++g_fail;
                } else {
                    ++g_pass;
                }
        CHECK(t20_data_ok, "T20c all 5 concurrent reads returned correct pre-written data");

        // Verify the 4 concurrently-written datasets, using each write buffer's
        // natural read-side counterpart (RD0<->WR0, RD1<->WR1, RD3<->WR2/WAGU_D,
        // RD4<->WR3/WAGU_E).
        bool t20_w_data_ok = true;
        {
            const int      chk_rbus[4] = {0, DUT::RD1_BASE, DUT::RD3_BASE, DUT::RD4_BASE};
            const uint64_t chk_base[4] = {T20_BASE_W0, T20_BASE_W1, T20_BASE_W2, T20_BASE_W3};
            data_t        *chk_exp[4]  = {t20_w0, t20_w1, t20_w2, t20_w3};
            for (int b = 0; b < 4; ++b) {
                uint64_t chk_addrs[N_PARA];
                make_addrs(chk_addrs, chk_base[b]);
                data_t chk_out[N_PARA];
                if (!do_read_group(chk_rbus[b], chk_addrs, chk_out)) {
                    std::printf("  FAIL  T20: write-verify read %d timed out\n", b);
                    t20_w_data_ok = false;
                    ++g_fail;
                    continue;
                }
                for (int m = 0; m < N_PARA; ++m)
                    if (!(chk_out[m] == chk_exp[b][m])) {
                        std::printf("  FAIL  T20: write-verify %d mismatch m=%d got=0x%08x "
                                    "want=0x%08x addr=0x%llx\n",
                                    b, m, (uint32_t)chk_out[m].range(31, 0).to_uint(),
                                    (uint32_t)chk_exp[b][m].range(31, 0).to_uint(),
                                    (unsigned long long)chk_addrs[m]);
                        t20_w_data_ok = false;
                        ++g_fail;
                    } else {
                        ++g_pass;
                    }
            }
        }
        CHECK(t20_w_data_ok, "T20d all 4 concurrently-written datasets correct");

        // ── T21 ──────────────────────────────────────────────────────────
        std::puts("\n=== T21: Intra-window bank conflict — crossbar serializes, no data loss ===");
        do_reset();
        // With the default mapping (R=4,C=4,L=8,mode=0), bank_id is a function
        // of address bits [8:4] only; row_id = addr>>9. Adding exactly 0x200
        // (row+1) keeps bits[8:4] identical, so addr A and B below map to the
        // SAME bank but different rows — a genuine intra-window conflict the
        // crossbar must serialize. C and D use distinct bank_id values as
        // conflict-free fillers in the same group.
        static constexpr uint64_t T21_ADDR_A = kBase;
        static constexpr uint64_t T21_ADDR_B = kBase + 0x200; // same bank as A, row+1
        static constexpr uint64_t T21_ADDR_C = kBase + 0x10;
        static constexpr uint64_t T21_ADDR_D = kBase + 0x20;
        const uint64_t t21_addrs[N_PARA]     = {T21_ADDR_A, T21_ADDR_B, T21_ADDR_C, T21_ADDR_D};
        data_t         t21_data[N_PARA];
        for (int m = 0; m < N_PARA; ++m)
            t21_data[m] = make_row<data_t>(0x21000000U + static_cast<uint32_t>(m));

        int  t21_gnt[N_PARA] = {};
        int  t21_rv = 0, t21_grp = 0;
        auto drive_t21 = [&](int g) {
            for (int m = 0; m < N_PARA; ++m) {
                const bool is_real = (g == 0);
                wport[WBUS + m].req.write(true);
                wport[WBUS + m].addr.write(is_real ? t21_addrs[m] : 0);
                wport[WBUS + m].we.write(true);
                wport[WBUS + m].be.write(FULL_BE);
                wport[WBUS + m].wdata.write(is_real ? t21_data[m] : data_t(0));
            }
        };
        drive_t21(0);
        for (int iter = 0; iter < FRAME_SIZE * kTimeout && t21_rv < FRAME_SIZE; ++iter) {
            tick(clk);
            bool ag = true;
            for (int m = 0; m < N_PARA; ++m) {
                if (t21_gnt[m] <= t21_grp) {
                    if (wport[WBUS + m].gnt.read()) {
                        ++t21_gnt[m];
                        idle_wport(WBUS + m);
                    } else
                        ag = false;
                }
            }
            if (ag && t21_grp < FRAME_GRPS - 1) {
                ++t21_grp;
                drive_t21(t21_grp);
            }
            for (int m = 0; m < N_PARA; ++m)
                if (wport[WBUS + m].rvalid.read())
                    ++t21_rv;
        }
        for (int m = 0; m < N_PARA; ++m)
            idle_wport(WBUS + m);
        CHECK(t21_rv == FRAME_SIZE, "T21a write completes despite intra-window bank conflict");

        // Write acks are POSTED (see buffer.hpp's write branch): the last
        // ack means the window's TDM burst is in flight, not that every
        // bank committed — and this window's conflicting pair serializes
        // at its bank for a few extra cycles behind the acks. Let the
        // flush drain before reading back (production flows separate write
        // and read phases by fences thousands of cycles wide; a
        // back-to-back read of a just-acked conflicted address is exactly
        // the ordering posted acks give up).
        for (int i = 0; i < 8; ++i)
            tick(clk);

        data_t t21_out[N_PARA];
        CHECK(do_read_group(RBUS, t21_addrs, t21_out), "T21b read-back completed");
        bool t21_ok = true;
        for (int m = 0; m < N_PARA; ++m) {
            if (!(t21_out[m] == t21_data[m])) {
                std::printf("  FAIL  T21: mismatch m=%d\n", m);
                t21_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        if (t21_ok)
            std::puts("  PASS  T21c all 4 addresses (including the conflicting pair) read back "
                      "correctly");

        // ── T22 ──────────────────────────────────────────────────────────
        std::puts("\n=== T22: Reset recovery — mid-fill interruption returns to a clean state ===");
        do_reset();
        // Start a write frame but interrupt it partway through fill with an
        // async reset; the buffer must recover to a fully clean state and
        // accept a completely fresh, independent write/read afterward.
        data_t t22_abandoned[FRAME_SIZE];
        fill_frame(t22_abandoned, make_row<data_t>(0xBAD00BADU));
        int  t22_grp   = 0;
        auto drive_t22 = [&](int g) {
            for (int m = 0; m < N_PARA; ++m) {
                wport[WBUS + m].req.write(true);
                wport[WBUS + m].addr.write(kBase +
                                           static_cast<uint64_t>(g * N_PARA + m) * ADDR_STEP);
                wport[WBUS + m].we.write(true);
                wport[WBUS + m].be.write(FULL_BE);
                wport[WBUS + m].wdata.write(t22_abandoned[g * N_PARA + m]);
            }
        };
        drive_t22(0);
        for (int iter = 0; iter < 3; ++iter) {
            tick(clk);
            for (int m = 0; m < N_PARA; ++m)
                if (wport[WBUS + m].gnt.read())
                    idle_wport(WBUS + m);
            if (t22_grp < 2) {
                ++t22_grp;
                drive_t22(t22_grp);
            }
        }
        // Interrupt mid-fill (only some groups accepted so far) with a full reset.
        do_reset();

        data_t t22_fresh[FRAME_SIZE];
        seq_frame(t22_fresh, 0x22000001U);
        CHECK(do_write_frame(WBUS, kBase, t22_fresh),
              "T22a fresh write completes after mid-fill reset");
        uint64_t t22_addrs[N_PARA];
        make_addrs(t22_addrs, kBase);
        data_t t22_out[N_PARA];
        CHECK(do_read_group(RBUS, t22_addrs, t22_out), "T22b fresh read-back completes");
        bool t22_ok = true;
        for (int m = 0; m < N_PARA; ++m) {
            if (!(t22_out[m] == t22_fresh[m])) {
                std::printf("  FAIL  T22: mismatch m=%d\n", m);
                t22_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        if (t22_ok)
            std::puts("  PASS  T22c fresh data correct (no leftover state from abandoned fill)");

        // ── T23 ──────────────────────────────────────────────────────────
        std::puts("\n=== T23: 32-way same-bank conflict — shadow flush serializes 1-per-cycle ===");
        do_reset();
        // All 32 addresses share the same TDM-mapped bank (same trick as
        // T21: with the default mapping, bank_id depends only on address
        // bits[8:4]; adding exactly 0x200 changes only the row, not the
        // bank). This is the TDM-side analog of the crossbar same-bank
        // conflict test (tb_top_crossbar_conflict.cpp): the snapshot's shadow
        // burst presents all
        // 32 cells to the crossbar simultaneously, but since they all target
        // ONE physical bank, only one gets granted per cycle.
        static constexpr uint64_t T23_BASE =
            kBase + 25 * static_cast<uint64_t>(FRAME_SIZE) * ADDR_STEP;
        data_t t23_data[FRAME_SIZE];
        for (int i = 0; i < FRAME_SIZE; ++i)
            t23_data[i] = make_row<data_t>(0x23000000U + static_cast<uint32_t>(i));

        int t23_cycle = 0;
        CHECK(do_write_frame(WBUS, T23_BASE, t23_data, FULL_BE, 0x200, &t23_cycle),
              "T23a all 32 same-bank writes eventually complete");

        // Write acks are POSTED (see buffer.hpp's write branch), so the
        // conflicted window's own acks stream at full rate — its 32-way
        // serialization instead BACK-PRESSURES whatever writes next: the
        // following window fills, then its snapshot must wait for every
        // conflicted shadow to drain through the one shared bank. Time a
        // non-conflicting write issued right behind the conflicted one to
        // observe exactly that relocated cost.
        data_t t23_follow_data[FRAME_SIZE];
        seq_frame(t23_follow_data, 0x23BA5E00U);
        // T23_BASE spans [kBase+25*FRAME_SIZE*ADDR_STEP, +31*0x200] because of
        // its wide 0x200 stride (32 entries × 0x200 = 0x4000 total) — far
        // wider than a normal ADDR_STEP-spaced frame. Use a base well clear
        // of that whole span to avoid an address collision.
        static constexpr uint64_t T23_FOLLOW_BASE =
            kBase + 300 * static_cast<uint64_t>(FRAME_SIZE) * ADDR_STEP;
        int follow_cycle = 0;
        CHECK(do_write_frame(WBUS, T23_FOLLOW_BASE, t23_follow_data, FULL_BE, ADDR_STEP,
                             &follow_cycle),
              "T23b non-conflicting write issued behind the conflict completes");

        char t23_lbl[192];
        std::snprintf(t23_lbl, sizeof(t23_lbl),
                      "T23c conflict cost lands as back-pressure: the conflicted window acks "
                      "fast (%d cycles, posted) while the FOLLOWING write absorbs the "
                      "serialization (%d cycles)",
                      t23_cycle, follow_cycle);
        CHECK(follow_cycle > t23_cycle + FRAME_SIZE / 2, t23_lbl);

        uint64_t t23_addrs[FRAME_SIZE];
        for (int i = 0; i < FRAME_SIZE; ++i)
            t23_addrs[i] = T23_BASE + static_cast<uint64_t>(i) * 0x200;

        // Data correctness: all 32 addresses read back correctly despite the
        // shadow-flush serialization (distinct rows, so no aliasing).
        bool t23_data_ok = true;
        for (int i = 0; i < FRAME_SIZE; i += N_PARA) {
            uint64_t a4[N_PARA] = {t23_addrs[i], t23_addrs[i + 1], t23_addrs[i + 2],
                                   t23_addrs[i + 3]};
            data_t   o4[N_PARA];
            if (!do_read_group(RBUS, a4, o4)) {
                t23_data_ok = false;
                std::printf("  FAIL  T23: read group at idx=%d timed out\n", i);
                continue;
            }
            for (int m = 0; m < N_PARA; ++m)
                if (!(o4[m] == t23_data[i + m])) {
                    t23_data_ok = false;
                    std::printf("  FAIL  T23: mismatch idx=%d addr=0x%llx got=0x%08x want=0x%08x\n",
                                i + m, (unsigned long long)t23_addrs[i + m],
                                (uint32_t)o4[m].range(31, 0).to_uint(),
                                (uint32_t)t23_data[i + m].range(31, 0).to_uint());
                }
        }
        CHECK(t23_data_ok, "T23d all 32 same-bank addresses read back correctly");

        // ─────────────────────────────────────────────────────────────────
        sc_stop();
    }
};

int sc_main(int, char **) {
    tb t{"tb"};
    sc_start();
    return report_and_exit();
}
