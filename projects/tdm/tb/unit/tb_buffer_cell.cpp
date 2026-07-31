// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Unit tests for buffer_cell<BYTES_PER_ROW>.
//
// Build and run:
//   make -C projects/tdm/tb/unit
//
// Tests:
//   T01      Reset: all outputs low after reset
//   T02      IDLE: m_bus.req=0 before address is presented
//   T03      Address latch: en_i pulse → REQUESTING (req+addr+be asserted)
//   T04      REQUESTING: req stable across multiple cycles while gnt=0
//   T05      REQUESTING: addr not re-latched if en_i pulses again
//   T06      TDM gnt → GRANTED: m_bus.req deasserts; addr/be zeroed
//   T07      TDM rvalid → VALID: data latched, valid_o=1
//   T08      gnt+rvalid same cycle: impl-defined tolerance (TDM R-5 violation)
//   T09      VALID + all_valid_i=0: AGU outputs silent
//   T10      VALID + all_valid_i=1: p_bus.gnt+p_bus.rvalid+p_bus.rdata asserted
//            (p_bus.rvalid fires same cycle as p_bus.gnt; buffer registers it → R-5 met at system
//            boundary)
//   T11      p_bus.gnt pre-asserted before p_bus.req (OBI R-3.2.1)
//   T12      p_bus.req=1 alone has no effect on p_bus.gnt
//   T13      VALID → INVALID when all_valid_i=1 (next cycle); invalid_o=1
//   T14      INVALID: all AGU outputs 0
//   T15      all_valid_i (even with en_i) while a fetch is already pending
//            does not interrupt the in-flight fetch — a fetch only ever
//            starts once !pending (see buffer_cell.hpp's own header comment)
//   T16      all_valid_i alone (no en_i): drains this cell (valid_o clears)
//            but starts no new fetch — en_i must be present too
//   T17      all_valid_i + en_i together start the next fetch immediately,
//            the same cycle this cell's own group drains
//   T18      A fetch started in the background (T17) commits only once
//            all_valid_i is asserted again (this cell's group is drained a
//            second time) — the response isn't lost while it waits
//   T19      Once fetch_addr_i in place, this cell behaves exactly like a
//            fresh boot-path fetch (grant → rvalid → commit)
//   T-OBI-A  Premature TDM gnt (not mid-fetch) is ignored
//   T-OBI-B  TDM rvalid before grant is ignored (OBI R-5 enforcement by cell)
//   T20      m_bus.we always 0; m_bus.be=0 outside address phase
//   T21      Full round-trip: boot fetch → drain + second fetch start
//            (same cycle, all_valid_i+en_i) → second fetch commits once
//            drained again, with valid_o held throughout the gap
// -----------------------------------------------------------------------------

#include "buffer_cell.hpp"
#include "obi_data.hpp"
#include "unit_test_common.hpp"
#include <cstdio>
#include <cstdlib>
#include <systemc.h>

// Use a compact 4-byte data word so values are easy to reason about.
static constexpr int kBytes = 4;
using data_t                = obi_data<kBytes>;
using DUT                   = buffer_cell<kBytes>;

// ---------------------------------------------------------------------------
// Testbench module
// ---------------------------------------------------------------------------
SC_MODULE(tb) {
    sc_clock        clk{"clk", 10, SC_NS};
    sc_signal<bool> rst_n{"rst_n"};

    // DUT inputs
    sc_signal<uint64_t> addr_i{"addr_i"};
    sc_signal<bool>     en_i{"en_i"};
    sc_signal<bool>     all_valid_i{"all_valid_i"};
    sc_signal<bool> commit_ok_i{"commit_ok_i"}; // held 0: the echo has its own tests via tb_buffer
    sc_signal<bool> reset_window_i{"reset_window_i"};

    // DUT outputs
    sc_signal<bool> valid_o{"valid_o"};
    sc_signal<bool> invalid_o{"invalid_o"};

    // Port- and TDM-side OBI as wire bundles
    obi_signal_bundle<data_t> p_bus;
    obi_signal_bundle<data_t> m_bus;

    DUT *dut;

    SC_HAS_PROCESS(tb);

    tb(sc_module_name nm) : sc_module(nm) {
        dut = new DUT("dut");
        dut->clk_i(clk);
        dut->rst_ni(rst_n);
        dut->addr_i(addr_i);
        dut->en_i(en_i);
        bind_obi(dut->p, p_bus);
        bind_obi(dut->m, m_bus);
        dut->all_valid_i(all_valid_i);
        dut->commit_ok_i(commit_ok_i);
        dut->reset_window_i(reset_window_i);
        dut->valid_o(valid_o);
        dut->invalid_o(invalid_o);

        SC_THREAD(run);
    }

    ~tb() {
        delete dut;
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    // Assert reset and hold for n clocks, then release and settle.
    void do_reset(int cycles = 2) {
        rst_n.write(false);
        en_i.write(false);
        m_bus.gnt.write(false);
        m_bus.rvalid.write(false);
        m_bus.rdata.write(data_t{0});
        p_bus.req.write(false);
        all_valid_i.write(false);
        reset_window_i.write(false);
        addr_i.write(0);
        p_bus.addr.write(0);
        for (int i = 0; i < cycles; ++i)
            wait(clk.posedge_event());
        rst_n.write(true);
        tick(clk);
    }

    // Advance one clock and let all delta cycles from the posedge settle.
    // wait(1, SC_NS) (< half the 10 ns period) ensures every sc_signal
    // write in seq_proc has propagated through comb_proc to its outputs.

    // Drive en_i for one clock to latch addr, then deassert.
    void latch_addr(uint64_t addr) {
        addr_i.write(addr);
        p_bus.addr.write(addr); // keep cross-check signal in sync with what actually gets latched
        en_i.write(true);
        tick(clk);
        en_i.write(false);
    }

    // Drive TDM grant for one cycle.
    void grant() {
        m_bus.gnt.write(true);
        tick(clk);
        m_bus.gnt.write(false);
    }

    // Drive TDM rvalid+data for one cycle.
    void rvalid(data_t d) {
        m_bus.rvalid.write(true);
        m_bus.rdata.write(d);
        tick(clk);
        m_bus.rvalid.write(false);
        m_bus.rdata.write(data_t{0});
    }

    // Bring the cell from reset all the way to VALID with given addr and data.
    void bring_to_valid(uint64_t addr, data_t data) {
        latch_addr(addr);
        grant();
        rvalid(data);
    }

    // -----------------------------------------------------------------------
    // Test cases
    // -----------------------------------------------------------------------
    void run() {
        // -------------------------------------------------------------------
        std::puts("\n=== T01: Reset — all outputs low ===");
        // -------------------------------------------------------------------
        do_reset();
        CHECK(!m_bus.req.read(), "T01a m_bus.req=0");
        CHECK(!p_bus.gnt.read(), "T01b p_bus.gnt=0");
        CHECK(!p_bus.rvalid.read(), "T01c p_bus.rvalid=0");
        CHECK(!valid_o.read(), "T01d valid_o=0");
        CHECK(invalid_o.read(), "T01e invalid_o=1 — read mode's idle flag: holding nothing, "
                                "fetching nothing (see buffer_cell.hpp's comb_proc_read)");

        // -------------------------------------------------------------------
        std::puts("\n=== T02: IDLE — no req before address ===");
        // -------------------------------------------------------------------
        do_reset();
        // Hold idle for several cycles
        for (int i = 0; i < 4; ++i)
            tick(clk);
        CHECK(!m_bus.req.read(), "T02 m_bus.req=0 while IDLE (no addr)");

        // -------------------------------------------------------------------
        std::puts("\n=== T03: Address latch — IDLE→REQUESTING ===");
        // -------------------------------------------------------------------
        do_reset();
        latch_addr(0xDEAD'BEEF'0000'0100ULL);
        // After tick: seq_proc latched addr, comb_proc drives req
        CHECK(m_bus.req.read(), "T03a m_bus.req=1");
        CHECK(m_bus.addr.read() == 0xDEAD'BEEF'0000'0100ULL, "T03b m_bus.addr=latched addr");
        CHECK(!m_bus.we.read(), "T03c m_bus.we=0 (read)");
        static constexpr uint32_t kBeFull = static_cast<uint32_t>((uint64_t{1} << kBytes) - 1u);
        CHECK(m_bus.be.read() == kBeFull, "T03d m_bus.be=all-ones");

        // -------------------------------------------------------------------
        std::puts("\n=== T04: REQUESTING — req stable, no gnt ===");
        // -------------------------------------------------------------------
        for (int i = 0; i < 4; ++i) {
            tick(clk);
            CHECK(m_bus.req.read(), "T04 m_bus.req stable while waiting for gnt");
        }

        // -------------------------------------------------------------------
        std::puts("\n=== T05: REQUESTING — addr not re-latched on second en_i ===");
        // -------------------------------------------------------------------
        addr_i.write(0xFFFF'FFFF'FFFF'FFFFULL);
        en_i.write(true);
        tick(clk);
        en_i.write(false);
        CHECK(m_bus.addr.read() == 0xDEAD'BEEF'0000'0100ULL,
              "T05 addr_q unchanged on second en_i (REQUESTING, not IDLE)");

        // -------------------------------------------------------------------
        std::puts("\n=== T06: gnt → GRANTED: req deasserts ===");
        // -------------------------------------------------------------------
        grant();
        CHECK(!m_bus.req.read(), "T06a m_bus.req=0 after gnt");
        CHECK(m_bus.addr.read() == 0, "T06b m_bus.addr=0 outside address phase");
        CHECK(m_bus.be.read() == 0, "T06c m_bus.be=0 outside address phase");

        // -------------------------------------------------------------------
        std::puts("\n=== T07: rvalid → VALID: data latched ===");
        // -------------------------------------------------------------------
        data_t d1{"0xCAFEBABE"};
        rvalid(d1);
        CHECK(valid_o.read(), "T07a valid_o=1");
        CHECK(!invalid_o.read(), "T07b invalid_o=0");
        CHECK(!m_bus.req.read(), "T07c m_bus.req=0 in VALID");

        // -------------------------------------------------------------------
        // T08: gnt + rvalid same cycle.
        //
        // OBI R-5 states the earliest rvalid can fire is the cycle AFTER
        // both req and gnt have been sampled high — so a TDM subordinate
        // asserting gnt and rvalid simultaneously is technically an R-5
        // violation.  The cell nonetheless handles it gracefully: seq_proc
        // evaluates REQUESTING→GRANTED then GRANTED→VALID in the same pass.
        // The test documents this implementation-defined tolerance, not a
        // required protocol behaviour.
        // -------------------------------------------------------------------
        std::puts("\n=== T08: gnt+rvalid coincide (impl-defined; TDM R-5 violation) ===");
        do_reset();
        latch_addr(0x1000);
        m_bus.gnt.write(true);
        m_bus.rvalid.write(true);
        m_bus.rdata.write(data_t{"0xABCD1234"});
        tick(clk);
        m_bus.gnt.write(false);
        m_bus.rvalid.write(false);
        m_bus.rdata.write(data_t{0});
        CHECK(valid_o.read(), "T08 cell reaches VALID despite illegal same-cycle gnt+rvalid");

        // -------------------------------------------------------------------
        std::puts("\n=== T09: VALID + all_valid_i=0 — AGU outputs silent ===");
        // -------------------------------------------------------------------
        do_reset();
        bring_to_valid(0x2000, data_t{"0x12345678"});
        // all_valid_i is 0 by default
        CHECK(!p_bus.gnt.read(), "T09a p_bus.gnt=0 when all_valid_i=0");
        CHECK(!p_bus.rvalid.read(), "T09b p_bus.rvalid=0 when all_valid_i=0");
        CHECK(p_bus.rdata.read() == data_t{0}, "T09c p_bus.rdata=0 when all_valid_i=0");

        // -------------------------------------------------------------------
        // T10: p_bus.gnt and p_bus.rvalid both fire when window_ready.
        //
        // For the port-facing interface (cell is subordinate) OBI R-5 also
        // applies: rvalid should only appear in the cycle after req+gnt are
        // sampled.  The cell asserts p_bus.rvalid combinatorially in the same
        // cycle as p_bus.gnt, which would violate R-5 in isolation.  This is
        // intentional: the parent buffer seq_proc re-registers p_bus.rvalid /
        // p_bus.rdata before forwarding them to the AGU, satisfying R-5 at
        // the system boundary.  Here we verify the cell's internal behaviour.
        // -------------------------------------------------------------------
        std::puts("\n=== T10: VALID + all_valid_i=1 — AGU R-channel asserted ===");
        data_t d2{"0x12345678"};
        all_valid_i.write(true);
        wait(1, SC_NS);
        CHECK(p_bus.gnt.read(), "T10a p_bus.gnt=1");
        CHECK(p_bus.rvalid.read(), "T10b p_bus.rvalid=1 (same cycle as gnt; buffer will register)");
        CHECK(p_bus.rdata.read() == d2, "T10c p_bus.rdata=expected data");
        all_valid_i.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T11: p_bus.gnt pre-asserted before p_bus.req (OBI R-3.2.1) ===");
        // -------------------------------------------------------------------
        all_valid_i.write(true);
        p_bus.req.write(false);
        wait(1, SC_NS);
        CHECK(p_bus.gnt.read(), "T11 p_bus.gnt=1 even when p_bus.req=0");
        all_valid_i.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T12: p_bus.req alone has no effect on p_bus.gnt ===");
        // -------------------------------------------------------------------
        p_bus.req.write(true);
        all_valid_i.write(false);
        wait(1, SC_NS);
        CHECK(!p_bus.gnt.read(), "T12 p_bus.gnt=0 when all_valid_i=0 regardless of p_bus.req");
        p_bus.req.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T13: VALID → INVALID when all_valid_i=1 ===");
        // -------------------------------------------------------------------
        all_valid_i.write(true);
        tick(clk); // posedge: seq_proc sees all_valid_i=1 while valid_q → drains
        all_valid_i.write(false);
        wait(1, SC_NS); // let comb_proc_read settle after the all_valid_i write above
        CHECK(invalid_o.read(), "T13a invalid_o=1 — drained with en_i low: idle/parked "
                                "(read mode's idle flag, not a write-mode INVALID state)");
        CHECK(!valid_o.read(), "T13b valid_o=0");

        // -------------------------------------------------------------------
        std::puts("\n=== T14: drained — AGU outputs 0 ===");
        // -------------------------------------------------------------------
        CHECK(!p_bus.gnt.read(), "T14a p_bus.gnt=0 once drained");
        CHECK(!p_bus.rvalid.read(), "T14b p_bus.rvalid=0 once drained");
        CHECK(p_bus.rdata.read() == data_t{0}, "T14c p_bus.rdata=0 once drained");

        // -------------------------------------------------------------------
        std::puts("\n=== T15: all_valid_i while pending — does not interrupt the fetch ===");
        // -------------------------------------------------------------------
        do_reset();
        latch_addr(0x4000); // boot-path fetch: now pending, req asserted for 0x4000
        CHECK(m_bus.req.read(), "T15 pre: req active while fetch pending");
        // Even asserting en_i+all_valid_i together while a fetch is already
        // pending must not restart it — a cell's own group can never
        // legitimately drain (all_valid_i) while still pending in the first
        // place (see buffer.hpp's eval_group/can_drain), but the guard
        // (!pending in "start") holds regardless of how this cell gets
        // driven standalone here.
        addr_i.write(0x5000);
        en_i.write(true);
        all_valid_i.write(true);
        tick(clk);
        en_i.write(false);
        all_valid_i.write(false);
        CHECK(m_bus.req.read(), "T15a m_bus.req still 1 — in-flight fetch uninterrupted");
        CHECK(m_bus.addr.read() == 0x4000, "T15b m_bus.addr unchanged — still targeting 0x4000");
        CHECK(!valid_o.read(), "T15c valid_o=0 — nothing committed yet");
        // The original (uninterrupted) fetch completes normally.
        grant();
        rvalid(data_t{"0x00004000"});
        CHECK(valid_o.read(), "T15d valid_o=1 once the original fetch completes");

        // -------------------------------------------------------------------
        std::puts("\n=== T16: all_valid_i alone (no en_i) drains but starts no new fetch ===");
        // -------------------------------------------------------------------
        do_reset();
        bring_to_valid(0x8000, data_t{"0x0000DEAD"});
        all_valid_i.write(true);
        tick(clk);
        all_valid_i.write(false);
        CHECK(!valid_o.read(), "T16a valid_o=0 — this cell's own group drained");
        CHECK(!m_bus.req.read(), "T16b m_bus.req=0 — no fetch started (en_i wasn't asserted)");
        CHECK(p_bus.rdata.read() == data_t{0},
              "T16c p_bus.rdata=0 (all_valid_i deasserted, port silent)");

        // -------------------------------------------------------------------
        std::puts("\n=== T17: all_valid_i + en_i together start the next fetch immediately ===");
        // -------------------------------------------------------------------
        // Continuing from T16: this cell just drained and is sitting !valid.
        // Its own group draining again (all_valid_i) with a fresh address on
        // the bus (en_i) starts the next fetch the same cycle — no separate
        // trigger to wait on (see buffer_cell.hpp's own header comment).
        addr_i.write(0x9000);
        en_i.write(true);
        all_valid_i.write(true);
        tick(clk);
        en_i.write(false);
        CHECK(m_bus.req.read(),
              "T17a m_bus.req=1 — new fetch for 0x9000 started same cycle as drain");
        CHECK(m_bus.addr.read() == 0x9000, "T17b m_bus.addr=0x9000");
        CHECK(!valid_o.read(), "T17c valid_o=0 — new fetch not yet complete");
        all_valid_i.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T18: background fetch commits only once drained again (all_valid_i) ===");
        // -------------------------------------------------------------------
        // Continuing T17's in-flight fetch for 0x9000: grant + rvalid arrive
        // while all_valid_i is deasserted (this cell isn't being drained
        // right now) — the response must wait, not get lost. valid_o
        // (is_fwd) can still preview it combinationally once !valid alone
        // (comb_proc_read's own, deliberately laxer safe — see that
        // function's own comment on why it can't match seq_proc's stricter
        // one without reintroducing a real deadlock); what must NOT happen
        // early is the durable, register-gated commit — checked here via
        // p_bus.rvalid, which is gated on the registered valid_q && all_valid_i
        // (still false), not on the is_fwd preview.
        grant();
        m_bus.rvalid.write(true);
        m_bus.rdata.write(data_t{"0x00009000"});
        tick(clk);
        m_bus.rvalid.write(false);
        m_bus.rdata.write(data_t{0});
        CHECK(!p_bus.rvalid.read(),
              "T18a p_bus.rvalid=0 (fetch complete but not yet safe to durably commit)");
        // Now this cell's group is drained again — the new data commits
        // immediately (is_fwd) instead of waiting an extra edge.
        all_valid_i.write(true);
        wait(1, SC_NS);
        CHECK(valid_o.read(), "T18b valid_o=1 combinatorially (is_fwd: new data ready + now safe)");
        CHECK(p_bus.rdata.read() == data_t{"0x00009000"},
              "T18c p_bus.rdata=new (0x9000's) data via is_fwd");
        tick(clk);
        all_valid_i.write(false);
        wait(1, SC_NS); // let comb_proc_read settle after the all_valid_i write above
        CHECK(valid_o.read(), "T18d valid_o=1 (now durably registered from the commit)");
        CHECK(p_bus.rdata.read() == data_t{0}, "T18e p_bus.rdata=0 once all_valid_i deasserts");

        // -------------------------------------------------------------------
        std::puts("\n=== T19: fresh boot-style fetch still works after all this ===");
        // -------------------------------------------------------------------
        do_reset();
        latch_addr(0xB000);
        CHECK(m_bus.req.read(), "T19a req asserted for a fresh boot-path fetch");
        CHECK(m_bus.addr.read() == 0xB000, "T19b new addr latched");

        // -------------------------------------------------------------------
        // T-OBI-A: premature TDM gnt (arrives while cell is not REQUESTING).
        //
        // OBI R-3.2.1 allows a subordinate to assert gnt before req, but
        // the cell must not change state on early gnt: only a gnt received
        // while in REQUESTING phase is meaningful.  Test: gnt in IDLE and
        // in GRANTED states → cell stays put, req not driven spuriously.
        // -------------------------------------------------------------------
        std::puts("\n=== T-OBI-A: premature TDM gnt ignored (not in REQUESTING) ===");
        do_reset();
        // IDLE: gnt arrives before address is latched
        m_bus.gnt.write(true);
        tick(clk);
        m_bus.gnt.write(false);
        CHECK(!m_bus.req.read(), "T-OBI-Aa gnt in IDLE: req still 0 (no addr)");
        CHECK(!valid_o.read(), "T-OBI-Ab gnt in IDLE: state still MISSING");
        // Latch addr → REQUESTING; grant correctly; land in GRANTED
        latch_addr(0xD000);
        grant();
        CHECK(!m_bus.req.read(), "T-OBI-Ac in GRANTED after correct grant");
        // Second gnt while already GRANTED (req is deasserted)
        m_bus.gnt.write(true);
        tick(clk);
        m_bus.gnt.write(false);
        CHECK(!valid_o.read(), "T-OBI-Ad second gnt in GRANTED: still waiting for rvalid");
        // Complete legitimately
        rvalid(data_t{"0x0000EEEE"});
        CHECK(valid_o.read(), "T-OBI-Ae VALID after legitimate rvalid");

        // -------------------------------------------------------------------
        // T-OBI-B: TDM rvalid before grant violates OBI R-5.
        //
        // "rvalid shall not start before req and gnt have been sampled high."
        // The cell enforces R-5 by only transitioning MISSING→VALID in the
        // GRANTED fetch phase.  A rvalid arriving while still REQUESTING
        // must be ignored — the cell stays in REQUESTING and keeps req high.
        // -------------------------------------------------------------------
        std::puts("\n=== T-OBI-B: TDM rvalid before grant ignored (OBI R-5 enforcement) ===");
        do_reset();
        latch_addr(0xE000);
        // Premature rvalid while still in REQUESTING (no grant yet)
        m_bus.rvalid.write(true);
        m_bus.rdata.write(data_t{"0x0000BAD0"});
        tick(clk);
        m_bus.rvalid.write(false);
        m_bus.rdata.write(data_t{0});
        CHECK(m_bus.req.read(), "T-OBI-Ba req still asserted (not granted yet)");
        CHECK(!valid_o.read(), "T-OBI-Bb still not VALID (premature rvalid ignored)");
        // Now grant legitimately and complete
        grant();
        rvalid(data_t{"0x0000600D"});
        CHECK(valid_o.read(), "T-OBI-Bc VALID after correct grant+rvalid sequence");

        // -------------------------------------------------------------------
        std::puts("\n=== T20: m_bus.we always 0; m_bus.be=0 outside address phase ===");
        // -------------------------------------------------------------------
        do_reset();
        // In IDLE: req=0 → be=0, we=0
        CHECK(!m_bus.we.read(), "T20a m_bus.we=0 in IDLE");
        CHECK(m_bus.be.read() == 0, "T20b m_bus.be=0 in IDLE");
        latch_addr(0xC000);
        CHECK(!m_bus.we.read(), "T20c m_bus.we=0 in REQUESTING");
        CHECK(m_bus.be.read() == kBeFull, "T20d m_bus.be=all-ones in REQUESTING");
        grant();
        CHECK(!m_bus.we.read(), "T20e m_bus.we=0 in GRANTED");
        CHECK(m_bus.be.read() == 0, "T20f m_bus.be=0 in GRANTED");

        // -------------------------------------------------------------------
        std::puts("\n=== T21: Full round-trip (two consecutive fetches) ===");
        // -------------------------------------------------------------------
        do_reset();
        // First fetch
        data_t rd1{"0xAAAAAAAA"};
        bring_to_valid(0x1000, rd1);
        CHECK(valid_o.read(), "T21a first fetch VALID");
        // Drain, and start the second fetch in the very same cycle
        // (all_valid_i + en_i together) — this cell's own group draining is
        // what starts its next fetch now, not a separate reset_window_i
        // trigger (see buffer_cell.hpp's own header comment).
        data_t rd2{"0xBBBBBBBB"};
        addr_i.write(0x2000);
        en_i.write(true);
        all_valid_i.write(true);
        tick(clk);
        en_i.write(false);
        CHECK(!valid_o.read(), "T21b valid_o=0 after drain (no INVALID state in read mode)");
        CHECK(m_bus.req.read(),
              "T21c m_bus.req=1 — second fetch for 0x2000 started same cycle as drain");
        all_valid_i.write(false);
        grant();
        rvalid(rd2);
        // Fetch complete, but all_valid_i has been deasserted since — the
        // durable, register-gated commit waits for this cell's group to be
        // drained again (mirrors the real buffer's own wraparound — see
        // buffer.hpp). Checked via p_bus.rvalid, not valid_o — see T18's own
        // comment on why valid_o (is_fwd) can preview this combinationally
        // even before then, harmlessly.
        CHECK(!p_bus.rvalid.read(), "T21c2 fetch complete but not yet safe to durably commit");
        all_valid_i.write(true);
        wait(1, SC_NS);
        CHECK(valid_o.read(), "T21d second fetch: valid_o=1 same cycle (is_fwd forwarding)");
        // p_bus.rvalid (window_ready = valid_q && all_valid_i) reflects the
        // REGISTERED valid_q, not valid_o's own is_fwd combinational cover —
        // buffer.hpp never actually reads this cell's own p_bus.rvalid/p_bus.gnt
        // (it sinks them and drives the port directly from its own group
        // logic — see that file's own comment), so it needs one more edge
        // for the delayed register commit here (see safe's own comment: a
        // primed cell's commit now always waits for all_valid_i, even if
        // this cell itself is already !valid, so a structurally-NOP sibling
        // lane can't race ahead of a slower real lane in the same group).
        tick(clk);
        CHECK(p_bus.rvalid.read(), "T21e second fetch: p_bus.rvalid=1 one edge later");
        CHECK(p_bus.rdata.read() == rd2, "T21f second fetch: correct data");
        all_valid_i.write(false);

        // -------------------------------------------------------------------
        // -------------------------------------------------------------------
        std::puts("\n=== T22: NOP fetch (addr=0) never touches the bus ===");
        // -------------------------------------------------------------------
        do_reset();
        latch_addr(0); // NOP sentinel
        wait(1, SC_NS);
        CHECK(!m_bus.req.read(), "T22a a NOP fetch issues no TDM request");
        tick(clk);
        wait(1, SC_NS);
        CHECK(valid_o.read(), "T22b NOP completes to VALID without any bus activity");
        CHECK(!m_bus.req.read(), "T22c still quiet after completing");

        // -------------------------------------------------------------------
        std::puts("\n=== T23: idle report (invalid_o) across a fetch lifecycle ===");
        // -------------------------------------------------------------------
        do_reset();
        wait(1, SC_NS);
        CHECK(invalid_o.read(), "T23a idle after reset (!valid && !pending)");
        latch_addr(0x3300);
        wait(1, SC_NS);
        CHECK(!invalid_o.read(), "T23b not idle while the fetch is pending");
        grant();
        m_bus.rvalid.write(true);
        m_bus.rdata.write(data_t(0x33ull));
        tick(clk);
        m_bus.rvalid.write(false);
        wait(1, SC_NS);
        CHECK(!invalid_o.read() && valid_o.read(), "T23c not idle while holding valid data");

        // -------------------------------------------------------------------
        std::puts("\n=== T24: en gap of arbitrary length — no state disturbed ===");
        // -------------------------------------------------------------------
        // Cell holds valid data from T23; en stays low for a long gap, then a
        // new address is offered. Nothing may change until the drain.
        for (int c = 0; c < 12; ++c)
            tick(clk);
        wait(1, SC_NS);
        CHECK(valid_o.read(), "T24a data survives a 12-cycle en gap untouched");
        addr_i.write(0x3400);
        en_i.write(true); // en returns while still valid: start blocked by valid_q
        wait(1, SC_NS);
        CHECK(!m_bus.req.read(), "T24b no refetch while presenting undrained data");
        // drain it: all_valid pulse frees the slot and arms the refetch
        all_valid_i.write(true);
        tick(clk);
        all_valid_i.write(false);
        wait(1, SC_NS);
        CHECK(m_bus.req.read() && m_bus.addr.read() == 0x3400,
              "T24c the drain edge launches the refetch for the offered address");
        en_i.write(false);
        grant();
        m_bus.rvalid.write(true);
        m_bus.rdata.write(data_t(0x34ull));
        tick(clk);
        m_bus.rvalid.write(false);
        wait(1, SC_NS);
        CHECK(valid_o.read(), "T24d refetch completes normally after the gap");

        sc_stop();
    }
};

int sc_main(int, char **) {
    tb t{"tb"};
    sc_start();
    return report_and_exit();
}
