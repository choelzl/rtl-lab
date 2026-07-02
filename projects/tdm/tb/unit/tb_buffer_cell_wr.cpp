// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Unit tests for buffer_cell<BYTES_PER_ROW, IS_WRITE=true> — the pipelined
// write cell: a port-facing PRIMARY LATCH and a TDM-facing SHADOW FLUSH
// ENGINE running independently, so the parent can fill window k+1 into the
// primary while window k's shadow write drains to the bank (see
// buffer_cell.hpp's header comment).
//
// Contract under test:
//   Primary latch:  p_req with a free latch captures addr/wdata/be
//                   (invalid_o = latch free); held stable against port
//                   changes; a second p_req while full is ignored.
//   Snapshot:       reset_window_i hands the primary to the shadow and
//                   frees the primary the same edge — including when the
//                   final fill beat arrives on that very edge. addr==0
//                   (NOP padding) frees the latch without ever touching
//                   the TDM bus.
//   Shadow engine:  valid_o = shadow busy; m.req/addr/we/be/wdata driven
//                   from the shadow's own copy (stable while the primary
//                   relatches); req holds until gnt (OBI), deasserts after,
//                   rvalid completes the write and idles the shadow.
//                   Premature gnt/rvalid (OBI R-5) are ignored.
//   Overlap:        a fresh beat latches while the shadow is mid-flight;
//                   neither disturbs the other.
//
// Tests:
//   T01  Reset — latch free, shadow idle, all outputs low
//   T02  Idle — no port request, nothing happens
//   T03  Latch captures addr/wdata/be; no TDM activity before snapshot
//   T04  Latched data held against port wiggling; re-req ignored while full
//   T05  Snapshot — shadow fires TDM write with the latched values; primary
//        frees the same edge
//   T06  Snapshot with the final beat arriving the SAME edge — the shadow
//        gets the fresh beat, not a stale one
//   T07  NOP (addr==0) — snapshot frees the latch, no TDM write ever
//   T08  Shadow OBI: req holds until gnt, addr/data stable while the
//        primary relatches underneath; deasserts after gnt; rvalid idles it
//   T09  OBI R-5: rvalid before gnt ignored; gnt without req ignored
//   T10  Overlap: latch a new beat mid-flight — invalid_o/valid_o move
//        independently, shadow values undisturbed
//   T11  Batch: 32 back-to-back latch→snapshot→complete cycles (unique
//        addr/wdata/be each), then 32 more — unlimited reuse
// -----------------------------------------------------------------------------

#include "buffer_cell.hpp"
#include "obi_data.hpp"
#include "unit_test_common.hpp"
#include <cstdio>
#include <systemc.h>

static constexpr int      kBytes  = 4;
static constexpr uint32_t kBeFull = static_cast<uint32_t>((uint64_t{1} << kBytes) - 1u); // 0xF
using data_t                      = obi_data<kBytes>;
using DUT                         = buffer_cell<kBytes, true>;

static data_t make_data(uint32_t v) {
    return data_t(static_cast<unsigned long long>(v));
}

SC_MODULE(tb) {
    sc_clock        clk{"clk", 10, SC_NS};
    sc_signal<bool> rst_n{"rst_n"};

    // DUT inputs
    sc_signal<bool> all_valid_i{"all_valid_i"};
    sc_signal<bool> commit_ok_i{"commit_ok_i"}; // held 0: the echo has its own tests via tb_buffer
                                                // // unused in write mode
    sc_signal<bool> reset_window_i{"reset_window_i"}; // the snapshot pulse

    // Unused read-mode inputs (held at 0)
    sc_signal<uint64_t> addr_i{"addr_i"};
    sc_signal<bool>     en_i{"en_i"};

    // DUT outputs
    sc_signal<bool> valid_o{"valid_o"};     // shadow busy
    sc_signal<bool> invalid_o{"invalid_o"}; // primary latch free

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

    void do_reset(int cycles = 2) {
        rst_n.write(false);
        p_bus.req.write(false);
        p_bus.addr.write(0);
        p_bus.wdata.write(data_t{0});
        p_bus.be.write(0);
        m_bus.gnt.write(false);
        m_bus.rvalid.write(false);
        all_valid_i.write(false);
        reset_window_i.write(false);
        for (int i = 0; i < cycles; ++i)
            wait(clk.posedge_event());
        rst_n.write(true);
        tick(clk);
    }

    // Present one fill beat for exactly one edge (the parent's granted-group
    // pattern: p_req routed to this cell for the accepted cycle only).
    void latch_beat(uint64_t addr, data_t wdata, uint32_t be = kBeFull) {
        p_bus.req.write(true);
        p_bus.addr.write(addr);
        p_bus.wdata.write(wdata);
        p_bus.be.write(be);
        tick(clk);
        p_bus.req.write(false);
    }

    // One-edge snapshot pulse.
    void snapshot() {
        reset_window_i.write(true);
        tick(clk);
        reset_window_i.write(false);
    }

    void grant() {
        m_bus.gnt.write(true);
        tick(clk);
        m_bus.gnt.write(false);
    }

    void ack() { // bank write response (rvalid, no data)
        m_bus.rvalid.write(true);
        tick(clk);
        m_bus.rvalid.write(false);
    }

    void run() {
        // -------------------------------------------------------------------
        std::puts("\n=== T01: Reset — latch free, shadow idle ===");
        // -------------------------------------------------------------------
        do_reset();
        CHECK(!m_bus.req.read(), "T01a m_bus.req=0");
        CHECK(!m_bus.we.read(), "T01b m_bus.we=0");
        CHECK(!valid_o.read(), "T01c valid_o=0 (shadow idle)");
        CHECK(invalid_o.read(), "T01d invalid_o=1 (primary latch free)");

        // -------------------------------------------------------------------
        std::puts("\n=== T02: Idle — no request, nothing moves ===");
        // -------------------------------------------------------------------
        for (int i = 0; i < 3; ++i)
            tick(clk);
        CHECK(!m_bus.req.read() && invalid_o.read() && !valid_o.read(),
              "T02 stays idle without a port request");

        // -------------------------------------------------------------------
        std::puts("\n=== T03: Latch captures the beat; no TDM before snapshot ===");
        // -------------------------------------------------------------------
        latch_beat(0x40, make_data(0xAA110001), 0x5);
        CHECK(!invalid_o.read(), "T03a latch full after the accepted beat");
        CHECK(!m_bus.req.read(), "T03b no TDM write yet (waits for the snapshot)");
        CHECK(!valid_o.read(), "T03c shadow still idle");
        for (int i = 0; i < 3; ++i)
            tick(clk);
        CHECK(!m_bus.req.read(), "T03d still quiet — latched state is stable");

        // -------------------------------------------------------------------
        std::puts("\n=== T04: Latched data held; re-req while full ignored ===");
        // -------------------------------------------------------------------
        p_bus.addr.write(0x9999);
        p_bus.wdata.write(make_data(0xDEADBEEF));
        p_bus.be.write(0x3);
        p_bus.req.write(true); // parent would never do this; must be ignored
        tick(clk);
        p_bus.req.write(false);
        CHECK(!invalid_o.read(), "T04a still latched");
        // The held values are only observable via the TDM side after a
        // snapshot — do it and check the ORIGINAL beat comes out.
        snapshot();
        CHECK(m_bus.req.read(), "T04b shadow fires after snapshot");
        CHECK(m_bus.addr.read() == 0x40, "T04c original address (port wiggle ignored)");
        CHECK(m_bus.wdata.read() == make_data(0xAA110001), "T04d original data");
        CHECK(m_bus.be.read() == 0x5, "T04e original byte-enable");
        CHECK(m_bus.we.read(), "T04f m_bus.we=1 (write)");
        CHECK(invalid_o.read(), "T04g primary freed by the snapshot");
        CHECK(valid_o.read(), "T04h shadow busy");
        grant();
        ack();
        CHECK(!valid_o.read(), "T04i shadow idle after gnt+rvalid");

        // -------------------------------------------------------------------
        std::puts("\n=== T05: plain latch → snapshot → complete ===");
        // -------------------------------------------------------------------
        do_reset();
        latch_beat(0x80, make_data(0xBB220002));
        snapshot();
        CHECK(m_bus.req.read() && m_bus.addr.read() == 0x80, "T05a TDM write for the latched beat");
        CHECK(invalid_o.read() && valid_o.read(), "T05b primary free, shadow busy — same edge");
        grant();
        ack();
        CHECK(!valid_o.read() && invalid_o.read(), "T05c both stages idle after completion");

        // -------------------------------------------------------------------
        std::puts("\n=== T06: final beat and snapshot on the SAME edge ===");
        // -------------------------------------------------------------------
        // The window's last fill group is granted the same edge its snapshot
        // fires (see buffer.hpp) — the shadow must get the FRESH beat.
        do_reset();
        p_bus.req.write(true);
        p_bus.addr.write(0xC0);
        p_bus.wdata.write(make_data(0xCC330003));
        p_bus.be.write(kBeFull);
        reset_window_i.write(true);
        tick(clk);
        p_bus.req.write(false);
        reset_window_i.write(false);
        CHECK(m_bus.req.read() && m_bus.addr.read() == 0xC0,
              "T06a shadow carries the beat that arrived on the snapshot edge");
        CHECK(m_bus.wdata.read() == make_data(0xCC330003), "T06b with its data");
        CHECK(invalid_o.read(), "T06c and the primary is already free again");
        grant();
        ack();

        // -------------------------------------------------------------------
        std::puts("\n=== T07: NOP (addr==0) never touches the bus ===");
        // -------------------------------------------------------------------
        do_reset();
        latch_beat(0x0, make_data(0));
        CHECK(!invalid_o.read(), "T07a NOP occupies the latch like any beat");
        snapshot();
        CHECK(invalid_o.read(), "T07b snapshot frees it");
        bool nop_quiet = true;
        for (int i = 0; i < 4; ++i) {
            tick(clk);
            nop_quiet &= !m_bus.req.read() && !valid_o.read();
        }
        CHECK(nop_quiet, "T07c no TDM write, shadow never busy");

        // -------------------------------------------------------------------
        std::puts("\n=== T08: shadow OBI discipline ===");
        // -------------------------------------------------------------------
        do_reset();
        latch_beat(0x100, make_data(0xDD440004));
        snapshot();
        bool held = true;
        for (int i = 0; i < 3; ++i) {
            // Port keeps moving underneath (the parent is filling the next
            // window into the primary) — the in-flight write must not care.
            p_bus.addr.write(0x5000 + 0x10 * static_cast<uint64_t>(i));
            p_bus.wdata.write(make_data(0x12340000u + static_cast<uint32_t>(i)));
            tick(clk);
            held &= m_bus.req.read() && (m_bus.addr.read() == 0x100) &&
                    (m_bus.wdata.read() == make_data(0xDD440004));
        }
        CHECK(held, "T08a req+addr+data held stable until gnt, port wiggle ignored");
        // The grant ends the shadow's job: the bank fabric samples the
        // payload the edge after, so busy clears at the grant — no rvalid
        // tracking (the returning rvalid finds nothing busy, T09 pins that
        // it's ignored). valid_o even previews the grant combinationally
        // (the parent may reuse this shadow the very next edge).
        m_bus.gnt.write(true);
        wait(1, SC_NS);
        CHECK(!valid_o.read(), "T08b valid_o drops the same cycle the gnt is live (preview)");
        tick(clk);
        m_bus.gnt.write(false);
        CHECK(!m_bus.req.read(), "T08c req deasserts after gnt");
        CHECK(!valid_o.read(), "T08d shadow free from the grant on");
        ack();
        CHECK(!valid_o.read() && !m_bus.req.read(), "T08e the returning rvalid changes nothing");

        // -------------------------------------------------------------------
        std::puts("\n=== T09: OBI R-5 — premature rvalid/gnt ignored ===");
        // -------------------------------------------------------------------
        do_reset();
        latch_beat(0x140, make_data(0xEE550005));
        // rvalid with no snapshot yet: nothing in flight, must be ignored.
        ack();
        CHECK(!valid_o.read() && !invalid_o.read(), "T09a stray rvalid before snapshot ignored");
        snapshot();
        // rvalid BEFORE gnt: must be ignored (R-5).
        ack();
        CHECK(valid_o.read() && m_bus.req.read(),
              "T09b rvalid before gnt ignored — still requesting");
        grant();
        ack();
        CHECK(!valid_o.read(), "T09c completes at its gnt (rvalid tracked by nobody)");
        // gnt with nothing in flight: ignored.
        grant();
        CHECK(!valid_o.read() && !m_bus.req.read(), "T09d stray gnt ignored");

        // -------------------------------------------------------------------
        std::puts("\n=== T10: fill overlaps the in-flight flush ===");
        // -------------------------------------------------------------------
        do_reset();
        latch_beat(0x180, make_data(0xFF660006));
        snapshot();                               // shadow in flight
        latch_beat(0x1C0, make_data(0xAB770007)); // next window's beat arrives already
        CHECK(!invalid_o.read(), "T10a primary holds the NEW beat");
        CHECK(valid_o.read(), "T10b while the shadow is still busy with the OLD one");
        CHECK(m_bus.req.read() && m_bus.addr.read() == 0x180 &&
                  m_bus.wdata.read() == make_data(0xFF660006),
              "T10c TDM side still shows the old write, undisturbed");
        grant();
        ack();
        CHECK(!valid_o.read() && !invalid_o.read(),
              "T10d shadow done; new beat still latched, ready for ITS snapshot");
        snapshot();
        CHECK(m_bus.req.read() && m_bus.addr.read() == 0x1C0, "T10e and it flushes next");
        grant();
        ack();

        // -------------------------------------------------------------------
        std::puts("\n=== T11: 2x32 back-to-back reuse ===");
        // -------------------------------------------------------------------
        do_reset();
        bool batch_ok = true;
        for (int round = 0; round < 2; ++round) {
            for (int i = 0; i < 32; ++i) {
                const uint64_t a = 0x1000 + 0x40 * static_cast<uint64_t>(round * 32 + i);
                const data_t   d = make_data(0x50000000u + static_cast<uint32_t>(round * 32 + i));
                const uint32_t b = (i % 2) ? 0xFu : 0x3u;
                latch_beat(a, d, b);
                snapshot();
                batch_ok &= m_bus.req.read() && (m_bus.addr.read() == a) &&
                            (m_bus.wdata.read() == d) && (m_bus.be.read() == b);
                grant();
                ack();
                batch_ok &= !valid_o.read() && invalid_o.read();
            }
            CHECK(batch_ok, round == 0 ? "T11a first 32 transactions all correct"
                                       : "T11b second 32 — unlimited reuse");
        }

        // -------------------------------------------------------------------
        std::puts("\n=== T12: we_o is 1 for the whole flush, and only then ===");
        // -------------------------------------------------------------------
        do_reset();
        wait(1, SC_NS);
        CHECK(!m_bus.we.read() || !m_bus.req.read(), "T12a no write presented while idle");
        latch_beat(0x5000, make_data(0xD00D), 0xF);
        snapshot();
        wait(1, SC_NS);
        CHECK(m_bus.req.read() && m_bus.we.read(), "T12b flush presents we=1 with the request");
        grant();
        wait(1, SC_NS);
        CHECK(!m_bus.req.read(), "T12c request (and its we) gone after the grant");
        ack();

        // -------------------------------------------------------------------
        std::puts("\n=== T13: NOP primary (addr=0) — snapshot arms NO shadow ===");
        // -------------------------------------------------------------------
        do_reset();
        latch_beat(0, make_data(0xFEED), 0xF); // padding beat
        snapshot();
        wait(1, SC_NS);
        CHECK(!m_bus.req.read(), "T13a a NOP beat never reaches the TDM bus");
        CHECK(invalid_o.read(), "T13b primary freed by the snapshot regardless");
        CHECK(!valid_o.read(), "T13c no shadow busy for a NOP");

        // -------------------------------------------------------------------
        std::puts("\n=== T14: zero byte-enable is latched and flushed verbatim ===");
        // -------------------------------------------------------------------
        do_reset();
        latch_beat(0x6000, make_data(0xABCD), 0x0);
        snapshot();
        wait(1, SC_NS);
        CHECK(m_bus.req.read() && m_bus.be.read() == 0x0,
              "T14 be=0 passes through unmodified (the bank treats it as a no-op)");
        grant();
        ack();

        sc_stop();
    }
};

int sc_main(int, char **) {
    tb t{"tb"};
    sc_start();
    return report_and_exit();
}
