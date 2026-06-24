// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Description:
//   Single prefetch-buffer slot.
//
//   FSM:
//     MISSING  — no data; drives TDM OBI A-channel while address is held.
//     VALID    — data received; presents data when all_valid_i is asserted.
//     INVALID  — consumed by the port; waits for window reset.
//
//   Transitions:
//     MISSING  → VALID    : TDM R-channel completes (m_rvalid_i)
//     VALID    → INVALID  : window drained (all_valid_i, driven by parent buffer)
//     any      → MISSING  : reset_window_i (buffer signals end of window)
//
//   TDM OBI (this cell is manager):
//     A-channel: linear fetch sub-state drives req+addr until grant received.
//       IDLE       — waiting for address (en_i)
//       REQUESTING — address latched, req asserted until m_gnt_i
//       GRANTED    — grant received, req deasserted, waiting for m_rvalid_i
//     R-channel: m_rvalid_i completes the transaction; data latched into data_q.
//
//   Port OBI (this cell is subordinate):
//     A-channel: p_req_i, p_addr_i — OBI A-channel inputs used by this cell.
//                p_gnt_o pre-asserted when VALID and window ready (R-3.2.1).
//                p_req_i does not gate grant; grant is window-driven.
//                p_addr_i is cross-checked against addr_q at drain time
//                (simulation debug). p_we_i and p_be_i are not present:
//                this cell is read-only (m_we_o is always 0).
//     R-channel: valid_o and p_rdata_o update combinatorially when m_rvalid_i
//                arrives while GRANTED (same-cycle forwarding), so the parent
//                buffer can grant in the same cycle as TDM fill completes.
//                p_gnt_o and p_rvalid_o are bound to unused sinks in
//                buffer.hpp; the parent drives port gnt/rvalid directly.
//                data_q is not cleared on reset; stale data is gated by
//                window_ready / is_fwd before reaching the port side.
// -----------------------------------------------------------------------------

#ifndef BUFFER_CELL_HPP
#define BUFFER_CELL_HPP

#include "obi_data.hpp"
#include <cstdint>
#include <systemc.h>

template <int BYTES_PER_ROW = 16> SC_MODULE(buffer_cell) {
    static_assert(BYTES_PER_ROW >= 1 && BYTES_PER_ROW <= 32, "BYTES_PER_ROW must be in [1, 32]");

    using data_t = obi_data<BYTES_PER_ROW>;

    // -----------------------------------------------------------------------
    // Clock / reset
    // -----------------------------------------------------------------------
    sc_in<bool> clk_i;
    sc_in<bool> rst_ni;

    // -----------------------------------------------------------------------
    // TDM OBI (cell → TDM)
    // -----------------------------------------------------------------------
    sc_out<bool>     m_req_o;
    sc_out<uint64_t> m_addr_o;
    sc_out<bool>     m_we_o;
    sc_out<uint32_t> m_be_o;
    sc_in<bool>      m_gnt_i;
    sc_in<bool>      m_rvalid_i;
    sc_in<data_t>    m_rdata_i;

    // -----------------------------------------------------------------------
    // Port OBI
    // -----------------------------------------------------------------------
    sc_in<bool>     p_req_i;    // not used in grant logic (grant is window-driven)
    sc_in<uint64_t> p_addr_i;   // cross-checked vs addr_q at drain
    sc_out<bool>    p_gnt_o;    // sunk in buffer.hpp; included for standalone OBI completeness
    sc_out<bool>    p_rvalid_o; // sunk in buffer.hpp; included for standalone OBI completeness
    sc_out<data_t>  p_rdata_o;

    // -----------------------------------------------------------------------
    // Address input — latched when en_i=1 and fetch phase is IDLE
    // -----------------------------------------------------------------------
    sc_in<uint64_t> addr_i;
    sc_in<bool>     en_i;

    // -----------------------------------------------------------------------
    // Window control
    // -----------------------------------------------------------------------
    sc_in<bool> all_valid_i;    // all cells in the access window are VALID
    sc_in<bool> reset_window_i; // unconditionally reset any state to MISSING

    // -----------------------------------------------------------------------
    // FSM state outputs (combinatorial)
    // -----------------------------------------------------------------------
    sc_out<bool> valid_o;
    sc_out<bool> invalid_o;

    // =======================================================================
    // Internal
    // =======================================================================
  private:
    enum State : uint8_t { MISSING = 0, VALID = 1, INVALID = 2 };
    enum FetchPhase : uint8_t { IDLE = 0, REQUESTING = 1, GRANTED = 2 };

    sc_signal<uint8_t>  state_q;
    sc_signal<uint8_t>  fetch_q; // FetchPhase — only meaningful in MISSING
    sc_signal<uint64_t> addr_q;
    sc_signal<data_t>   data_q;

    SC_HAS_PROCESS(buffer_cell);

  public:
    explicit buffer_cell(sc_module_name nm) : sc_module(nm) {
        SC_METHOD(comb_proc);
        sensitive << state_q << fetch_q << addr_q << data_q << all_valid_i << m_rvalid_i
                  << m_rdata_i << p_addr_i;

        SC_THREAD(seq_proc);
        sensitive << clk_i.pos();
        async_reset_signal_is(rst_ni, false);
    }

  private:
    void comb_proc() {
        auto st    = static_cast<State>(state_q.read());
        auto fetch = static_cast<FetchPhase>(fetch_q.read());

        // Same-cycle forwarding: if m_rvalid_i arrives while GRANTED, propagate
        // valid_o and rdata combinatorially so the parent buffer can observe the
        // fill in the same delta and grant in the same cycle (OBI gnt cycle N,
        // rvalid cycle N+1 from buffer.seq_proc).
        bool is_fwd = (st == MISSING) && (fetch == GRANTED) && m_rvalid_i.read();

        valid_o.write((st == VALID) || is_fwd);
        invalid_o.write(st == INVALID);

        // ---- TDM OBI A-channel (manager) ----
        bool m_req = (st == MISSING) && (fetch == REQUESTING);
        m_req_o.write(m_req);
        m_addr_o.write(m_req ? addr_q.read() : uint64_t{0});
        m_we_o.write(false);
        m_be_o.write(m_req ? static_cast<uint32_t>((uint64_t{1} << BYTES_PER_ROW) - 1u) : 0u);

        // ---- Port OBI ----
        bool window_ready = (st == VALID) && all_valid_i.read();
        p_gnt_o.write(window_ready);
        p_rvalid_o.write(window_ready);
        p_rdata_o.write(is_fwd ? m_rdata_i.read() : (window_ready ? data_q.read() : data_t{0}));

        // ---- Drain-time cross-checks (simulation debug) ----
        if ((window_ready || is_fwd) && p_addr_i.read() != addr_q.read())
            SC_REPORT_ERROR("buffer_cell", "p_addr_i != addr_q at drain");
    }

    void seq_proc() {
        state_q.write(MISSING);
        fetch_q.write(IDLE);
        addr_q.write(0);
        data_q.write(data_t{0});
        wait();

        while (true) {
            auto     st    = static_cast<State>(state_q.read());
            auto     fetch = static_cast<FetchPhase>(fetch_q.read());
            uint64_t addr  = addr_q.read();
            data_t   dat   = data_q.read();

            if (reset_window_i.read()) {
                st    = MISSING;
                fetch = IDLE;
            } else {
                if (st == MISSING) {
                    // IDLE → REQUESTING: latch address
                    if (fetch == IDLE && en_i.read()) {
                        addr  = addr_i.read();
                        fetch = REQUESTING;
                    }
                    // REQUESTING → GRANTED: TDM accepts the request
                    if (fetch == REQUESTING && m_gnt_i.read())
                        fetch = GRANTED;
                    // GRANTED → VALID: data arrives from TDM
                    if (fetch == GRANTED && m_rvalid_i.read()) {
                        dat   = m_rdata_i.read();
                        st    = VALID;
                        fetch = IDLE;
                    }
                }

                // VALID → INVALID: window drained
                if (st == VALID && all_valid_i.read())
                    st = INVALID;
            }

            state_q.write(st);
            fetch_q.write(fetch);
            addr_q.write(addr);
            data_q.write(dat);

            wait();
        }
    }
};

#endif // BUFFER_CELL_HPP
