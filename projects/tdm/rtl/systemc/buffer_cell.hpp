// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Description:
//   Single read/write-buffer slot.
//
//   Template parameters:
//     BYTES_PER_ROW  — data width in bytes
//     IS_WRITE       — false (default): read-prefetch cell; true: write cell
//
//   FSM:
//     MISSING  — idle; drives TDM OBI A-channel once a target address is held.
//     VALID    — TDM transaction complete; presents response when all_valid_i.
//     INVALID  — consumed by the port; waits for window reset.
//
//   Transitions:
//     MISSING  → VALID    : TDM R-channel completes (m_rvalid_i)
//     VALID    → INVALID  : window drained (all_valid_i, driven by parent buffer)
//     any      → MISSING  : reset_window_i (buffer signals end of window)
//
//   TDM OBI (this cell is manager):
//     A-channel: linear fetch sub-state drives req+addr until grant received.
//       IDLE       — waiting for address trigger
//       REQUESTING — address latched, req asserted until m_gnt_i
//       GRANTED    — grant received, req deasserted, waiting for m_rvalid_i
//     R-channel: m_rvalid_i completes the transaction.
//
//   Read mode (IS_WRITE=false):
//     Address latched from addr_i when en_i fires while IDLE.
//     m_we_o always 0; full byte-enable computed from BYTES_PER_ROW.
//     TDM rdata latched into data_q; forwarded to port on drain.
//     Same-cycle forwarding (is_fwd): when m_rvalid_i arrives while GRANTED,
//     valid_o asserts combinatorially so the parent buffer can drain in the
//     same cycle TDM fill completes.
//
//   Write mode (IS_WRITE=true):
//     Address, wdata, and be latched from p_addr_i/p_wdata_i/p_be_i when
//     p_req_i fires while IDLE (triggered by the parent buffer routing the
//     port's group request).  m_we_o=1 during REQUESTING; m_wdata_o and
//     m_be_o driven from latched values.  p_rdata_o always 0 (write response
//     carries no data).  Same-cycle forwarding applies to valid_o: when the
//     TDM write ack arrives, valid_o asserts combinatorially.
//
//   Port OBI (this cell is subordinate):
//     p_gnt_o and p_rvalid_o are sunk in buffer.hpp; the parent drives port
//     gnt/rvalid directly.  p_we_i is absent: write mode always drives m_we_o=1.
// -----------------------------------------------------------------------------

#ifndef BUFFER_CELL_HPP
#define BUFFER_CELL_HPP

#include "obi_data.hpp"
#include <cstdint>
#include <systemc.h>

template <int BYTES_PER_ROW = 16, bool IS_WRITE = false> SC_MODULE(buffer_cell) {
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
    sc_out<data_t>   m_wdata_o; // driven in write mode; 0 in read mode
    sc_in<bool>      m_gnt_i;
    sc_in<bool>      m_rvalid_i;
    sc_in<data_t>    m_rdata_i; // used in read mode; ignored in write mode

    // -----------------------------------------------------------------------
    // Port OBI
    // -----------------------------------------------------------------------
    sc_in<bool>     p_req_i;    // write mode: triggers IDLE→REQUESTING; read: unused in grant logic
    sc_in<uint64_t> p_addr_i;   // write mode: latched as address
    sc_in<data_t>   p_wdata_i;  // write mode: latched as TDM write data; ignored in read mode
    sc_in<uint32_t> p_be_i;     // write mode: latched as byte-enable; ignored in read mode
    sc_out<bool>    p_gnt_o;    // sunk in buffer.hpp; included for standalone OBI completeness
    sc_out<bool>    p_rvalid_o; // sunk in buffer.hpp; included for standalone OBI completeness
    sc_out<data_t>  p_rdata_o;  // read mode: carries fetched data; write mode: always 0

    // -----------------------------------------------------------------------
    // Address input — read mode: latched when en_i=1 and fetch phase is IDLE
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
    sc_signal<uint8_t>  fetch_q;
    sc_signal<uint64_t> addr_q;
    sc_signal<data_t>   data_q; // read mode: TDM rdata; write mode: port wdata to forward to TDM
    sc_signal<uint32_t> be_q;   // write mode: latched byte-enable; unused in read mode

    SC_HAS_PROCESS(buffer_cell);

  public:
    explicit buffer_cell(sc_module_name nm) : sc_module(nm) {
        SC_METHOD(comb_proc);
        sensitive << state_q << fetch_q << addr_q << data_q << all_valid_i << m_rvalid_i
                  << m_rdata_i << p_addr_i;
        if constexpr (IS_WRITE)
            sensitive << be_q;

        SC_THREAD(seq_proc);
        sensitive << clk_i.pos();
        async_reset_signal_is(rst_ni, false);
    }

  private:
    void comb_proc() {
        auto st    = static_cast<State>(state_q.read());
        auto fetch = static_cast<FetchPhase>(fetch_q.read());

        // Same-cycle forwarding: when m_rvalid_i arrives while GRANTED,
        // valid_o asserts combinatorially so the parent can drain immediately.
        bool is_fwd = (st == MISSING) && (fetch == GRANTED) && m_rvalid_i.read();

        valid_o.write((st == VALID) || is_fwd);
        invalid_o.write(st == INVALID);

        // ---- TDM OBI A-channel (manager) ----
        bool m_req = (st == MISSING) && (fetch == REQUESTING);
        m_req_o.write(m_req);
        m_addr_o.write(m_req ? addr_q.read() : uint64_t{0});

        if constexpr (IS_WRITE) {
            m_we_o.write(m_req);
            m_be_o.write(m_req ? be_q.read() : uint32_t{0});
            m_wdata_o.write(m_req ? data_q.read() : data_t{0});
        } else {
            m_we_o.write(false);
            m_be_o.write(m_req ? static_cast<uint32_t>((uint64_t{1} << BYTES_PER_ROW) - 1u) : 0u);
            m_wdata_o.write(data_t{0});
        }

        // ---- Port OBI ----
        bool window_ready = (st == VALID) && all_valid_i.read();
        p_gnt_o.write(window_ready);
        p_rvalid_o.write(window_ready);

        if constexpr (IS_WRITE) {
            p_rdata_o.write(data_t{0});
        } else {
            p_rdata_o.write(is_fwd ? m_rdata_i.read() : (window_ready ? data_q.read() : data_t{0}));
        }
    }

    void seq_proc() {
        state_q.write(MISSING);
        fetch_q.write(IDLE);
        addr_q.write(0);
        data_q.write(data_t{0});
        be_q.write(0);
        wait();

        while (true) {
            auto     st    = static_cast<State>(state_q.read());
            auto     fetch = static_cast<FetchPhase>(fetch_q.read());
            uint64_t addr  = addr_q.read();
            data_t   dat   = data_q.read();
            uint32_t be    = be_q.read();

            if (reset_window_i.read()) {
                st    = MISSING;
                fetch = IDLE;
            } else {
                if (st == MISSING) {
                    // IDLE → REQUESTING: latch address (and data in write mode)
                    if (fetch == IDLE) {
                        if constexpr (IS_WRITE) {
                            if (p_req_i.read()) {
                                addr  = p_addr_i.read();
                                dat   = p_wdata_i.read();
                                be    = p_be_i.read();
                                fetch = REQUESTING;
                            }
                        } else {
                            if (en_i.read()) {
                                addr  = addr_i.read();
                                fetch = REQUESTING;
                            }
                        }
                    }

                    // REQUESTING → GRANTED: TDM accepts the request
                    if (fetch == REQUESTING && m_gnt_i.read())
                        fetch = GRANTED;

                    // GRANTED → VALID: TDM response arrives
                    if (fetch == GRANTED && m_rvalid_i.read()) {
                        if constexpr (!IS_WRITE)
                            dat = m_rdata_i.read(); // latch read data
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
            be_q.write(be);

            wait();
        }
    }
};

#endif // BUFFER_CELL_HPP
