// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Description:
//   Prefetch / write buffer — one instance per port group.
//   Composed of NUM_TDM buffer_cell instances (one per TDM slot), plus
//   combinatorial logic to orchestrate drain windows and drive port outputs.
//
//   Template parameters:
//     NUM_REQ       — OBI beats per port
//     PORT_COUNT    — maximum ports/groups connected to this buffer
//     BYTES_PER_ROW — data width in bytes
//     NUM_TDM       — TDM lanes and slots in one fetch window
//     IS_WRITE      — false (default): read-prefetch buffer; true: write buffer
//
//   Window drain protocol:
//     Cells are drained sequentially in groups of active_beats() = active_ports()
//     * NUM_REQ.  rd_ptr_q tracks the current group's base slot.  When the last
//     group drains, reset_window is asserted and all cells reset to MISSING for
//     the next window; rd_ptr_q wraps back to 0 in the same cycle.
//
//   Read mode (IS_WRITE=false):
//     Cells proactively fetch TDM data when fetch_addr_valid_i fires.  Group
//     drains when all cells are VALID and all active ports have asserted p_req_i.
//     p_rdata_o carries the fetched data.
//
//   Write mode (IS_WRITE=true) — three phases:
//     FILL    : ports write one group (n_beats) at a time into the buffer.
//               All ports in the active group must assert p_req_i together;
//               addr/wdata/be are latched into the cells and p_gnt_o fires.
//               The fill-ptr advances by n_beats each accepted group until the
//               entire buffer (NUM_TDM slots) is populated.
//     FLUSH   : buffer full; p_req_i is re-asserted to all cells, which start
//               their TDM write transactions simultaneously.  The buffer waits
//               until every cell reaches VALID (TDM write ack received).
//     RESPOND : p_rvalid_o is sent back to ports group-by-group using the same
//               rd_ptr drain loop as READ mode.  p_rdata_o is always 0.
//     fetch_addr_i and fetch_addr_valid_i are present but unused in write mode.
//
//   active_mode encoding (matches buffer.sv):
//     0   → 1 active port
//     1   → 2 active ports
//     2/3 → 4 active ports (clamped to PORT_COUNT)
// -----------------------------------------------------------------------------

#ifndef BUFFER_HPP
#define BUFFER_HPP

#include <systemc.h>

#include "buffer_cell.hpp"
#include "obi_data.hpp"
#include <algorithm>
#include <cstdint>
#include <cstdio>

template <int NUM_REQ = 4, int PORT_COUNT = 4, int BYTES_PER_ROW = 4 * 4, int NUM_TDM = 32,
          bool IS_WRITE = false>
SC_MODULE(buffer) {
    static constexpr int BUFFER_SIZE = NUM_TDM;
    static constexpr int NUM_IO      = PORT_COUNT * NUM_REQ;
    using data_t                     = obi_data<BYTES_PER_ROW>;
    using cell_t                     = buffer_cell<BYTES_PER_ROW, IS_WRITE>;

    static_assert(NUM_REQ >= 1, "NUM_REQ must be >= 1");
    static_assert(PORT_COUNT >= 1 && PORT_COUNT <= 4, "PORT_COUNT must be in [1, 4]");
    static_assert(NUM_TDM >= NUM_IO && NUM_TDM % NUM_IO == 0,
                  "NUM_TDM must be a positive multiple of NUM_IO");
    static_assert(BYTES_PER_ROW >= 1 && BYTES_PER_ROW <= 32, "BYTES_PER_ROW must be in [1, 32]");

    // -----------------------------------------------------------------------
    // Clock / reset / mode
    // -----------------------------------------------------------------------
    sc_in<bool>     clk_i;
    sc_in<bool>     rst_ni;
    sc_in<uint32_t> active_mode;

    // -----------------------------------------------------------------------
    // Port-facing OBI  (NUM_IO = PORT_COUNT * NUM_REQ ports)
    // -----------------------------------------------------------------------
    sc_in<bool>     p_req_i[NUM_IO];
    sc_in<uint64_t> p_addr_i[NUM_IO];
    sc_in<uint32_t> p_be_i[NUM_IO];
    sc_in<data_t>   p_wdata_i[NUM_IO];
    sc_out<bool>    p_gnt_o[NUM_IO];
    sc_out<bool>    p_rvalid_o[NUM_IO];
    sc_out<data_t>  p_rdata_o[NUM_IO];

    // -----------------------------------------------------------------------
    // TDM-facing OBI  (NUM_TDM-wide; one full OBI port per slot)
    // -----------------------------------------------------------------------
    sc_out<bool>     m_req_o[NUM_TDM];
    sc_out<uint64_t> m_addr_o[NUM_TDM];
    sc_out<bool>     m_we_o[NUM_TDM];
    sc_out<uint32_t> m_be_o[NUM_TDM];
    sc_out<data_t>   m_wdata_o[NUM_TDM]; // driven in write mode; 0 in read mode
    sc_in<bool>      m_gnt_i[NUM_TDM];
    sc_in<bool>      m_rvalid_i[NUM_TDM];
    sc_in<data_t>    m_rdata_i[NUM_TDM];

    // Per-slot fetch addresses — used in read mode; present but unused in write mode
    sc_in<uint64_t> fetch_addr_i[NUM_TDM];
    sc_in<bool>     fetch_addr_valid_i;

    // =======================================================================
    // Internal
    // =======================================================================
  private:
    // -----------------------------------------------------------------------
    // Cell array
    // -----------------------------------------------------------------------
    cell_t *cells[NUM_TDM];

    // -----------------------------------------------------------------------
    // Interconnect signals (buffer → cells)
    // -----------------------------------------------------------------------
    sc_signal<bool>     cell_all_valid_s[NUM_TDM];
    sc_signal<bool>     cell_p_req_s[NUM_TDM];
    sc_signal<uint64_t> cell_p_addr_s[NUM_TDM];
    sc_signal<data_t>   cell_p_wdata_s[NUM_TDM]; // write mode: port wdata routed to active cell
    sc_signal<uint32_t> cell_p_be_s[NUM_TDM];    // write mode: port be routed to active cell
    sc_signal<bool>     cell_reset_window_s;

    // -----------------------------------------------------------------------
    // Interconnect signals (cells → buffer)
    // -----------------------------------------------------------------------
    sc_signal<bool>   cell_valid_s[NUM_TDM];
    sc_signal<data_t> cell_p_rdata_s[NUM_TDM];

    // Unused sink signals — required to bind cell output ports that the buffer
    // overrides directly (p_gnt_o, p_rvalid_o) or does not use (invalid_o).
    sc_signal<bool> cell_invalid_s[NUM_TDM];
    sc_signal<bool> cell_p_rvalid_s[NUM_TDM];
    sc_signal<bool> cell_p_gnt_s[NUM_TDM];

    // -----------------------------------------------------------------------
    // Pointer register (rd_ptr in read mode; fill-ptr during FILL, respond-ptr
    // during RESPOND in write mode)
    // -----------------------------------------------------------------------
    sc_signal<int> rd_ptr_q;

    // -----------------------------------------------------------------------
    // Write-mode state machine
    //   FILL    — accepting port writes group-by-group
    //   FLUSH   — buffer full; all cells writing to TDM simultaneously
    //   RESPOND — all TDM writes done; sending p_rvalid group-by-group
    // -----------------------------------------------------------------------
    enum class BufPhase : uint8_t { FILL = 0, FLUSH = 1, RESPOND = 2 };
    sc_signal<uint8_t> phase_q; // only meaningful in IS_WRITE mode

    SC_HAS_PROCESS(buffer);

  public:
    explicit buffer(sc_module_name nm) : sc_module(nm) {
        for (int t = 0; t < NUM_TDM; ++t) {
            char name[32];
            std::snprintf(name, sizeof(name), "cell_%d", t);
            cells[t] = new cell_t(name);

            cells[t]->clk_i(clk_i);
            cells[t]->rst_ni(rst_ni);

            cells[t]->addr_i(fetch_addr_i[t]);
            cells[t]->en_i(fetch_addr_valid_i);

            cells[t]->m_req_o(m_req_o[t]);
            cells[t]->m_addr_o(m_addr_o[t]);
            cells[t]->m_we_o(m_we_o[t]);
            cells[t]->m_be_o(m_be_o[t]);
            cells[t]->m_wdata_o(m_wdata_o[t]);
            cells[t]->m_gnt_i(m_gnt_i[t]);
            cells[t]->m_rvalid_i(m_rvalid_i[t]);
            cells[t]->m_rdata_i(m_rdata_i[t]);

            cells[t]->p_req_i(cell_p_req_s[t]);
            cells[t]->p_addr_i(cell_p_addr_s[t]);
            cells[t]->p_wdata_i(cell_p_wdata_s[t]);
            cells[t]->p_be_i(cell_p_be_s[t]);
            cells[t]->p_gnt_o(cell_p_gnt_s[t]);
            cells[t]->p_rvalid_o(cell_p_rvalid_s[t]);
            cells[t]->p_rdata_o(cell_p_rdata_s[t]);
            cells[t]->all_valid_i(cell_all_valid_s[t]);
            cells[t]->reset_window_i(cell_reset_window_s);

            cells[t]->valid_o(cell_valid_s[t]);
            cells[t]->invalid_o(cell_invalid_s[t]);
        }

        SC_METHOD(comb_proc);
        sensitive << active_mode << rd_ptr_q << phase_q;
        for (int t = 0; t < NUM_TDM; ++t)
            sensitive << cell_valid_s[t];
        for (int i = 0; i < NUM_IO; ++i) {
            sensitive << p_req_i[i] << p_addr_i[i];
            if constexpr (IS_WRITE)
                sensitive << p_wdata_i[i] << p_be_i[i];
        }

        SC_THREAD(seq_proc);
        sensitive << clk_i.pos();
        async_reset_signal_is(rst_ni, false);
    }

    ~buffer() {
        for (int t = 0; t < NUM_TDM; ++t)
            delete cells[t];
    }

  public:
    struct snapshot_t {
        int  rd_ptr;
        int  n_valid;
        int  n_beats;
        bool window_reset;
    };

    snapshot_t snapshot() const {
        int n = 0;
        for (int t = 0; t < NUM_TDM; ++t)
            if (cell_valid_s[t].read())
                ++n;
        return {rd_ptr_q.read(), n, active_beats(), cell_reset_window_s.read()};
    }

  private:
    int active_ports() const {
        switch (active_mode.read() & 0x3u) {
        case 0:
            return std::min(PORT_COUNT, 1);
        case 1:
            return std::min(PORT_COUNT, 2);
        default:
            return std::min(PORT_COUNT, 4);
        }
    }

    int active_beats() const {
        return active_ports() * NUM_REQ;
    }

    struct GroupState {
        bool ports_req;
        bool cells_valid;
        bool can_drain;
        bool is_last;
    };

    GroupState eval_group(int base, int n_beats) const {
        bool ports_req = true;
        for (int i = 0; i < n_beats; ++i)
            if (!p_req_i[i].read()) {
                ports_req = false;
                break;
            }

        bool cells_valid = (base + n_beats <= BUFFER_SIZE);
        for (int i = 0; cells_valid && i < n_beats; ++i)
            cells_valid = cell_valid_s[base + i].read();

        return {ports_req, cells_valid, ports_req && cells_valid, base + n_beats == BUFFER_SIZE};
    }

    // -----------------------------------------------------------------------
    // Combinational process — drives cell controls and port grant
    // -----------------------------------------------------------------------
    void comb_proc() {
        int n_beats = active_beats();
        int base    = rd_ptr_q.read();

        if constexpr (!IS_WRITE) {
            // ------------------------------------------------------------------
            // READ mode: TDM pre-filled → drain to ports group-by-group
            // ------------------------------------------------------------------
            auto grp = eval_group(base, n_beats);

            cell_reset_window_s.write(grp.can_drain && grp.is_last);

            for (int t = 0; t < NUM_TDM; ++t) {
                bool in_grp = (t >= base) && (t < base + n_beats);
                int  lane   = t - base;
                cell_all_valid_s[t].write(grp.can_drain && in_grp);
                cell_p_req_s[t].write(grp.ports_req && in_grp);
                cell_p_addr_s[t].write(in_grp ? p_addr_i[lane].read() : uint64_t{0});
            }

            for (int i = 0; i < NUM_IO; ++i)
                p_gnt_o[i].write(grp.can_drain && (i < n_beats));

        } else {
            // ------------------------------------------------------------------
            // WRITE mode: fill from ports group-by-group → flush to TDM → respond
            // ------------------------------------------------------------------
            auto ph = static_cast<BufPhase>(phase_q.read());

            // Defaults — overridden per phase below
            cell_reset_window_s.write(false);
            for (int t = 0; t < NUM_TDM; ++t) {
                cell_all_valid_s[t].write(false);
                cell_p_req_s[t].write(false);
                cell_p_addr_s[t].write(uint64_t{0});
                cell_p_wdata_s[t].write(data_t{0});
                cell_p_be_s[t].write(uint32_t{0});
            }
            for (int i = 0; i < NUM_IO; ++i) {
                p_gnt_o[i].write(false);
            }

            if (ph == BufPhase::FILL) {
                // Accept one group of port writes; grant when all ports request.
                bool ports_req = true;
                for (int i = 0; i < n_beats; ++i)
                    if (!p_req_i[i].read()) {
                        ports_req = false;
                        break;
                    }

                for (int t = 0; t < NUM_TDM; ++t) {
                    bool in_grp = (t >= base) && (t < base + n_beats);
                    int  lane   = t - base;
                    cell_p_req_s[t].write(ports_req && in_grp);
                    cell_p_addr_s[t].write(in_grp ? p_addr_i[lane].read() : uint64_t{0});
                    cell_p_wdata_s[t].write(in_grp ? p_wdata_i[lane].read() : data_t{0});
                    cell_p_be_s[t].write(in_grp ? p_be_i[lane].read() : uint32_t{0});
                }
                for (int i = 0; i < NUM_IO; ++i)
                    p_gnt_o[i].write(ports_req && (i < n_beats));

            } else if (ph == BufPhase::FLUSH) {
                // Re-assert p_req_i on all cells; LATCHED cells see this and
                // transition to REQUESTING (else-if in cell seq_proc prevents
                // IDLE cells from spuriously capturing new data).
                for (int t = 0; t < NUM_TDM; ++t)
                    cell_p_req_s[t].write(true);

            } else { // RESPOND
                // Send p_rvalid to current group, mirroring the READ drain logic.
                bool cells_valid = (base + n_beats <= BUFFER_SIZE);
                for (int i = 0; cells_valid && i < n_beats; ++i)
                    cells_valid = cell_valid_s[base + i].read();

                bool is_last = (base + n_beats == BUFFER_SIZE);
                cell_reset_window_s.write(cells_valid && is_last);

                for (int t = 0; t < NUM_TDM; ++t) {
                    bool in_grp = (t >= base) && (t < base + n_beats);
                    cell_all_valid_s[t].write(cells_valid && in_grp);
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Sequential process — advances rd_ptr and registers port response
    // -----------------------------------------------------------------------
    void seq_proc() {
        rd_ptr_q.write(0);
        if constexpr (IS_WRITE)
            phase_q.write(static_cast<uint8_t>(BufPhase::FILL));
        for (int i = 0; i < NUM_IO; ++i) {
            p_rvalid_o[i].write(false);
            p_rdata_o[i].write(data_t{0});
        }
        wait();

        while (true) {
            int n_beats = active_beats();
            int base    = rd_ptr_q.read();

            bool   rvalid_next[NUM_IO] = {};
            data_t rdata_next[NUM_IO]  = {};

            if constexpr (!IS_WRITE) {
                // -------------------------------------------------------
                // READ mode: drain TDM-prefilled cells to ports
                // -------------------------------------------------------
                auto grp = eval_group(base, n_beats);

                if (grp.can_drain) {
                    for (int i = 0; i < n_beats; ++i) {
                        rvalid_next[i] = true;
                        rdata_next[i]  = cell_p_rdata_s[base + i].read();
                    }
                    base = grp.is_last ? 0 : base + n_beats;
                }

                rd_ptr_q.write(base);

            } else {
                // -------------------------------------------------------
                // WRITE mode state machine
                // -------------------------------------------------------
                auto     ph       = static_cast<BufPhase>(phase_q.read());
                BufPhase next_ph  = ph;
                int      next_ptr = base;

                if (ph == BufPhase::FILL) {
                    // Advance fill-ptr when all ports in the current group
                    // are requesting (comb_proc grants them this cycle).
                    bool ports_req = true;
                    for (int i = 0; i < n_beats; ++i)
                        if (!p_req_i[i].read()) {
                            ports_req = false;
                            break;
                        }

                    if (ports_req) {
                        next_ptr = base + n_beats;
                        if (next_ptr == BUFFER_SIZE) {
                            next_ph  = BufPhase::FLUSH;
                            next_ptr = 0;
                        }
                    }

                } else if (ph == BufPhase::FLUSH) {
                    // Wait until every cell has received its TDM write ack.
                    bool all_valid = true;
                    for (int t = 0; t < NUM_TDM; ++t)
                        if (!cell_valid_s[t].read()) {
                            all_valid = false;
                            break;
                        }

                    if (all_valid)
                        next_ph = BufPhase::RESPOND;
                    // next_ptr stays 0 — RESPOND starts from group 0

                } else { // RESPOND
                    // Mirror READ drain: send p_rvalid group-by-group.
                    // After FLUSH all cells are VALID, so cells_valid is true
                    // every cycle until drained; one group per cycle.
                    bool cells_valid = (base + n_beats <= BUFFER_SIZE);
                    for (int i = 0; cells_valid && i < n_beats; ++i)
                        cells_valid = cell_valid_s[base + i].read();

                    if (cells_valid) {
                        for (int i = 0; i < n_beats; ++i)
                            rvalid_next[i] = true;

                        next_ptr = base + n_beats;
                        if (next_ptr == BUFFER_SIZE) {
                            next_ph  = BufPhase::FILL;
                            next_ptr = 0;
                        }
                    }
                }

                phase_q.write(static_cast<uint8_t>(next_ph));
                rd_ptr_q.write(next_ptr);
            }

            for (int i = 0; i < NUM_IO; ++i) {
                p_rvalid_o[i].write(rvalid_next[i]);
                p_rdata_o[i].write(rdata_next[i]); // always 0 in write mode
            }

            wait();
        }
    }
};

#endif // BUFFER_HPP
