// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   Native SystemC Address Generation Unit (AGU) — a non-synthesizable
//   verification driver for the crossbar DUT (lives under tb/systemc/). It is an
//   OBI manager with N_REQ request ports that replays a CSV memory trace
//   (addr,we,data) and drives the simplified single-channel OBI protocol (see
//   doc/specs/obi.md):
//
//     request  (AGU -> crossbar) : req_o, addr_o, we_o, be_o, wdata_o   [N_REQ]
//     response (crossbar -> AGU) : gnt_i, rvalid_i, rdata_i             [N_REQ]
//
//   Operation:
//     - The trace is consumed in groups of N_REQ rows — one row per port. The
//       AGU issues the whole group at once, one independent OBI request per
//       port (addr_o is the GLOBAL byte address from the trace; the crossbar
//       decodes bank/row).
//     - Lock-step: the AGU does NOT load/issue the next group until ALL ports
//       of the current group have completed (their rvalid received). A port
//       held off by arbitration simply keeps its request asserted until granted.
//     - Per-port handshake: drive req until gnt (accept), then wait for rvalid.
//       Address-phase signals are held stable (registered) until granted.
//     - Writes (we==1) drive the trace's `data` column as wdata, with all byte
//       enables set; reads (we==0) ignore wdata.
//     - Every completed access is recorded (cycle, addr, we, data) and, at end
//       of simulation, dumped to the output log given to the constructor
//       (e.g. out_N.log) for manual inspection — writes show the value written,
//       reads the value returned, so read-back can be checked by eye.
//     - `done_o` is raised once the whole trace has been consumed.
//
//   Reset is active-low (rst_ni).
//
// Template parameters:
//   N_REQ      - request ports issued per cycle (group size) (default 4)
//   WORD_BYTES - bytes per word / OBI data beat              (default 4)
// -----------------------------------------------------------------------------

#ifndef AGU_HPP
#define AGU_HPP

#include <systemc.h>

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

template <int N_REQ = 4, int WORD_BYTES = 4>
SC_MODULE(agu) {
    sc_in<bool>      clk_i;
    sc_in<bool>      rst_ni;
    sc_out<bool>     req_o[N_REQ];
    sc_out<uint64_t> addr_o[N_REQ];
    sc_out<bool>     we_o[N_REQ];
    sc_out<uint32_t> be_o[N_REQ];
    sc_out<uint64_t> wdata_o[N_REQ];
    sc_in<bool>      gnt_i[N_REQ];
    sc_in<bool>      rvalid_i[N_REQ];
    sc_in<uint64_t>  rdata_i[N_REQ];
    sc_out<bool>     done_o;

    struct access_t { uint64_t cycle; uint64_t addr; bool we; uint64_t data; };
    std::vector<access_t> log_;

    static constexpr uint32_t BE_FULL =
        (WORD_BYTES >= 32) ? ~0u : ((1u << WORD_BYTES) - 1);

    std::string                out_path_;
    struct trace_entry_t { uint64_t addr; bool we; uint64_t data; };
    std::vector<trace_entry_t> trace_;
    std::size_t                next_row_;
    uint64_t                   cycle_;
    bool                       finished_;

    enum port_state_t { ISSUE, WAITRESP, DONE };
    port_state_t st_[N_REQ];
    uint64_t     addr_cur_[N_REQ];
    bool         we_cur_[N_REQ];
    uint64_t     wdata_cur_[N_REQ];

    static std::string trim(const std::string& s) {
        const std::size_t b = s.find_first_not_of(" \t\r\n");
        if (b == std::string::npos) return std::string();
        const std::size_t e = s.find_last_not_of(" \t\r\n");
        return s.substr(b, e - b + 1);
    }

    void load_trace(const std::string& path) {
        std::ifstream f(path.c_str());
        if (!f) SC_REPORT_FATAL(name(), ("cannot open trace: " + path).c_str());
        std::string line;
        while (std::getline(f, line)) {
            const std::size_t c1 = line.find(',');
            if (c1 == std::string::npos) continue;
            const std::size_t c2 = line.find(',', c1 + 1);
            const std::string a = trim(line.substr(0, c1));
            const std::string w = trim(c2 == std::string::npos
                                           ? line.substr(c1 + 1)
                                           : line.substr(c1 + 1, c2 - c1 - 1));
            const std::string d =
                (c2 == std::string::npos) ? std::string() : trim(line.substr(c2 + 1));
            if (a.empty() || a == "addr") continue;
            trace_entry_t e;
            e.addr = std::strtoull(a.c_str(), nullptr, 0);
            e.we   = (std::atoi(w.c_str()) != 0);
            e.data = d.empty() ? 0 : std::strtoull(d.c_str(), nullptr, 0);
            trace_.push_back(e);
        }
    }

    void reset_state() {
        for (int p = 0; p < N_REQ; ++p) {
            req_o[p].write(false);
            addr_o[p].write(0);
            we_o[p].write(false);
            be_o[p].write(0);
            wdata_o[p].write(0);
            st_[p]        = DONE;
            addr_cur_[p]  = 0;
            we_cur_[p]    = false;
            wdata_cur_[p] = 0;
        }
        next_row_ = 0;
        cycle_    = 0;
        finished_ = false;
        done_o.write(false);
        log_.clear();
    }

    void step() {
        if (!rst_ni.read()) {
            reset_state();
            return;
        }

        ++cycle_;

        for (int p = 0; p < N_REQ; ++p) {
            if (st_[p] == ISSUE) {
                if (gnt_i[p].read()) {
                    req_o[p].write(false);
                    st_[p] = WAITRESP;
                }
            } else if (st_[p] == WAITRESP) {
                if (rvalid_i[p].read()) {
                    const uint64_t data = we_cur_[p] ? wdata_cur_[p] : rdata_i[p].read();
                    access_t rec;
                    rec.cycle = cycle_;
                    rec.addr  = addr_cur_[p];
                    rec.we    = we_cur_[p];
                    rec.data  = data;
                    log_.push_back(rec);
                    st_[p] = DONE;
                }
            }
        }

        bool all_done = true;
        for (int p = 0; p < N_REQ; ++p)
            if (st_[p] != DONE) { all_done = false; break; }

        if (all_done && !finished_) {
            if (next_row_ >= trace_.size()) {
                finished_ = true;
                done_o.write(true);
            } else {
                for (int p = 0; p < N_REQ; ++p) {
                    if (next_row_ < trace_.size()) {
                        const trace_entry_t e = trace_[next_row_++];
                        addr_cur_[p]  = e.addr;
                        we_cur_[p]    = e.we;
                        wdata_cur_[p] = e.we ? e.data : 0;
                        addr_o[p].write(e.addr);
                        we_o[p].write(e.we);
                        be_o[p].write(BE_FULL);
                        wdata_o[p].write(wdata_cur_[p]);
                        req_o[p].write(true);
                        st_[p] = ISSUE;
                    } else {
                        req_o[p].write(false);
                        st_[p] = DONE;
                    }
                }
            }
        }
    }

    void end_of_simulation() override {
        if (out_path_.empty()) return;
        std::ofstream f(out_path_.c_str());
        if (!f) {
            SC_REPORT_WARNING(name(), ("cannot write log: " + out_path_).c_str());
            return;
        }
        f << "cycle,addr,we,data\n";
        for (const access_t& a : log_) {
            f << a.cycle << ",0x" << std::hex << std::setw(8) << std::setfill('0')
              << a.addr << "," << std::dec << (a.we ? 1 : 0) << ",0x" << std::hex
              << std::setw(8) << std::setfill('0') << a.data << std::dec << "\n";
        }
    }

    agu(sc_core::sc_module_name nm, const std::string& trace_path,
        const std::string& out_path = std::string())
        : sc_module(nm), out_path_(out_path) {
        load_trace(trace_path);

        for (int p = 0; p < N_REQ; ++p) st_[p] = DONE;
        next_row_ = 0;
        cycle_    = 0;
        finished_ = false;

        SC_METHOD(step);
        sensitive << clk_i.pos();
        dont_initialize();
    }
};

#endif
