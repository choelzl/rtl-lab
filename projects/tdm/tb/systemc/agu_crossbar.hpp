// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   Native SystemC Address Generation Unit (AGU) for the CROSSBAR design — a
//   non-synthesizable verification driver for the crossbar DUT (lives under
//   tb/systemc/). The TDM design has its own driver, agu_tdm.hpp. It is an
//   OBI manager with NUM_REQ request ports that replays a CSV memory trace
//   (addr,we,data) and drives the simplified single-channel OBI protocol (see
//   doc/specs/obi.md). The trace's first non-empty line is metadata (the shared
//   format carries the TDM mapping parameters there) and is skipped — the crossbar
//   does not use it:
//
//     request  (AGU -> crossbar) : req_o, addr_o, we_o, be_o, wdata_o   [NUM_REQ]
//     response (crossbar -> AGU) : gnt_i, rvalid_i, rdata_i             [NUM_REQ]
//
//   Operation (pipelined, group-synchronized):
//     - The trace is consumed in groups of NUM_REQ rows — one row per port. All
//       NUM_REQ requests of a group are presented at once (addr_o is the GLOBAL
//       byte address from the trace; the crossbar decodes bank/row).
//     - A port keeps its request asserted until granted. The group advances to
//       the next group as soon as ALL NUM_REQ ports are granted — it does NOT
//       wait for the responses. With no conflict every port is granted each
//       cycle, so a new group issues every cycle (throughput 1 group/cycle).
//       Under conflict the granted ports hold while the stalled ports retry,
//       and the group advances once all are granted.
//     - Pipelining stays strictly in order and shallow: the address phase of
//       the next group overlaps the response phase of the previous one, so at
//       most two transactions are in flight on a port at once (one being
//       responded, one being requested) — not real multi-outstanding. Each port
//       keeps a 1-deep in-flight record so it can log a response that arrives
//       after it has moved on to the next request.
//     - Writes (we==1) drive the trace's `data` column as wdata, all byte
//       enables set; reads (we==0) ignore wdata.
//     - Every completed access is recorded (cycle, addr, we, data) and dumped at
//       end of simulation to the output log (out_N.log) for inspection — writes
//       show the value written, reads the value returned.
//     - done_o is raised once the whole trace has been consumed (all groups
//       advanced and all responses collected).
//
//   Reset is active-low (rst_ni).
//
// Template parameters (set from PARAMS macros N_REQ, WORD_BYTES):
//   NUM_REQ    - request ports / group size (default 4)
//   BYTES_PER_WORD - bytes per word / OBI data beat (default 4)
// -----------------------------------------------------------------------------

#ifndef AGU_CROSSBAR_HPP
#define AGU_CROSSBAR_HPP

#include <systemc.h>

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

template <int NUM_REQ = 4, int BYTES_PER_WORD = 4> SC_MODULE(agu_crossbar) {
    sc_in<bool>      clk_i;
    sc_in<bool>      rst_ni;
    sc_out<bool>     req_o[NUM_REQ];
    sc_out<uint64_t> addr_o[NUM_REQ];
    sc_out<bool>     we_o[NUM_REQ];
    sc_out<uint32_t> be_o[NUM_REQ];
    sc_out<uint64_t> wdata_o[NUM_REQ];
    sc_in<bool>      gnt_i[NUM_REQ];
    sc_in<bool>      rvalid_i[NUM_REQ];
    sc_in<uint64_t>  rdata_i[NUM_REQ];
    sc_out<bool>     done_o;

    struct access_t {
        uint64_t cycle;
        uint64_t addr;
        bool     we;
        uint64_t data;
    };
    std::vector<access_t> log_;

    static constexpr uint32_t kBeFull = (BYTES_PER_WORD >= 32) ? ~0u : ((1u << BYTES_PER_WORD) - 1);

    std::string out_path_;
    struct trace_entry_t {
        uint64_t addr;
        bool     we;
        uint64_t data;
    };
    std::vector<trace_entry_t> trace_;
    std::size_t                n_groups_;
    std::size_t                group_;
    uint64_t                   cycle_;

    bool     granted_[NUM_REQ];
    bool     if_valid_[NUM_REQ];
    uint64_t if_addr_[NUM_REQ];
    bool     if_we_[NUM_REQ];
    uint64_t if_data_[NUM_REQ];

    bool has_row(std::size_t g, int p) const {
        return g * static_cast<std::size_t>(NUM_REQ) + p < trace_.size();
    }

    static std::string trim(const std::string &s) {
        const std::size_t b = s.find_first_not_of(" \t\r\n");
        if (b == std::string::npos)
            return std::string();
        const std::size_t e = s.find_last_not_of(" \t\r\n");
        return s.substr(b, e - b + 1);
    }

    void load_trace(const std::string &path) {
        std::ifstream f(path.c_str());
        if (!f) {
            SC_REPORT_INFO(name(), ("no stimuli (" + path + "), will be idle").c_str());
            return;
        }
        std::string line;
        while (std::getline(f, line))
            if (!trim(line).empty())
                break;
        while (std::getline(f, line)) {
            const std::size_t c1 = line.find(',');
            if (c1 == std::string::npos)
                continue;
            const std::size_t c2 = line.find(',', c1 + 1);
            const std::string a  = trim(line.substr(0, c1));
            const std::string w  = trim(c2 == std::string::npos ? line.substr(c1 + 1)
                                                                : line.substr(c1 + 1, c2 - c1 - 1));
            const std::string d =
                (c2 == std::string::npos) ? std::string() : trim(line.substr(c2 + 1));
            if (a.empty() || a == "addr")
                continue;
            trace_entry_t e;
            e.addr = std::strtoull(a.c_str(), nullptr, 0);
            e.we   = (std::atoi(w.c_str()) != 0);
            e.data = d.empty() ? 0 : std::strtoull(d.c_str(), nullptr, 0);
            trace_.push_back(e);
        }
    }

    void reset_state() {
        for (int p = 0; p < NUM_REQ; ++p) {
            req_o[p].write(false);
            addr_o[p].write(0);
            we_o[p].write(false);
            be_o[p].write(0);
            wdata_o[p].write(0);
            granted_[p]  = false;
            if_valid_[p] = false;
            if_addr_[p]  = 0;
            if_we_[p]    = false;
            if_data_[p]  = 0;
        }
        group_ = 0;
        cycle_ = 0;
        done_o.write(false);
        log_.clear();
    }

    void step() {
        if (!rst_ni.read()) {
            reset_state();
            return;
        }

        ++cycle_;

        for (int p = 0; p < NUM_REQ; ++p) {
            if (if_valid_[p] && rvalid_i[p].read()) {
                const uint64_t data = if_we_[p] ? if_data_[p] : rdata_i[p].read();
                access_t       rec;
                rec.cycle = cycle_;
                rec.addr  = if_addr_[p];
                rec.we    = if_we_[p];
                rec.data  = data;
                log_.push_back(rec);
                if_valid_[p] = false;
            }
        }

        for (int p = 0; p < NUM_REQ; ++p) {
            if (req_o[p].read() && gnt_i[p].read()) {
                const trace_entry_t e = trace_[group_ * NUM_REQ + p];
                granted_[p]           = true;
                if_valid_[p]          = true;
                if_addr_[p]           = e.addr;
                if_we_[p]             = e.we;
                if_data_[p]           = e.we ? e.data : 0;
            }
        }

        if (group_ < n_groups_) {
            bool group_granted = true;
            for (int p = 0; p < NUM_REQ; ++p)
                if (has_row(group_, p) && !granted_[p]) {
                    group_granted = false;
                    break;
                }
            if (group_granted) {
                ++group_;
                for (int p = 0; p < NUM_REQ; ++p)
                    granted_[p] = false;
            }
        }

        for (int p = 0; p < NUM_REQ; ++p) {
            if (group_ < n_groups_ && has_row(group_, p) && !granted_[p]) {
                const trace_entry_t e = trace_[group_ * NUM_REQ + p];
                addr_o[p].write(e.addr);
                we_o[p].write(e.we);
                be_o[p].write(kBeFull);
                wdata_o[p].write(e.we ? e.data : 0);
                req_o[p].write(true);
            } else {
                req_o[p].write(false);
            }
        }

        bool inflight = false;
        for (int p = 0; p < NUM_REQ; ++p)
            if (if_valid_[p]) {
                inflight = true;
                break;
            }
        done_o.write(group_ >= n_groups_ && !inflight);
    }

    void end_of_simulation() override {
        if (out_path_.empty())
            return;
        std::ofstream f(out_path_.c_str());
        if (!f) {
            SC_REPORT_WARNING(name(), ("cannot write log: " + out_path_).c_str());
            return;
        }
        f << "cycle,addr,we,data\n";
        for (const access_t &a : log_) {
            f << a.cycle << ",0x" << std::hex << std::setw(8) << std::setfill('0') << a.addr << ","
              << std::dec << (a.we ? 1 : 0) << ",0x" << std::hex << std::setw(8)
              << std::setfill('0') << a.data << std::dec << "\n";
        }
    }

    agu_crossbar(sc_core::sc_module_name nm, const std::string &trace_path,
                 const std::string &out_path = std::string())
        : sc_module(nm), out_path_(out_path) {
        load_trace(trace_path);

        n_groups_ = (trace_.size() + NUM_REQ - 1) / NUM_REQ;
        group_    = 0;
        cycle_    = 0;
        for (int p = 0; p < NUM_REQ; ++p) {
            granted_[p]  = false;
            if_valid_[p] = false;
        }

        SC_METHOD(step);
        sensitive << clk_i.pos();
        dont_initialize();
    }
};

#endif
