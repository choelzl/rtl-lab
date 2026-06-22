// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   Native SystemC Address Generation Unit (AGU) for the TDM design — a
//   non-synthesizable verification driver (lives under tb/systemc/). It is an
//   x-OBI manager (see doc/specs/x_obi.md) that replays a CSV memory trace and
//   drives the TDM buffer's group interface:
//
//     request  (AGU -> buffer) : req_o[NUM_REQ], addr_o (base), we_o, be_o,
//                                wdata_o[NUM_REQ]
//     response (buffer -> AGU) : gnt_i[NUM_REQ], rvalid_i[NUM_REQ], rdata_i[NUM_REQ]
//     mapping  (AGU -> tdm mux): num_banks_o, bank_width_o, r_o, c_o, l_o,
//                                store_mode_o  (kernel-wide, constant)
//
//   The trace file's FIRST line carries the kernel-wide mapping parameters as a
//   CSV in the order  num_banks, bank_width, C, R, L, store_mode  (store_mode is
//   the integer index of the storage mode, 0..14). They decide the access pattern
//   for the whole kernel, so the AGU parses them once and drives them constantly.
//   The remaining lines are the access trace (an optional addr,we,data header,
//   then rows of addr,we,data).
//
//   Each cycle's NUM_REQ trace rows form one GROUP, issued as a single x-OBI
//   sub-request: base = row 0's address, we/be uniform (row 0's we), each word its
//   own wdata. Word w of the group is the logical address base + w*BYTES_PER_WORD
//   (the per-row addr of rows 1..NUM_REQ-1 is not used); the TDM mapping decides
//   the actual bank/row from that logical address and the parameters above.
//
//   Handshake — **grant-based, pipelined**: a port keeps req asserted until
//   granted; the moment the whole group is granted the AGU records it in an
//   in-order in-flight queue and issues the next group the following cycle. So
//   with no conflict it keeps req active and starts a new group every cycle, and
//   the previous group's response is collected while the next is requested
//   (responses arrive in order; the buffer returns a full NUM_REQ chunk at once).
//
//   Every completed access is logged (cycle, logical addr, we, data) and dumped at
//   end of simulation to out_N.log. done_o rises once the whole trace has been
//   consumed and the in-flight queue has drained. Reset is active-low.
//
//   ASSUMPTIONS (v1): each AGU's groups form a consecutive stream (so the buffer's
//   prefetch of the next group is valid) and we is uniform within a group.
//
// Template parameters (set from PARAMS macros N_REQ, WORD_BYTES):
//   NUM_REQ        - words per group (default 4)
//   BYTES_PER_WORD - bytes per word / address step (default 4)
// -----------------------------------------------------------------------------

#ifndef AGU_TDM_HPP
#define AGU_TDM_HPP

#include <systemc.h>

#include <cstdint>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

template <int NUM_REQ = 4, int BYTES_PER_WORD = 4> SC_MODULE(agu_tdm) {
    sc_in<bool>      clk_i;
    sc_in<bool>      rst_ni;
    sc_out<bool>     req_o[NUM_REQ];
    sc_out<uint64_t> addr_o;
    sc_out<bool>     we_o;
    sc_out<uint32_t> be_o;
    sc_out<uint64_t> wdata_o[NUM_REQ];
    sc_in<bool>      gnt_i[NUM_REQ];
    sc_in<bool>      rvalid_i[NUM_REQ];
    sc_in<uint64_t>  rdata_i[NUM_REQ];
    sc_out<uint64_t> num_banks_o;
    sc_out<uint64_t> bank_width_o;
    sc_out<uint64_t> r_o;
    sc_out<uint64_t> c_o;
    sc_out<uint64_t> l_o;
    sc_out<uint64_t> store_mode_o;
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

    uint64_t p_num_banks_;
    uint64_t p_bank_width_;
    uint64_t p_R_;
    uint64_t p_C_;
    uint64_t p_L_;
    uint64_t p_store_mode_;

    bool granted_[NUM_REQ];

    struct grp_rec {
        uint64_t addr[NUM_REQ];
        bool     we;
        uint64_t data[NUM_REQ];
    };
    std::deque<grp_rec> inflight_;

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

    void parse_params(const std::string &line) {
        std::vector<uint64_t> v;
        std::size_t           pos = 0;
        while (pos <= line.size()) {
            const std::size_t c = line.find(',', pos);
            const std::string tok =
                trim(c == std::string::npos ? line.substr(pos) : line.substr(pos, c - pos));
            if (!tok.empty())
                v.push_back(std::strtoull(tok.c_str(), nullptr, 0));
            if (c == std::string::npos)
                break;
            pos = c + 1;
        }
        if (v.size() < 6)
            SC_REPORT_FATAL(name(), "parameter line must hold 6 values: "
                                    "num_banks,bank_width,C,R,L,store_mode");
        p_num_banks_  = v[0];
        p_bank_width_ = v[1];
        p_C_          = v[2];
        p_R_          = v[3];
        p_L_          = v[4];
        p_store_mode_ = v[5];
    }

    void load_trace(const std::string &path) {
        std::ifstream f(path.c_str());
        if (!f)
            SC_REPORT_FATAL(name(), ("cannot open trace: " + path).c_str());
        std::string line;
        bool        got_params = false;
        while (std::getline(f, line)) {
            if (trim(line).empty())
                continue;
            parse_params(trim(line));
            got_params = true;
            break;
        }
        if (!got_params)
            SC_REPORT_FATAL(name(), "trace missing parameter line");
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

    void drive_params() {
        num_banks_o.write(p_num_banks_);
        bank_width_o.write(p_bank_width_);
        r_o.write(p_R_);
        c_o.write(p_C_);
        l_o.write(p_L_);
        store_mode_o.write(p_store_mode_);
    }

    void reset_state() {
        for (int p = 0; p < NUM_REQ; ++p) {
            req_o[p].write(false);
            wdata_o[p].write(0);
            granted_[p] = false;
        }
        addr_o.write(0);
        we_o.write(false);
        be_o.write(0);
        drive_params();
        group_ = 0;
        cycle_ = 0;
        done_o.write(false);
        log_.clear();
        inflight_.clear();
    }

    void step() {
        drive_params();
        if (!rst_ni.read()) {
            reset_state();
            return;
        }

        ++cycle_;

        bool resp = false;
        for (int p = 0; p < NUM_REQ; ++p)
            if (rvalid_i[p].read()) {
                resp = true;
                break;
            }
        if (resp && !inflight_.empty()) {
            const grp_rec g = inflight_.front();
            inflight_.pop_front();
            for (int p = 0; p < NUM_REQ; ++p) {
                const uint64_t data = g.we ? g.data[p] : rdata_i[p].read();
                log_.push_back({cycle_, g.addr[p], g.we, data});
            }
        }

        for (int p = 0; p < NUM_REQ; ++p)
            if (req_o[p].read() && gnt_i[p].read())
                granted_[p] = true;

        if (group_ < n_groups_) {
            bool all_granted = true;
            for (int p = 0; p < NUM_REQ; ++p)
                if (has_row(group_, p) && !granted_[p]) {
                    all_granted = false;
                    break;
                }
            if (all_granted) {
                const trace_entry_t e0 = trace_[group_ * NUM_REQ];
                grp_rec             g;
                g.we = e0.we;
                for (int p = 0; p < NUM_REQ; ++p) {
                    g.addr[p] = e0.addr + static_cast<uint64_t>(p) * BYTES_PER_WORD;
                    g.data[p] = e0.we ? trace_[group_ * NUM_REQ + p].data : 0;
                }
                inflight_.push_back(g);
                ++group_;
                for (int p = 0; p < NUM_REQ; ++p)
                    granted_[p] = false;
            }
        }

        if (group_ < n_groups_) {
            const trace_entry_t e0 = trace_[group_ * NUM_REQ];
            addr_o.write(e0.addr);
            we_o.write(e0.we);
            be_o.write(kBeFull);
            for (int p = 0; p < NUM_REQ; ++p) {
                if (has_row(group_, p) && !granted_[p]) {
                    wdata_o[p].write(e0.we ? trace_[group_ * NUM_REQ + p].data : 0);
                    req_o[p].write(true);
                } else {
                    req_o[p].write(false);
                    wdata_o[p].write(0);
                }
            }
        } else {
            addr_o.write(0);
            we_o.write(false);
            be_o.write(0);
            for (int p = 0; p < NUM_REQ; ++p) {
                req_o[p].write(false);
                wdata_o[p].write(0);
            }
        }

        done_o.write(group_ >= n_groups_ && inflight_.empty());
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

    agu_tdm(sc_core::sc_module_name nm, const std::string &trace_path,
            const std::string &out_path = std::string())
        : sc_module(nm), out_path_(out_path), p_num_banks_(32), p_bank_width_(4), p_R_(4), p_C_(4),
          p_L_(8), p_store_mode_(0) {
        load_trace(trace_path);

        n_groups_ = (trace_.size() + NUM_REQ - 1) / NUM_REQ;
        group_    = 0;
        cycle_    = 0;
        for (int p = 0; p < NUM_REQ; ++p)
            granted_[p] = false;

        SC_METHOD(step);
        sensitive << clk_i.pos();
        dont_initialize();
    }
};

#endif
