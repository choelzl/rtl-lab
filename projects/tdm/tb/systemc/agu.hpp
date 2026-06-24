// -----------------------------------------------------------------------------
// Unified native SystemC Address Generation Unit (AGU) testbench driver.
//
// The module exposes one black-box OBI-like manager interface and selects the
// access policy internally:
//   - agu_target::crossbar: trace entry [g*ports_used_+p] gives the address for
//     port p of group g; ports are driven in parallel.
//   - agu_target::tdm: same indexing; entry [g*ports_used_+p] gives the per-cell
//     fetch address (addr_o[p]) and the drain request for port p.  The testbench
//     connects addr_o[0..NUM_TDM-1] to the buffer's fetch_addr_i, and req_o[0..N_IO-1]
//     to the drain req.  Ports N_IO..NUM_TDM-1 are address-only: gnt_i/rvalid_i tied.
//
// Trace format (first non-empty line is the descriptor):
//   start_cycle,ports_used_groups,C,R,L,store_mode
//   Then one addr entry per line.  RAGU: addr only (implicit read).
//   WAGU: addr,data (implicit write).  Legacy: addr,we,data.
// -----------------------------------------------------------------------------

#ifndef AGU_HPP
#define AGU_HPP

#include <systemc.h>

#include <cstdint>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "obi_data.hpp"

enum class agu_target { crossbar, tdm };

template <typename T> static inline T agu_data_from_u64(uint64_t v) {
    return T(static_cast<unsigned long long>(v));
}

template <> inline uint64_t agu_data_from_u64<uint64_t>(uint64_t v) {
    return v;
}

template <typename T> static inline std::string agu_data_hex(const T &v) {
    std::ostringstream os;
    os << v;
    return os.str();
}

template <> inline std::string agu_data_hex<uint64_t>(const uint64_t &v) {
    std::ostringstream os;
    os << "0x" << std::hex << std::setw(8) << std::setfill('0') << v;
    return os.str();
}

template <int NUM_REQ = 4, typename DATA_T = uint64_t, int BYTES_PER_BEAT = 4, int N_PER_GROUP = 1>
SC_MODULE(agu) {
    using data_t = DATA_T;

    sc_in<bool>      clk_i;
    sc_in<bool>      rst_ni;
    sc_out<bool>     req_o[NUM_REQ];
    sc_out<uint64_t> addr_o[NUM_REQ];
    sc_out<bool>     we_o[NUM_REQ];
    sc_out<uint32_t> be_o[NUM_REQ];
    sc_out<data_t>   wdata_o[NUM_REQ];
    sc_in<bool>      gnt_i[NUM_REQ];
    sc_in<bool>      rvalid_i[NUM_REQ];
    sc_in<data_t>    rdata_i[NUM_REQ];
    sc_out<bool>     done_o;

    struct access_t {
        uint64_t cycle;
        uint64_t addr;
        bool     we;
        data_t   data;
    };
    std::vector<access_t> log_;

    static constexpr uint32_t kBeFull = (BYTES_PER_BEAT >= 32) ? ~0u : ((1u << BYTES_PER_BEAT) - 1);

    std::string out_path_;

    struct trace_entry_t {
        uint64_t addr;
        bool     we;
        data_t   data;
    };
    std::vector<trace_entry_t> trace_;
    std::size_t                n_groups_;
    std::size_t                group_;
    uint64_t                   cycle_;
    agu_target                 target_;

    uint64_t start_cycle_;
    int      ports_used_;

    uint64_t p_num_banks_;
    uint64_t p_bank_width_;
    uint64_t p_R_;
    uint64_t p_C_;
    uint64_t p_L_;
    uint64_t p_store_mode_;

    bool granted_[NUM_REQ];

    struct lane_rec_t {
        uint64_t addr;
        bool     we;
        data_t   data;
    };
    std::deque<lane_rec_t> lane_inflight_[NUM_REQ];

    struct group_rec_t {
        uint64_t addr[NUM_REQ];
        bool     valid[NUM_REQ];
        bool     we;
        data_t   data[NUM_REQ];
    };
    std::deque<group_rec_t> group_inflight_;

    // Both modes: each group occupies ports_used_ consecutive trace entries.
    // TDM: entry [g*ports_used_ + p] holds the fetch address for buffer cell p.
    // Crossbar: entry [g*ports_used_ + p] holds the per-port memory address.
    std::size_t trace_base(std::size_t g) const {
        return g * static_cast<std::size_t>(ports_used_);
    }

    bool has_row(std::size_t g, int p) const {
        return p < ports_used_ && trace_base(g) + static_cast<std::size_t>(p) < trace_.size();
    }

    static std::string trim(const std::string &s) {
        const std::size_t b = s.find_first_not_of(" \t\r\n");
        if (b == std::string::npos)
            return std::string();
        const std::size_t e = s.find_last_not_of(" \t\r\n");
        return s.substr(b, e - b + 1);
    }

    void parse_descriptor(const std::string &line) {
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
            SC_REPORT_FATAL(name(),
                            "descriptor must hold: start_cycle,ports_used,C,R,L,store_mode");
        start_cycle_  = v[0];
        ports_used_   = static_cast<int>(v[1]) * N_PER_GROUP;
        p_C_          = v[2];
        p_R_          = v[3];
        p_L_          = v[4];
        p_store_mode_ = v[5];
        if (ports_used_ < N_PER_GROUP || ports_used_ > NUM_REQ || ports_used_ % N_PER_GROUP != 0)
            SC_REPORT_FATAL(name(),
                            "ports_used (groups) out of range: must be 1..NUM_REQ/N_PER_GROUP");
    }

    void load_trace(const std::string &path) {
        std::ifstream f(path.c_str());
        if (!f) {
            SC_REPORT_INFO(name(), ("no stimuli (" + path + "), will be idle").c_str());
            return;
        }

        std::string line;
        bool        got_meta = false;
        while (std::getline(f, line)) {
            line = trim(line);
            if (line.empty())
                continue;
            parse_descriptor(line);
            got_meta = true;
            break;
        }
        if (!got_meta)
            SC_REPORT_FATAL(name(), "trace missing descriptor line");

        while (std::getline(f, line)) {
            line = trim(line);
            if (line.empty())
                continue;
            const std::size_t c1 = line.find(',');
            const std::string a  = trim(c1 == std::string::npos ? line : line.substr(0, c1));
            if (a.empty() || a == "addr")
                continue;
            trace_entry_t e;
            e.addr = std::strtoull(a.c_str(), nullptr, 0);
            if (c1 == std::string::npos) {
                // addr only → implicit read
                e.we   = false;
                e.data = data_t(0);
            } else {
                const std::size_t c2 = line.find(',', c1 + 1);
                const std::string f1 =
                    trim(c2 == std::string::npos ? line.substr(c1 + 1)
                                                 : line.substr(c1 + 1, c2 - c1 - 1));
                if (f1.empty()) {
                    // "addr," → trailing comma, treat as read
                    e.we   = false;
                    e.data = data_t(0);
                } else if (c2 == std::string::npos) {
                    // "addr,data" → implicit write
                    e.we   = true;
                    e.data = agu_data_from_u64<data_t>(std::strtoull(f1.c_str(), nullptr, 0));
                } else {
                    // "addr,we,data" → legacy explicit we
                    const std::string d = trim(line.substr(c2 + 1));
                    e.we                = (std::atoi(f1.c_str()) != 0);
                    e.data              = d.empty()
                                              ? data_t(0)
                                              : agu_data_from_u64<data_t>(std::strtoull(d.c_str(), nullptr, 0));
                }
            }
            trace_.push_back(e);
        }
    }

    uint64_t request_addr(std::size_t g, int p) const {
        return trace_[trace_base(g) + static_cast<std::size_t>(p)].addr;
    }

    bool request_we(std::size_t g, int p) const {
        return trace_[trace_base(g) + static_cast<std::size_t>(p)].we;
    }

    data_t request_data(std::size_t g, int p) const {
        const bool we = request_we(g, p);
        if (!we)
            return data_t(0);
        return trace_[trace_base(g) + static_cast<std::size_t>(p)].data;
    }

    void reset_state() {
        for (int p = 0; p < NUM_REQ; ++p) {
            req_o[p].write(false);
            addr_o[p].write(0);
            we_o[p].write(false);
            be_o[p].write(0);
            wdata_o[p].write(0);
            granted_[p] = false;
            lane_inflight_[p].clear();
        }
        group_inflight_.clear();
        group_ = 0;
        cycle_ = 0;
        done_o.write(false);
        log_.clear();
    }

    void collect_crossbar_responses() {
        for (int p = 0; p < NUM_REQ; ++p) {
            if (!lane_inflight_[p].empty() && rvalid_i[p].read()) {
                const lane_rec_t rec = lane_inflight_[p].front();
                lane_inflight_[p].pop_front();
                log_.push_back({cycle_, rec.addr, rec.we, rec.we ? rec.data : rdata_i[p].read()});
            }
        }
    }

    void collect_tdm_responses() {
        if (group_inflight_.empty())
            return;
        const group_rec_t &front    = group_inflight_.front();
        bool               complete = false;
        for (int p = 0; p < NUM_REQ; ++p)
            if (front.valid[p])
                complete = true;
        for (int p = 0; p < NUM_REQ; ++p)
            if (front.valid[p] && !rvalid_i[p].read())
                complete = false;
        if (!complete)
            return;

        const group_rec_t rec = front;
        group_inflight_.pop_front();
        for (int p = 0; p < NUM_REQ; ++p)
            if (rec.valid[p])
                log_.push_back(
                    {cycle_, rec.addr[p], rec.we, rec.we ? rec.data[p] : rdata_i[p].read()});
    }

    void record_grants() {
        if (group_ >= n_groups_ || cycle_ < start_cycle_)
            return;
        for (int p = 0; p < ports_used_; ++p) {
            if (has_row(group_, p) && req_o[p].read() && gnt_i[p].read()) {
                granted_[p] = true;
                if (target_ == agu_target::crossbar)
                    lane_inflight_[p].push_back(
                        {request_addr(group_, p), request_we(group_, p), request_data(group_, p)});
            }
        }
    }

    void advance_group_if_granted() {
        if (group_ >= n_groups_)
            return;

        bool all_granted = true;
        for (int p = 0; p < ports_used_; ++p) {
            if (has_row(group_, p) && !granted_[p]) {
                all_granted = false;
                break;
            }
        }
        if (!all_granted)
            return;

        if (target_ == agu_target::tdm) {
            group_rec_t rec;
            rec.we = false; // TDM fetch is always reads
            for (int p = 0; p < NUM_REQ; ++p) {
                rec.valid[p] = has_row(group_, p);
                rec.addr[p]  = rec.valid[p] ? request_addr(group_, p) : 0;
                rec.data[p]  = data_t(0);
            }
            group_inflight_.push_back(rec);
        }

        ++group_;
        for (int p = 0; p < NUM_REQ; ++p)
            granted_[p] = false;
    }

    void drive_requests() {
        for (int p = 0; p < NUM_REQ; ++p) {
            const bool active =
                cycle_ >= start_cycle_ && group_ < n_groups_ && has_row(group_, p) && !granted_[p];
            if (active) {
                addr_o[p].write(request_addr(group_, p));
                we_o[p].write(request_we(group_, p));
                be_o[p].write(kBeFull);
                wdata_o[p].write(request_data(group_, p));
                req_o[p].write(true);
            } else {
                addr_o[p].write(0);
                we_o[p].write(false);
                be_o[p].write(0);
                wdata_o[p].write(0);
                req_o[p].write(false);
            }
        }
    }

    bool has_inflight() const {
        if (target_ == agu_target::tdm)
            return !group_inflight_.empty();
        for (int p = 0; p < NUM_REQ; ++p)
            if (!lane_inflight_[p].empty())
                return true;
        return false;
    }

    void step() {
        if (!rst_ni.read()) {
            reset_state();
            return;
        }

        ++cycle_;
        if (target_ == agu_target::tdm)
            collect_tdm_responses();
        else
            collect_crossbar_responses();
        record_grants();
        advance_group_if_granted();
        drive_requests();
        done_o.write(group_ >= n_groups_ && !has_inflight());
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
              << std::dec << (a.we ? 1 : 0) << "," << agu_data_hex(a.data) << "\n";
        }
    }

    agu(sc_core::sc_module_name nm, const std::string &trace_path,
        const std::string &out_path = std::string(), agu_target target = agu_target::crossbar)
        : sc_module(nm), out_path_(out_path), n_groups_(0), group_(0), cycle_(0), target_(target),
          start_cycle_(0), ports_used_(NUM_REQ), p_num_banks_(32), p_bank_width_(BYTES_PER_BEAT),
          p_R_(4), p_C_(4), p_L_(8), p_store_mode_(0) {
        load_trace(trace_path);
        n_groups_ = ports_used_ > 0
                        ? (trace_.size() + ports_used_ - 1) / static_cast<std::size_t>(ports_used_)
                        : 0;
        for (int p = 0; p < NUM_REQ; ++p)
            granted_[p] = false;

        SC_METHOD(step);
        sensitive << clk_i.pos();
        dont_initialize();
    }
};

#endif
