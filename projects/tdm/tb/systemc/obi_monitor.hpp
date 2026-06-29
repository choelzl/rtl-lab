// -----------------------------------------------------------------------------
// Author: Cedric Hoelzl
//
// obi_monitor<N, BYTES> — edge-triggered OBI transaction logger.
//
// Connects to both sides of an N-port OBI bus and emits one CSV row per
// handshake event:
//   REQ  — req rises (0→1); logs addr, we, be, wdata
//   GNT  — gnt asserts while req is high; logs same fields
//   RSP  — rvalid rises (0→1); logs addr/we carried from the matching GNT
//
// In-flight queue per port tracks addr/we/wdata from GNT until RSP so the
// response row can identify which request completed.
//
// CSV header: cycle,component,port,event,addr,we,be,wdata,rdata
// A field is "-" when it is not meaningful for that event.
//
// The file is not opened when path is empty — the module is silently idle.
// -----------------------------------------------------------------------------

#ifndef OBI_MONITOR_HPP
#define OBI_MONITOR_HPP

#include <systemc.h>

#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#include "obi_data.hpp"

template <int N, int BYTES>
SC_MODULE(obi_monitor) {
    using data_t = obi_data<BYTES>;

    sc_in<bool>     clk_i;
    sc_in<bool>     rst_ni;
    sc_in<bool>     req_i[N];
    sc_in<uint64_t> addr_i[N];
    sc_in<bool>     we_i[N];
    sc_in<uint32_t> be_i[N];
    sc_in<data_t>   wdata_i[N];
    sc_in<bool>     gnt_i[N];
    sc_in<bool>     rvalid_i[N];
    sc_in<data_t>   rdata_i[N];

  private:
    struct inflight_t {
        uint64_t addr;
        bool     we;
        data_t   wdata;
    };

    std::string            component_;
    std::ofstream          out_;
    int64_t                cycle_;
    bool                   prev_req_[N];
    bool                   prev_gnt_[N];
    std::deque<inflight_t> inflight_[N];

    static std::string fmt_addr(uint64_t a) {
        std::ostringstream os;
        os << "0x" << std::hex << std::setw(10) << std::setfill('0') << a;
        return os.str();
    }

    static std::string fmt_be(uint32_t be) {
        std::ostringstream os;
        os << "0x" << std::hex << std::setw(8) << std::setfill('0') << be;
        return os.str();
    }

    static std::string fmt_data(const data_t &d) {
        constexpr int W      = BYTES * 8;
        constexpr int chunks = (W + 31) / 32;
        std::ostringstream os;
        os << "0x";
        for (int i = chunks - 1; i >= 0; --i) {
            const int hi = (i + 1) * 32 - 1 < W - 1 ? (i + 1) * 32 - 1 : W - 1;
            const int lo = i * 32;
            os << std::hex << std::setw(8) << std::setfill('0')
               << static_cast<uint32_t>(d.range(hi, lo).to_uint());
        }
        return os.str();
    }

    void emit(int p, const char *event, uint64_t addr, bool we, uint32_t be, const data_t &wdata,
              const data_t &rdata, bool has_rdata) {
        out_ << std::dec << cycle_ << "," << component_ << "," << p << "," << event << ","
             << fmt_addr(addr) << "," << (we ? "W" : "R") << "," << fmt_be(be) << ","
             << (we || !has_rdata ? fmt_data(wdata) : std::string("-")) << ","
             << (has_rdata ? fmt_data(rdata) : std::string("-")) << "\n";
    }

    void observe() {
        if (!rst_ni.read()) {
            for (int p = 0; p < N; ++p) {
                inflight_[p].clear();
                prev_req_[p] = false;
                prev_gnt_[p] = false;
            }
            cycle_ = 0;
            return;
        }

        ++cycle_;
        if (!out_.is_open())
            return;

        for (int p = 0; p < N; ++p) {
            const bool req    = req_i[p].read();
            const bool gnt    = gnt_i[p].read();
            const bool rvalid = rvalid_i[p].read();

            const bool req_rise  = req && !prev_req_[p];
            const bool back2back = req && prev_gnt_[p];  // new addr phase after back-to-back gnt
            const bool gnt_fire  = gnt && req;

            if (req_rise || back2back) {
                emit(p, "REQ", addr_i[p].read(), we_i[p].read(), be_i[p].read(),
                     wdata_i[p].read(), data_t(0), false);
            }

            if (gnt_fire) {
                inflight_[p].push_back(
                    {addr_i[p].read(), we_i[p].read(), wdata_i[p].read()});
                emit(p, "GNT", addr_i[p].read(), we_i[p].read(), be_i[p].read(),
                     wdata_i[p].read(), data_t(0), false);
            }

            // Level-triggered RSP: emit for every cycle rvalid is asserted while
            // there is an inflight entry.  Edge detection (rvalid 0→1) would miss
            // back-to-back responses where rvalid stays high across consecutive
            // drain cycles (common in TDM prefetch drain and crossbar pipelines).
            if (rvalid) {
                if (!inflight_[p].empty()) {
                    const inflight_t &inf = inflight_[p].front();
                    emit(p, "RSP", inf.addr, inf.we, 0, inf.wdata, rdata_i[p].read(), true);
                    inflight_[p].pop_front();
                } else {
                    emit(p, "RSP!", 0, false, 0, data_t(0), rdata_i[p].read(), true);
                }
            }

            prev_req_[p] = req;
            prev_gnt_[p] = gnt_fire;
        }
    }

  public:
    SC_HAS_PROCESS(obi_monitor);

    obi_monitor(sc_module_name nm, const std::string &component, const std::string &path)
        : sc_module(nm), component_(component), cycle_(0) {
        for (int p = 0; p < N; ++p) {
            prev_req_[p] = false;
            prev_gnt_[p] = false;
        }
        if (!path.empty()) {
            out_.open(path);
            if (out_)
                out_ << "cycle,component,port,event,addr,we,be,wdata,rdata\n";
            else
                SC_REPORT_WARNING(name(), ("cannot open OBI log: " + path).c_str());
        }
        SC_METHOD(observe);
        sensitive << clk_i.pos();
        dont_initialize();
    }
};

#endif // OBI_MONITOR_HPP
