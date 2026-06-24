// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Description:
//   Native SystemC design top (DUT) for the crossbar design, mirroring the
//   three-level SV architecture in top_crossbar.sv.
//
//   Pipeline:
//     port -> addr_hash -> L1 -> L2 -> L3 -> bank
//
//   Only L3 and the banks are shared between read and write traffic. The read
//   and write paths have separate external ports, L1 crossbars, and L2 crossbars.
//
//   Terminology: this module works in read/write ports. Each port contains
//   NUM_REQ independent OBI buses. Upstream logic may drive one or more ports;
//   that mapping is handled above this module.
//
//   L1: one NUM_REQ x NUM_REQ crossbar per read/write port. Each L1 routes the
//   port's NUM_REQ OBI buses to the NUM_REQ L2 groups using
//   addr[ROUTE_LSB +: LOG_REQ].
//
//   L2: one crossbar per request lane. Read L2 instances are
//   NUM_RPORT x (NUM_BANK/NUM_REQ); write L2 instances are
//   NUM_WPORT x (NUM_BANK/NUM_REQ). Here NUM_RPORT/NUM_WPORT are
//   read/write port counts. They route across bank groups using
//   addr[ROUTE_LSB+LOG_REQ +: LOG_BANK_GRP].
//
//   Level 3 (NUM_BANK instances, 2x2):
//     One crossbar per logical bank. Each merges the read and write paths onto
//     even/odd physical banks using addr[ROUTE_LSB+LOG_REQ+LOG_BANK_GRP] (bit 9
//     for default parameters).
//
//   Banks: NUM_BANK*2 physical banks, each NUM_ROW/2 rows. The ROUTE_BITS-wide
//   routing field starting at ROUTE_LSB is stripped before the address reaches
//   the bank.
//
//   addr_hash scrambles the L2-select bits (addr[8:6]) by adding the
//   overlapping addr[11:9] field, matching top_crossbar.sv.
//
//   Address layout (defaults: BYTES_PER_ROW=16, NUM_REQ=4, NUM_BANK_GRP=8):
//     addr[ROUTE_LSB-1:0]                              byte offset in row
//     addr[ROUTE_LSB         +: LOG_REQ     ]          L1 select
//     addr[ROUTE_LSB+LOG_REQ +: LOG_BANK_GRP]          L2 select
//     addr[ROUTE_LSB+LOG_REQ+LOG_BANK_GRP]             L3 even/odd select
//     addr[31:ROUTE_LSB+ROUTE_BITS]                    bank-local row
//
// Template parameters:
//   NUM_RPORT      - number of read ports
//   NUM_WPORT      - number of write ports
//   NUM_REQ       - OBI buses per read/write port
//   NUM_BANK, NUM_ROW, BYTES_PER_WORD, WORDS_PER_ROW
// -----------------------------------------------------------------------------

#ifndef TOP_CROSSBAR_HPP
#define TOP_CROSSBAR_HPP

#include <cstdint>
#include <systemc.h>

#include "bank.hpp"
#include "crossbar.hpp"

namespace tc_detail {
constexpr bool is_pow2(int n) {
    return n > 0 && (n & (n - 1)) == 0;
}

constexpr int log2_pow2(int n) {
    return n <= 1 ? 0 : 1 + log2_pow2(n >> 1);
}
} // namespace tc_detail

template <int NUM_RPORT = 2, int NUM_WPORT = 2, int NUM_REQ = 4, int NUM_BANK = 8,
          int NUM_ROW = 1024, int BYTES_PER_WORD = 4, int WORDS_PER_ROW = 4>
SC_MODULE(top_crossbar) {
    static_assert(tc_detail::is_pow2(NUM_REQ), "NUM_REQ must be a power of two");
    static_assert(tc_detail::is_pow2(NUM_BANK), "NUM_BANK must be a power of two");
    static_assert(NUM_BANK % NUM_REQ == 0, "NUM_BANK must be divisible by NUM_REQ");
    static_assert(NUM_ROW % 2 == 0,
                  "NUM_ROW must be even because L3 splits banks into even/odd halves");
    static_assert(BYTES_PER_WORD > 0 && WORDS_PER_ROW > 0, "word and row sizes must be positive");
    static_assert(tc_detail::is_pow2(BYTES_PER_WORD * WORDS_PER_ROW),
                  "BYTES_PER_ROW must be a power of two");

    // -----------------------------------------------------------------------
    // Derived constants
    // -----------------------------------------------------------------------
    static constexpr int NUM_RPORT_PORTS = NUM_RPORT * NUM_REQ;
    static constexpr int NUM_WPORT_PORTS = NUM_WPORT * NUM_REQ;
    static constexpr int NUM_BANK_GRP    = NUM_BANK / NUM_REQ;
    static constexpr int BYTES_PER_ROW   = WORDS_PER_ROW * BYTES_PER_WORD;
    using data_t                         = obi_data<BYTES_PER_ROW>;
    static constexpr int ROUTE_LSB       = tc_detail::log2_pow2(BYTES_PER_ROW);
    static constexpr int LOG_REQ         = tc_detail::log2_pow2(NUM_REQ);
    static constexpr int LOG_BANK_GRP    = tc_detail::log2_pow2(NUM_BANK_GRP);
    static constexpr int ROUTE_BITS      = LOG_REQ + LOG_BANK_GRP + 1;
    static constexpr int L2_SEL          = ROUTE_LSB + LOG_REQ;
    static constexpr int L3_SEL          = ROUTE_LSB + LOG_REQ + LOG_BANK_GRP;

    static constexpr int NUM_L1_L2_RD   = NUM_RPORT_PORTS;
    static constexpr int NUM_L1_L2_WR   = NUM_WPORT_PORTS;
    static constexpr int NUM_L2_L3      = NUM_BANK;
    static constexpr int NUM_PHYS_BANKS = NUM_BANK * 2;

    // -----------------------------------------------------------------------
    // External ports
    // -----------------------------------------------------------------------
    sc_in<bool> clk_i;
    sc_in<bool> rst_ni;

    sc_in<bool>     rport_req_i[NUM_RPORT_PORTS];
    sc_in<uint64_t> rport_addr_i[NUM_RPORT_PORTS];
    sc_in<bool>     rport_we_i[NUM_RPORT_PORTS];
    sc_in<uint32_t> rport_be_i[NUM_RPORT_PORTS];
    sc_in<data_t>   rport_wdata_i[NUM_RPORT_PORTS];
    sc_out<bool>    rport_gnt_o[NUM_RPORT_PORTS];
    sc_out<bool>    rport_rvalid_o[NUM_RPORT_PORTS];
    sc_out<data_t>  rport_rdata_o[NUM_RPORT_PORTS];

    sc_in<bool>     wport_req_i[NUM_WPORT_PORTS];
    sc_in<uint64_t> wport_addr_i[NUM_WPORT_PORTS];
    sc_in<bool>     wport_we_i[NUM_WPORT_PORTS];
    sc_in<uint32_t> wport_be_i[NUM_WPORT_PORTS];
    sc_in<data_t>   wport_wdata_i[NUM_WPORT_PORTS];
    sc_out<bool>    wport_gnt_o[NUM_WPORT_PORTS];
    sc_out<bool>    wport_rvalid_o[NUM_WPORT_PORTS];
    sc_out<data_t>  wport_rdata_o[NUM_WPORT_PORTS];

    // -----------------------------------------------------------------------
    // Hashed addresses and bank-local addresses
    // -----------------------------------------------------------------------
    sc_signal<uint64_t> rport_haddr[NUM_RPORT_PORTS];
    sc_signal<uint64_t> wport_haddr[NUM_WPORT_PORTS];
    sc_signal<uint64_t> bank_addr[NUM_PHYS_BANKS];

    // -----------------------------------------------------------------------
    // Inter-level wires: L1 -> L2
    //   Index [j*NUM_REQ+k]: L1 instance j, slave port k
    //                        -> L2 instance k, master port j
    // -----------------------------------------------------------------------
    sc_signal<bool>     l1_l2_rd_req[NUM_L1_L2_RD];
    sc_signal<uint64_t> l1_l2_rd_addr[NUM_L1_L2_RD];
    sc_signal<bool>     l1_l2_rd_we[NUM_L1_L2_RD];
    sc_signal<uint32_t> l1_l2_rd_be[NUM_L1_L2_RD];
    sc_signal<data_t>   l1_l2_rd_wdata[NUM_L1_L2_RD];
    sc_signal<bool>     l1_l2_rd_gnt[NUM_L1_L2_RD];
    sc_signal<bool>     l1_l2_rd_rvalid[NUM_L1_L2_RD];
    sc_signal<data_t>   l1_l2_rd_rdata[NUM_L1_L2_RD];

    sc_signal<bool>     l1_l2_wr_req[NUM_L1_L2_WR];
    sc_signal<uint64_t> l1_l2_wr_addr[NUM_L1_L2_WR];
    sc_signal<bool>     l1_l2_wr_we[NUM_L1_L2_WR];
    sc_signal<uint32_t> l1_l2_wr_be[NUM_L1_L2_WR];
    sc_signal<data_t>   l1_l2_wr_wdata[NUM_L1_L2_WR];
    sc_signal<bool>     l1_l2_wr_gnt[NUM_L1_L2_WR];
    sc_signal<bool>     l1_l2_wr_rvalid[NUM_L1_L2_WR];
    sc_signal<data_t>   l1_l2_wr_rdata[NUM_L1_L2_WR];

    // -----------------------------------------------------------------------
    // Inter-level wires: L2 -> L3
    //   Index [k*NUM_BANK_GRP+g]: L2 instance k, slave g -> L3 logical bank b
    // -----------------------------------------------------------------------
    sc_signal<bool>     l2_l3_rd_req[NUM_L2_L3];
    sc_signal<uint64_t> l2_l3_rd_addr[NUM_L2_L3];
    sc_signal<bool>     l2_l3_rd_we[NUM_L2_L3];
    sc_signal<uint32_t> l2_l3_rd_be[NUM_L2_L3];
    sc_signal<data_t>   l2_l3_rd_wdata[NUM_L2_L3];
    sc_signal<bool>     l2_l3_rd_gnt[NUM_L2_L3];
    sc_signal<bool>     l2_l3_rd_rvalid[NUM_L2_L3];
    sc_signal<data_t>   l2_l3_rd_rdata[NUM_L2_L3];

    sc_signal<bool>     l2_l3_wr_req[NUM_L2_L3];
    sc_signal<uint64_t> l2_l3_wr_addr[NUM_L2_L3];
    sc_signal<bool>     l2_l3_wr_we[NUM_L2_L3];
    sc_signal<uint32_t> l2_l3_wr_be[NUM_L2_L3];
    sc_signal<data_t>   l2_l3_wr_wdata[NUM_L2_L3];
    sc_signal<bool>     l2_l3_wr_gnt[NUM_L2_L3];
    sc_signal<bool>     l2_l3_wr_rvalid[NUM_L2_L3];
    sc_signal<data_t>   l2_l3_wr_rdata[NUM_L2_L3];

    // -----------------------------------------------------------------------
    // Inter-level wires: L3 -> physical banks
    //   Index [b*2+i]: L3 instance b, slave i (0=even, 1=odd)
    // -----------------------------------------------------------------------
    sc_signal<bool>     l3_bank_req[NUM_PHYS_BANKS];
    sc_signal<uint64_t> l3_bank_addr[NUM_PHYS_BANKS];
    sc_signal<bool>     l3_bank_we[NUM_PHYS_BANKS];
    sc_signal<uint32_t> l3_bank_be[NUM_PHYS_BANKS];
    sc_signal<data_t>   l3_bank_wdata[NUM_PHYS_BANKS];
    sc_signal<bool>     l3_bank_gnt[NUM_PHYS_BANKS];
    sc_signal<bool>     l3_bank_rvalid[NUM_PHYS_BANKS];
    sc_signal<data_t>   l3_bank_rdata[NUM_PHYS_BANKS];

    // -----------------------------------------------------------------------
    // Submodules
    // -----------------------------------------------------------------------
    sc_vector<crossbar<NUM_REQ, NUM_REQ, BYTES_PER_ROW, ROUTE_LSB, LOG_REQ>>          l1_rd_;
    sc_vector<crossbar<NUM_REQ, NUM_REQ, BYTES_PER_ROW, ROUTE_LSB, LOG_REQ>>          l1_wr_;
    sc_vector<crossbar<NUM_RPORT, NUM_BANK_GRP, BYTES_PER_ROW, L2_SEL, LOG_BANK_GRP>> l2_rd_;
    sc_vector<crossbar<NUM_WPORT, NUM_BANK_GRP, BYTES_PER_ROW, L2_SEL, LOG_BANK_GRP>> l2_wr_;
    sc_vector<crossbar<2, 2, BYTES_PER_ROW, L3_SEL, 1>>                               l3_;
    sc_vector<bank<NUM_ROW / 2, BYTES_PER_ROW>>                                       banks_;

    static uint64_t addr_hash(uint64_t a) {
        const uint64_t hi  = (a >> 9) & 0x7;
        const uint64_t mid = (a >> 6) & 0x7;
        const uint64_t sum = (hi + mid) & 0x7;
        return (a & ~(static_cast<uint64_t>(0x7) << 6)) | (sum << 6);
    }

    static uint64_t local_addr(uint64_t a) {
        const uint64_t below = a & ((1ULL << ROUTE_LSB) - 1);
        const uint64_t above = a >> (ROUTE_LSB + ROUTE_BITS);
        return (above << ROUTE_LSB) | below;
    }

    void hash_rd_addr() {
        for (int m = 0; m < NUM_RPORT_PORTS; ++m)
            rport_haddr[m].write(addr_hash(rport_addr_i[m].read()));
    }

    void hash_wr_addr() {
        for (int m = 0; m < NUM_WPORT_PORTS; ++m)
            wport_haddr[m].write(addr_hash(wport_addr_i[m].read()));
    }

    void compute_bank_addr() {
        for (int i = 0; i < NUM_PHYS_BANKS; ++i)
            bank_addr[i].write(local_addr(l3_bank_addr[i].read()));
    }

    void bind_l1_read() {
        for (int j = 0; j < NUM_RPORT; ++j) {
            l1_rd_[j].clk_i(clk_i);
            l1_rd_[j].rst_ni(rst_ni);
            for (int m = 0; m < NUM_REQ; ++m) {
                const int ext = j * NUM_REQ + m;
                l1_rd_[j].m_req_i[m](rport_req_i[ext]);
                l1_rd_[j].m_addr_i[m](rport_haddr[ext]);
                l1_rd_[j].m_we_i[m](rport_we_i[ext]);
                l1_rd_[j].m_be_i[m](rport_be_i[ext]);
                l1_rd_[j].m_wdata_i[m](rport_wdata_i[ext]);
                l1_rd_[j].m_gnt_o[m](rport_gnt_o[ext]);
                l1_rd_[j].m_rvalid_o[m](rport_rvalid_o[ext]);
                l1_rd_[j].m_rdata_o[m](rport_rdata_o[ext]);

                l1_rd_[j].b_req_o[m](l1_l2_rd_req[ext]);
                l1_rd_[j].b_addr_o[m](l1_l2_rd_addr[ext]);
                l1_rd_[j].b_we_o[m](l1_l2_rd_we[ext]);
                l1_rd_[j].b_be_o[m](l1_l2_rd_be[ext]);
                l1_rd_[j].b_wdata_o[m](l1_l2_rd_wdata[ext]);
                l1_rd_[j].b_gnt_i[m](l1_l2_rd_gnt[ext]);
                l1_rd_[j].b_rvalid_i[m](l1_l2_rd_rvalid[ext]);
                l1_rd_[j].b_rdata_i[m](l1_l2_rd_rdata[ext]);
            }
        }
    }

    void bind_l1_write() {
        for (int j = 0; j < NUM_WPORT; ++j) {
            l1_wr_[j].clk_i(clk_i);
            l1_wr_[j].rst_ni(rst_ni);
            for (int m = 0; m < NUM_REQ; ++m) {
                const int ext = j * NUM_REQ + m;
                l1_wr_[j].m_req_i[m](wport_req_i[ext]);
                l1_wr_[j].m_addr_i[m](wport_haddr[ext]);
                l1_wr_[j].m_we_i[m](wport_we_i[ext]);
                l1_wr_[j].m_be_i[m](wport_be_i[ext]);
                l1_wr_[j].m_wdata_i[m](wport_wdata_i[ext]);
                l1_wr_[j].m_gnt_o[m](wport_gnt_o[ext]);
                l1_wr_[j].m_rvalid_o[m](wport_rvalid_o[ext]);
                l1_wr_[j].m_rdata_o[m](wport_rdata_o[ext]);

                l1_wr_[j].b_req_o[m](l1_l2_wr_req[ext]);
                l1_wr_[j].b_addr_o[m](l1_l2_wr_addr[ext]);
                l1_wr_[j].b_we_o[m](l1_l2_wr_we[ext]);
                l1_wr_[j].b_be_o[m](l1_l2_wr_be[ext]);
                l1_wr_[j].b_wdata_o[m](l1_l2_wr_wdata[ext]);
                l1_wr_[j].b_gnt_i[m](l1_l2_wr_gnt[ext]);
                l1_wr_[j].b_rvalid_i[m](l1_l2_wr_rvalid[ext]);
                l1_wr_[j].b_rdata_i[m](l1_l2_wr_rdata[ext]);
            }
        }
    }

    void bind_l2_read() {
        for (int k = 0; k < NUM_REQ; ++k) {
            l2_rd_[k].clk_i(clk_i);
            l2_rd_[k].rst_ni(rst_ni);
            for (int j = 0; j < NUM_RPORT; ++j) {
                const int sig = j * NUM_REQ + k;
                l2_rd_[k].m_req_i[j](l1_l2_rd_req[sig]);
                l2_rd_[k].m_addr_i[j](l1_l2_rd_addr[sig]);
                l2_rd_[k].m_we_i[j](l1_l2_rd_we[sig]);
                l2_rd_[k].m_be_i[j](l1_l2_rd_be[sig]);
                l2_rd_[k].m_wdata_i[j](l1_l2_rd_wdata[sig]);
                l2_rd_[k].m_gnt_o[j](l1_l2_rd_gnt[sig]);
                l2_rd_[k].m_rvalid_o[j](l1_l2_rd_rvalid[sig]);
                l2_rd_[k].m_rdata_o[j](l1_l2_rd_rdata[sig]);
            }
            for (int g = 0; g < NUM_BANK_GRP; ++g) {
                const int b = k * NUM_BANK_GRP + g;
                l2_rd_[k].b_req_o[g](l2_l3_rd_req[b]);
                l2_rd_[k].b_addr_o[g](l2_l3_rd_addr[b]);
                l2_rd_[k].b_we_o[g](l2_l3_rd_we[b]);
                l2_rd_[k].b_be_o[g](l2_l3_rd_be[b]);
                l2_rd_[k].b_wdata_o[g](l2_l3_rd_wdata[b]);
                l2_rd_[k].b_gnt_i[g](l2_l3_rd_gnt[b]);
                l2_rd_[k].b_rvalid_i[g](l2_l3_rd_rvalid[b]);
                l2_rd_[k].b_rdata_i[g](l2_l3_rd_rdata[b]);
            }
        }
    }

    void bind_l2_write() {
        for (int k = 0; k < NUM_REQ; ++k) {
            l2_wr_[k].clk_i(clk_i);
            l2_wr_[k].rst_ni(rst_ni);
            for (int j = 0; j < NUM_WPORT; ++j) {
                const int sig = j * NUM_REQ + k;
                l2_wr_[k].m_req_i[j](l1_l2_wr_req[sig]);
                l2_wr_[k].m_addr_i[j](l1_l2_wr_addr[sig]);
                l2_wr_[k].m_we_i[j](l1_l2_wr_we[sig]);
                l2_wr_[k].m_be_i[j](l1_l2_wr_be[sig]);
                l2_wr_[k].m_wdata_i[j](l1_l2_wr_wdata[sig]);
                l2_wr_[k].m_gnt_o[j](l1_l2_wr_gnt[sig]);
                l2_wr_[k].m_rvalid_o[j](l1_l2_wr_rvalid[sig]);
                l2_wr_[k].m_rdata_o[j](l1_l2_wr_rdata[sig]);
            }
            for (int g = 0; g < NUM_BANK_GRP; ++g) {
                const int b = k * NUM_BANK_GRP + g;
                l2_wr_[k].b_req_o[g](l2_l3_wr_req[b]);
                l2_wr_[k].b_addr_o[g](l2_l3_wr_addr[b]);
                l2_wr_[k].b_we_o[g](l2_l3_wr_we[b]);
                l2_wr_[k].b_be_o[g](l2_l3_wr_be[b]);
                l2_wr_[k].b_wdata_o[g](l2_l3_wr_wdata[b]);
                l2_wr_[k].b_gnt_i[g](l2_l3_wr_gnt[b]);
                l2_wr_[k].b_rvalid_i[g](l2_l3_wr_rvalid[b]);
                l2_wr_[k].b_rdata_i[g](l2_l3_wr_rdata[b]);
            }
        }
    }

    void bind_l3_and_banks() {
        for (int b = 0; b < NUM_BANK; ++b) {
            l3_[b].clk_i(clk_i);
            l3_[b].rst_ni(rst_ni);

            l3_[b].m_req_i[0](l2_l3_rd_req[b]);
            l3_[b].m_addr_i[0](l2_l3_rd_addr[b]);
            l3_[b].m_we_i[0](l2_l3_rd_we[b]);
            l3_[b].m_be_i[0](l2_l3_rd_be[b]);
            l3_[b].m_wdata_i[0](l2_l3_rd_wdata[b]);
            l3_[b].m_gnt_o[0](l2_l3_rd_gnt[b]);
            l3_[b].m_rvalid_o[0](l2_l3_rd_rvalid[b]);
            l3_[b].m_rdata_o[0](l2_l3_rd_rdata[b]);

            l3_[b].m_req_i[1](l2_l3_wr_req[b]);
            l3_[b].m_addr_i[1](l2_l3_wr_addr[b]);
            l3_[b].m_we_i[1](l2_l3_wr_we[b]);
            l3_[b].m_be_i[1](l2_l3_wr_be[b]);
            l3_[b].m_wdata_i[1](l2_l3_wr_wdata[b]);
            l3_[b].m_gnt_o[1](l2_l3_wr_gnt[b]);
            l3_[b].m_rvalid_o[1](l2_l3_wr_rvalid[b]);
            l3_[b].m_rdata_o[1](l2_l3_wr_rdata[b]);

            for (int i = 0; i < 2; ++i) {
                const int ph = b * 2 + i;
                l3_[b].b_req_o[i](l3_bank_req[ph]);
                l3_[b].b_addr_o[i](l3_bank_addr[ph]);
                l3_[b].b_we_o[i](l3_bank_we[ph]);
                l3_[b].b_be_o[i](l3_bank_be[ph]);
                l3_[b].b_wdata_o[i](l3_bank_wdata[ph]);
                l3_[b].b_gnt_i[i](l3_bank_gnt[ph]);
                l3_[b].b_rvalid_i[i](l3_bank_rvalid[ph]);
                l3_[b].b_rdata_i[i](l3_bank_rdata[ph]);
            }
        }

        for (int i = 0; i < NUM_PHYS_BANKS; ++i) {
            banks_[i].clk_i(clk_i);
            banks_[i].rst_ni(rst_ni);
            banks_[i].req_i(l3_bank_req[i]);
            banks_[i].addr_i(bank_addr[i]);
            banks_[i].we_i(l3_bank_we[i]);
            banks_[i].be_i(l3_bank_be[i]);
            banks_[i].wdata_i(l3_bank_wdata[i]);
            banks_[i].gnt_o(l3_bank_gnt[i]);
            banks_[i].rvalid_o(l3_bank_rvalid[i]);
            banks_[i].rdata_o(l3_bank_rdata[i]);
        }
    }

    SC_CTOR(top_crossbar)
        : l1_rd_("l1_rd"), l1_wr_("l1_wr"), l2_rd_("l2_rd"), l2_wr_("l2_wr"), l3_("l3"),
          banks_("bank") {
        l1_rd_.init(NUM_RPORT);
        l1_wr_.init(NUM_WPORT);
        l2_rd_.init(NUM_REQ);
        l2_wr_.init(NUM_REQ);
        l3_.init(NUM_BANK);
        banks_.init(NUM_PHYS_BANKS);

        SC_METHOD(hash_rd_addr);
        for (int m = 0; m < NUM_RPORT_PORTS; ++m)
            sensitive << rport_addr_i[m];

        SC_METHOD(hash_wr_addr);
        for (int m = 0; m < NUM_WPORT_PORTS; ++m)
            sensitive << wport_addr_i[m];

        SC_METHOD(compute_bank_addr);
        for (int i = 0; i < NUM_PHYS_BANKS; ++i)
            sensitive << l3_bank_addr[i];

        bind_l1_read();
        bind_l1_write();
        bind_l2_read();
        bind_l2_write();
        bind_l3_and_banks();
    }
};

#endif
