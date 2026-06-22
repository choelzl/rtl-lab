// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Description:
//   Native SystemC design top (DUT) for the crossbar design, mirroring the
//   three-level SV architecture in top_crossbar.sv.
//
//   Pipeline:
//     AGU → addr_hash → L1[j] → L2[k] → L3[b] → bank[b*2+{0,1}]
//
//   Level 1 (NUM_RAGU + NUM_WAGU instances, NUM_REQ×NUM_REQ each):
//     One xbar_slice per AGU.  Routes each AGU's NUM_REQ ports to NUM_REQ L2
//     groups using addr[ROUTE_LSB +: LOG_REQ].
//
//   Level 2 (NUM_REQ instances each for read and write, NUM_RAGU/WAGU × NUM_BANK_GRP):
//     One xbar_slice per L2 group.  Routes across bank groups using
//     addr[ROUTE_LSB+LOG_REQ +: LOG_BANK_GRP].
//
//   Level 3 (NUM_BANK instances, 2×2):
//     One xbar_slice per logical bank.  Merges read and write paths onto
//     even/odd physical banks using addr[ROUTE_LSB+LOG_REQ+LOG_BANK_GRP] (bit 9
//     for default parameters).
//
//   Banks: NUM_BANK*2 physical banks, each NUM_ROW/2 rows.  The routing-bit
//   field (ROUTE_BITS wide starting at ROUTE_LSB) is stripped from the address
//   before it reaches the bank.
//
//   addr_hash scrambles the L2-select bits (addr[8:6]) by adding the
//   overlapping addr[11:9] field, matching top_crossbar.sv.
//
//   Address layout (defaults: BYTES_PER_ROW=16, NUM_REQ=4, NUM_BANK_GRP=8):
//     addr[ROUTE_LSB-1:0]                              byte offset in row
//     addr[ROUTE_LSB        +: LOG_REQ     ]           L1 select
//     addr[ROUTE_LSB+LOG_REQ +: LOG_BANK_GRP]          L2 select
//     addr[ROUTE_LSB+LOG_REQ+LOG_BANK_GRP]             L3 even/odd select
//     addr[31:ROUTE_LSB+ROUTE_BITS]                    bank-local row
//
// Template parameters:
//   NUM_RAGU, NUM_WAGU, NUM_REQ, NUM_BANK, NUM_ROW, BYTES_PER_WORD, WORDS_PER_ROW
// -----------------------------------------------------------------------------

#ifndef TOP_CROSSBAR_HPP
#define TOP_CROSSBAR_HPP

#include <cstdint>
#include <systemc.h>

#include "bank.hpp"
#include "crossbar.hpp"

namespace tc_detail {
constexpr int log2f(int n) {
    return n <= 1 ? 0 : 1 + log2f(n >> 1);
}
} // namespace tc_detail

template <int NUM_RAGU = 2, int NUM_WAGU = 2, int NUM_REQ = 4, int NUM_BANK = 8, int NUM_ROW = 1024,
          int BYTES_PER_WORD = 4, int WORDS_PER_ROW = 4>
SC_MODULE(top_crossbar) {
    // -----------------------------------------------------------------------
    // Derived constants
    // -----------------------------------------------------------------------
    static constexpr int NUM_RD        = NUM_RAGU * NUM_REQ;
    static constexpr int NUM_WR        = NUM_WAGU * NUM_REQ;
    static constexpr int NUM_BANK_GRP  = NUM_BANK / NUM_REQ;
    static constexpr int BYTES_PER_ROW = WORDS_PER_ROW * BYTES_PER_WORD;
    static constexpr int ROUTE_LSB     = tc_detail::log2f(BYTES_PER_ROW);
    static constexpr int LOG_REQ       = tc_detail::log2f(NUM_REQ);
    static constexpr int LOG_BANK_GRP  = tc_detail::log2f(NUM_BANK_GRP);
    static constexpr int ROUTE_BITS    = LOG_REQ + LOG_BANK_GRP + 1;
    static constexpr int L3_SEL        = ROUTE_LSB + LOG_REQ + LOG_BANK_GRP;

    // Signal counts for inter-level wires
    static constexpr int N_L1L2_RD = NUM_RAGU * NUM_REQ; // = NUM_RD
    static constexpr int N_L1L2_WR = NUM_WAGU * NUM_REQ; // = NUM_WR
    static constexpr int N_BANK    = NUM_BANK;           // L2→L3 per side
    static constexpr int N_PHYS    = NUM_BANK * 2;       // L3→banks

    // -----------------------------------------------------------------------
    // External ports
    // -----------------------------------------------------------------------
    sc_in<bool> clk_i;
    sc_in<bool> rst_ni;

    sc_in<bool>      m_req_i[NUM_RD];
    sc_in<uint64_t>  m_addr_i[NUM_RD];
    sc_in<bool>      m_we_i[NUM_RD];
    sc_in<uint32_t>  m_be_i[NUM_RD];
    sc_in<uint64_t>  m_wdata_i[NUM_RD];
    sc_out<bool>     m_gnt_o[NUM_RD];
    sc_out<bool>     m_rvalid_o[NUM_RD];
    sc_out<uint64_t> m_rdata_o[NUM_RD];

    sc_in<bool>      m_w_req_i[NUM_WR];
    sc_in<uint64_t>  m_w_addr_i[NUM_WR];
    sc_in<bool>      m_w_we_i[NUM_WR];
    sc_in<uint32_t>  m_w_be_i[NUM_WR];
    sc_in<uint64_t>  m_w_wdata_i[NUM_WR];
    sc_out<bool>     m_w_gnt_o[NUM_WR];
    sc_out<bool>     m_w_rvalid_o[NUM_WR];
    sc_out<uint64_t> m_w_rdata_o[NUM_WR];

    // -----------------------------------------------------------------------
    // Hashed address signals (addr_hash applied before L1)
    // -----------------------------------------------------------------------
    sc_signal<uint64_t> rd_haddr[NUM_RD];
    sc_signal<uint64_t> wr_haddr[NUM_WR];

    // -----------------------------------------------------------------------
    // Inter-level wires: L1→L2
    //   Index [j*NUM_REQ+k]: L1 instance j, slave port k
    //                        → L2 instance k, master port j
    // -----------------------------------------------------------------------
    sc_signal<bool>     l1l2_rd_req[N_L1L2_RD];
    sc_signal<uint64_t> l1l2_rd_addr[N_L1L2_RD];
    sc_signal<bool>     l1l2_rd_we[N_L1L2_RD];
    sc_signal<uint32_t> l1l2_rd_be[N_L1L2_RD];
    sc_signal<uint64_t> l1l2_rd_wdata[N_L1L2_RD];
    sc_signal<bool>     l1l2_rd_gnt[N_L1L2_RD];
    sc_signal<bool>     l1l2_rd_rvalid[N_L1L2_RD];
    sc_signal<uint64_t> l1l2_rd_rdata[N_L1L2_RD];

    sc_signal<bool>     l1l2_wr_req[N_L1L2_WR];
    sc_signal<uint64_t> l1l2_wr_addr[N_L1L2_WR];
    sc_signal<bool>     l1l2_wr_we[N_L1L2_WR];
    sc_signal<uint32_t> l1l2_wr_be[N_L1L2_WR];
    sc_signal<uint64_t> l1l2_wr_wdata[N_L1L2_WR];
    sc_signal<bool>     l1l2_wr_gnt[N_L1L2_WR];
    sc_signal<bool>     l1l2_wr_rvalid[N_L1L2_WR];
    sc_signal<uint64_t> l1l2_wr_rdata[N_L1L2_WR];

    // -----------------------------------------------------------------------
    // Inter-level wires: L2→L3
    //   Index [k*NUM_BANK_GRP+g]: L2 instance k, slave g → L3 instance b=k*NUM_BANK_GRP+g
    // -----------------------------------------------------------------------
    sc_signal<bool>     l2l3_rd_req[N_BANK];
    sc_signal<uint64_t> l2l3_rd_addr[N_BANK];
    sc_signal<bool>     l2l3_rd_we[N_BANK];
    sc_signal<uint32_t> l2l3_rd_be[N_BANK];
    sc_signal<uint64_t> l2l3_rd_wdata[N_BANK];
    sc_signal<bool>     l2l3_rd_gnt[N_BANK];
    sc_signal<bool>     l2l3_rd_rvalid[N_BANK];
    sc_signal<uint64_t> l2l3_rd_rdata[N_BANK];

    sc_signal<bool>     l2l3_wr_req[N_BANK];
    sc_signal<uint64_t> l2l3_wr_addr[N_BANK];
    sc_signal<bool>     l2l3_wr_we[N_BANK];
    sc_signal<uint32_t> l2l3_wr_be[N_BANK];
    sc_signal<uint64_t> l2l3_wr_wdata[N_BANK];
    sc_signal<bool>     l2l3_wr_gnt[N_BANK];
    sc_signal<bool>     l2l3_wr_rvalid[N_BANK];
    sc_signal<uint64_t> l2l3_wr_rdata[N_BANK];

    // -----------------------------------------------------------------------
    // Inter-level wires: L3→physical banks
    //   Index [b*2+i]: L3 instance b, slave i (0=even, 1=odd)
    // -----------------------------------------------------------------------
    sc_signal<bool>     l3bk_req[N_PHYS];
    sc_signal<uint64_t> l3bk_addr[N_PHYS];
    sc_signal<bool>     l3bk_we[N_PHYS];
    sc_signal<uint32_t> l3bk_be[N_PHYS];
    sc_signal<uint64_t> l3bk_wdata[N_PHYS];
    sc_signal<bool>     l3bk_gnt[N_PHYS];
    sc_signal<bool>     l3bk_rvalid[N_PHYS];
    sc_signal<uint64_t> l3bk_rdata[N_PHYS];

    // Bank-local addresses (routing bits stripped)
    sc_signal<uint64_t> bk_laddr[N_PHYS];

    // -----------------------------------------------------------------------
    // Submodules
    // -----------------------------------------------------------------------
    sc_vector<crossbar<NUM_REQ, NUM_REQ, 1, ROUTE_LSB, LOG_REQ>>                      l1_rd_;
    sc_vector<crossbar<NUM_REQ, NUM_REQ, 1, ROUTE_LSB, LOG_REQ>>                      l1_wr_;
    sc_vector<crossbar<NUM_RAGU, NUM_BANK_GRP, 1, ROUTE_LSB + LOG_REQ, LOG_BANK_GRP>> l2_rd_;
    sc_vector<crossbar<NUM_WAGU, NUM_BANK_GRP, 1, ROUTE_LSB + LOG_REQ, LOG_BANK_GRP>> l2_wr_;
    sc_vector<crossbar<2, 2, 1, L3_SEL, 1>>                                           l3_;
    sc_vector<bank<NUM_ROW / 2, BYTES_PER_ROW>>                                       banks_;

    // -----------------------------------------------------------------------
    // addr_hash: scrambles L2-select bits (addr[8:6] += addr[11:9])
    // -----------------------------------------------------------------------
    static uint64_t addr_hash(uint64_t a) {
        const uint64_t hi  = (a >> 9) & 0x7;
        const uint64_t mid = (a >> 6) & 0x7;
        const uint64_t sum = (hi + mid) & 0x7;
        return (a & ~(static_cast<uint64_t>(0x7) << 6)) | (sum << 6);
    }

    // Strip ROUTE_BITS-wide routing field at ROUTE_LSB, compacting the address
    static uint64_t local_addr(uint64_t a) {
        const uint64_t below = a & ((1ULL << ROUTE_LSB) - 1);
        const uint64_t above = a >> (ROUTE_LSB + ROUTE_BITS);
        return (above << ROUTE_LSB) | below;
    }

    void hash_rd_addr() {
        for (int m = 0; m < NUM_RD; ++m)
            rd_haddr[m].write(addr_hash(m_addr_i[m].read()));
    }
    void hash_wr_addr() {
        for (int m = 0; m < NUM_WR; ++m)
            wr_haddr[m].write(addr_hash(m_w_addr_i[m].read()));
    }
    void compute_bk_laddr() {
        for (int i = 0; i < N_PHYS; ++i)
            bk_laddr[i].write(local_addr(l3bk_addr[i].read()));
    }

    SC_CTOR(top_crossbar)
        : l1_rd_("l1_rd"), l1_wr_("l1_wr"), l2_rd_("l2_rd"), l2_wr_("l2_wr"), l3_("l3"),
          banks_("bank") {
        l1_rd_.init(NUM_RAGU);
        l1_wr_.init(NUM_WAGU);
        l2_rd_.init(NUM_REQ);
        l2_wr_.init(NUM_REQ);
        l3_.init(NUM_BANK);
        banks_.init(N_PHYS);

        // addr_hash processes
        SC_METHOD(hash_rd_addr);
        for (int m = 0; m < NUM_RD; ++m)
            sensitive << m_addr_i[m];

        SC_METHOD(hash_wr_addr);
        for (int m = 0; m < NUM_WR; ++m)
            sensitive << m_w_addr_i[m];

        SC_METHOD(compute_bk_laddr);
        for (int i = 0; i < N_PHYS; ++i)
            sensitive << l3bk_addr[i];

        // ----- L1 read -------------------------------------------------------
        // L1_RD[j]: AGU j's NUM_REQ ports → NUM_REQ L2_RD groups
        for (int j = 0; j < NUM_RAGU; ++j) {
            l1_rd_[j].clk_i(clk_i);
            l1_rd_[j].rst_ni(rst_ni);
            for (int m = 0; m < NUM_REQ; ++m) {
                const int ext = j * NUM_REQ + m;
                l1_rd_[j].m_req_i[m](m_req_i[ext]);
                l1_rd_[j].m_addr_i[m](rd_haddr[ext]);
                l1_rd_[j].m_we_i[m](m_we_i[ext]);
                l1_rd_[j].m_be_i[m](m_be_i[ext]);
                l1_rd_[j].m_wdata_i[m](m_wdata_i[ext]);
                l1_rd_[j].m_gnt_o[m](m_gnt_o[ext]);
                l1_rd_[j].m_rvalid_o[m](m_rvalid_o[ext]);
                l1_rd_[j].m_rdata_o[m](m_rdata_o[ext]);
            }
            for (int k = 0; k < NUM_REQ; ++k) {
                const int sig = j * NUM_REQ + k;
                l1_rd_[j].b_req_o[k](l1l2_rd_req[sig]);
                l1_rd_[j].b_addr_o[k](l1l2_rd_addr[sig]);
                l1_rd_[j].b_we_o[k](l1l2_rd_we[sig]);
                l1_rd_[j].b_be_o[k](l1l2_rd_be[sig]);
                l1_rd_[j].b_wdata_o[k](l1l2_rd_wdata[sig]);
                l1_rd_[j].b_gnt_i[k](l1l2_rd_gnt[sig]);
                l1_rd_[j].b_rvalid_i[k](l1l2_rd_rvalid[sig]);
                l1_rd_[j].b_rdata_i[k](l1l2_rd_rdata[sig]);
            }
        }

        // ----- L1 write ------------------------------------------------------
        for (int j = 0; j < NUM_WAGU; ++j) {
            l1_wr_[j].clk_i(clk_i);
            l1_wr_[j].rst_ni(rst_ni);
            for (int m = 0; m < NUM_REQ; ++m) {
                const int ext = j * NUM_REQ + m;
                l1_wr_[j].m_req_i[m](m_w_req_i[ext]);
                l1_wr_[j].m_addr_i[m](wr_haddr[ext]);
                l1_wr_[j].m_we_i[m](m_w_we_i[ext]);
                l1_wr_[j].m_be_i[m](m_w_be_i[ext]);
                l1_wr_[j].m_wdata_i[m](m_w_wdata_i[ext]);
                l1_wr_[j].m_gnt_o[m](m_w_gnt_o[ext]);
                l1_wr_[j].m_rvalid_o[m](m_w_rvalid_o[ext]);
                l1_wr_[j].m_rdata_o[m](m_w_rdata_o[ext]);
            }
            for (int k = 0; k < NUM_REQ; ++k) {
                const int sig = j * NUM_REQ + k;
                l1_wr_[j].b_req_o[k](l1l2_wr_req[sig]);
                l1_wr_[j].b_addr_o[k](l1l2_wr_addr[sig]);
                l1_wr_[j].b_we_o[k](l1l2_wr_we[sig]);
                l1_wr_[j].b_be_o[k](l1l2_wr_be[sig]);
                l1_wr_[j].b_wdata_o[k](l1l2_wr_wdata[sig]);
                l1_wr_[j].b_gnt_i[k](l1l2_wr_gnt[sig]);
                l1_wr_[j].b_rvalid_i[k](l1l2_wr_rvalid[sig]);
                l1_wr_[j].b_rdata_i[k](l1l2_wr_rdata[sig]);
            }
        }

        // ----- L2 read -------------------------------------------------------
        // L2_RD[k]: collects output k from every L1_RD[j], routes to NUM_BANK_GRP banks
        for (int k = 0; k < NUM_REQ; ++k) {
            l2_rd_[k].clk_i(clk_i);
            l2_rd_[k].rst_ni(rst_ni);
            for (int j = 0; j < NUM_RAGU; ++j) {
                const int sig = j * NUM_REQ + k;
                l2_rd_[k].m_req_i[j](l1l2_rd_req[sig]);
                l2_rd_[k].m_addr_i[j](l1l2_rd_addr[sig]);
                l2_rd_[k].m_we_i[j](l1l2_rd_we[sig]);
                l2_rd_[k].m_be_i[j](l1l2_rd_be[sig]);
                l2_rd_[k].m_wdata_i[j](l1l2_rd_wdata[sig]);
                l2_rd_[k].m_gnt_o[j](l1l2_rd_gnt[sig]);
                l2_rd_[k].m_rvalid_o[j](l1l2_rd_rvalid[sig]);
                l2_rd_[k].m_rdata_o[j](l1l2_rd_rdata[sig]);
            }
            for (int g = 0; g < NUM_BANK_GRP; ++g) {
                const int b = k * NUM_BANK_GRP + g;
                l2_rd_[k].b_req_o[g](l2l3_rd_req[b]);
                l2_rd_[k].b_addr_o[g](l2l3_rd_addr[b]);
                l2_rd_[k].b_we_o[g](l2l3_rd_we[b]);
                l2_rd_[k].b_be_o[g](l2l3_rd_be[b]);
                l2_rd_[k].b_wdata_o[g](l2l3_rd_wdata[b]);
                l2_rd_[k].b_gnt_i[g](l2l3_rd_gnt[b]);
                l2_rd_[k].b_rvalid_i[g](l2l3_rd_rvalid[b]);
                l2_rd_[k].b_rdata_i[g](l2l3_rd_rdata[b]);
            }
        }

        // ----- L2 write ------------------------------------------------------
        for (int k = 0; k < NUM_REQ; ++k) {
            l2_wr_[k].clk_i(clk_i);
            l2_wr_[k].rst_ni(rst_ni);
            for (int j = 0; j < NUM_WAGU; ++j) {
                const int sig = j * NUM_REQ + k;
                l2_wr_[k].m_req_i[j](l1l2_wr_req[sig]);
                l2_wr_[k].m_addr_i[j](l1l2_wr_addr[sig]);
                l2_wr_[k].m_we_i[j](l1l2_wr_we[sig]);
                l2_wr_[k].m_be_i[j](l1l2_wr_be[sig]);
                l2_wr_[k].m_wdata_i[j](l1l2_wr_wdata[sig]);
                l2_wr_[k].m_gnt_o[j](l1l2_wr_gnt[sig]);
                l2_wr_[k].m_rvalid_o[j](l1l2_wr_rvalid[sig]);
                l2_wr_[k].m_rdata_o[j](l1l2_wr_rdata[sig]);
            }
            for (int g = 0; g < NUM_BANK_GRP; ++g) {
                const int b = k * NUM_BANK_GRP + g;
                l2_wr_[k].b_req_o[g](l2l3_wr_req[b]);
                l2_wr_[k].b_addr_o[g](l2l3_wr_addr[b]);
                l2_wr_[k].b_we_o[g](l2l3_wr_we[b]);
                l2_wr_[k].b_be_o[g](l2l3_wr_be[b]);
                l2_wr_[k].b_wdata_o[g](l2l3_wr_wdata[b]);
                l2_wr_[k].b_gnt_i[g](l2l3_wr_gnt[b]);
                l2_wr_[k].b_rvalid_i[g](l2l3_wr_rvalid[b]);
                l2_wr_[k].b_rdata_i[g](l2l3_wr_rdata[b]);
            }
        }

        // ----- L3 (even/odd merge) -------------------------------------------
        // L3[b]: read (master 0) and write (master 1) → even bank (slave 0) or odd (slave 1)
        for (int b = 0; b < NUM_BANK; ++b) {
            l3_[b].clk_i(clk_i);
            l3_[b].rst_ni(rst_ni);

            l3_[b].m_req_i[0](l2l3_rd_req[b]);
            l3_[b].m_addr_i[0](l2l3_rd_addr[b]);
            l3_[b].m_we_i[0](l2l3_rd_we[b]);
            l3_[b].m_be_i[0](l2l3_rd_be[b]);
            l3_[b].m_wdata_i[0](l2l3_rd_wdata[b]);
            l3_[b].m_gnt_o[0](l2l3_rd_gnt[b]);
            l3_[b].m_rvalid_o[0](l2l3_rd_rvalid[b]);
            l3_[b].m_rdata_o[0](l2l3_rd_rdata[b]);

            l3_[b].m_req_i[1](l2l3_wr_req[b]);
            l3_[b].m_addr_i[1](l2l3_wr_addr[b]);
            l3_[b].m_we_i[1](l2l3_wr_we[b]);
            l3_[b].m_be_i[1](l2l3_wr_be[b]);
            l3_[b].m_wdata_i[1](l2l3_wr_wdata[b]);
            l3_[b].m_gnt_o[1](l2l3_wr_gnt[b]);
            l3_[b].m_rvalid_o[1](l2l3_wr_rvalid[b]);
            l3_[b].m_rdata_o[1](l2l3_wr_rdata[b]);

            for (int i = 0; i < 2; ++i) {
                const int ph = b * 2 + i;
                l3_[b].b_req_o[i](l3bk_req[ph]);
                l3_[b].b_addr_o[i](l3bk_addr[ph]);
                l3_[b].b_we_o[i](l3bk_we[ph]);
                l3_[b].b_be_o[i](l3bk_be[ph]);
                l3_[b].b_wdata_o[i](l3bk_wdata[ph]);
                l3_[b].b_gnt_i[i](l3bk_gnt[ph]);
                l3_[b].b_rvalid_i[i](l3bk_rvalid[ph]);
                l3_[b].b_rdata_i[i](l3bk_rdata[ph]);
            }
        }

        // ----- Physical banks ------------------------------------------------
        for (int i = 0; i < N_PHYS; ++i) {
            banks_[i].clk_i(clk_i);
            banks_[i].rst_ni(rst_ni);
            banks_[i].req_i(l3bk_req[i]);
            banks_[i].addr_i(bk_laddr[i]);
            banks_[i].we_i(l3bk_we[i]);
            banks_[i].be_i(l3bk_be[i]);
            banks_[i].wdata_i(l3bk_wdata[i]);
            banks_[i].gnt_o(l3bk_gnt[i]);
            banks_[i].rvalid_o(l3bk_rvalid[i]);
            banks_[i].rdata_o(l3bk_rdata[i]);
        }
    }
};

#endif
