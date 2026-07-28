// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Native SystemC crossbar DUT (mirrors top_crossbar.sv). Pipeline: port ->
// addr_hash -> L1 -> L2 -> L3 -> bank. L1/L2 are separate per read/write path;
// L3 and the banks are shared. addr_hash.hpp holds the L1/L2 hash formulas.
//
// Address layout (defaults: BYTES_PER_ROW=16, NUM_REQ=4, NUM_BANK_GRP=8):
//   addr[ROUTE_LSB-1:0]                     byte offset in row
//   addr[ROUTE_LSB          +: LOG_REQ]     L1 select
//   addr[ROUTE_LSB+LOG_REQ  +: LOG_BANK_GRP] L2 select
//   addr[ROUTE_LSB+LOG_REQ+LOG_BANK_GRP]    L3 even/odd select
//   addr[31:ROUTE_LSB+ROUTE_BITS]           bank-local row
//
// Template params: NUM_RPORT/NUM_WPORT (port counts), NUM_REQ (OBI buses per
// port), NUM_BANK, NUM_ROW, BYTES_PER_WORD, WORDS_PER_ROW.
// -----------------------------------------------------------------------------

#ifndef TOP_CROSSBAR_HPP
#define TOP_CROSSBAR_HPP

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <systemc.h>

#include "addr_hash.hpp"
#include "bank.hpp"
#include "crossbar.hpp"
#include "map_func.hpp"
#include "obi_ports.hpp"

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

#if defined(XBAR_HASH_DYNAMIC) || defined(XBAR_HASH16) || defined(XBAR_HASH32) || defined(XBAR_HASH_L1_V2)
    // The dynamic hash below writes a map_func::map_one() bank_id (a fixed
    // 5-bit value, see doc/specs/map_func.md's "32 banks" note) into the
    // L1+L2 logical-bank field, so that field must be exactly 5 bits wide.
    // XBAR_HASH_L1_V2's vector_multiport_addr() has the same 5-bit-field
    // dependency (hardcoded 0x1F masks), so it needs this too.
    static_assert(LOG_REQ + LOG_BANK_GRP == 5,
                  "XBAR_HASH_DYNAMIC/XBAR_HASH16/XBAR_HASH32/XBAR_HASH_L1_V2 need a 5-bit logical bank field (NUM_BANK == 32)");
#endif

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

#if defined(XBAR_HASH_DYNAMIC) || defined(XBAR_HASH16) || defined(XBAR_HASH32) || defined(XBAR_HASH_L1_V2)
    // Per-port-group task geometry (one scalar set per read/write driver
    // group, broadcast to that group's NUM_REQ lanes), fed by tb_top.cpp from
    // each AGU's current R/C/L/store_mode. XBAR_HASH_L1_V2 only reads R/C.
    sc_in<uint64_t> rport_map_r_i[NUM_RPORT];
    sc_in<uint64_t> rport_map_c_i[NUM_RPORT];
    sc_in<uint64_t> rport_map_l_i[NUM_RPORT];
    sc_in<uint64_t> rport_map_store_mode_i[NUM_RPORT];
    sc_in<uint64_t> wport_map_r_i[NUM_WPORT];
    sc_in<uint64_t> wport_map_c_i[NUM_WPORT];
    sc_in<uint64_t> wport_map_l_i[NUM_WPORT];
    sc_in<uint64_t> wport_map_store_mode_i[NUM_WPORT];
#endif

#if defined(XBAR_HASH_L1_V2)
    // Per-port-group "exactly one physical port active this task" flag
    // (ports_used_ <= NUM_REQ) — selects between vector_axis_fold_addr() and
    // vector_multiport_addr() for vector-axis tasks (is_vector_geometry()).
    sc_in<bool> rport_map_napa1_i[NUM_RPORT];
    sc_in<bool> wport_map_napa1_i[NUM_WPORT];
#endif

#if defined(XBAR_HASH16)
    // Per-port-group static "high half" selector: hash16_combine() only
    // produces 4 bits (banks 0-15), so this fixed, address-independent
    // per-AGU bit picks which half — partitioning AGUs onto disjoint bank
    // halves so different AGUs' traffic never shares a physical bank,
    // independent of whatever hash16_combine achieves within a half.
    sc_in<bool> rport_map_hi_bank_i[NUM_RPORT];
    sc_in<bool> wport_map_hi_bank_i[NUM_WPORT];
#endif

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
    obi_signal_bundle<data_t> l1_l2_rd[NUM_L1_L2_RD];
    obi_signal_bundle<data_t> l1_l2_wr[NUM_L1_L2_WR];

    // -----------------------------------------------------------------------
    // Inter-level wires: L2 -> L3
    //   Index [k*NUM_BANK_GRP+g]: L2 instance k, slave g -> L3 logical bank b
    // -----------------------------------------------------------------------
    obi_signal_bundle<data_t> l2_l3_rd[NUM_L2_L3];
    obi_signal_bundle<data_t> l2_l3_wr[NUM_L2_L3];

    // -----------------------------------------------------------------------
    // Inter-level wires: L3 -> physical banks
    //   Index [b*2+i]: L3 instance b, slave i (0=even, 1=odd)
    // -----------------------------------------------------------------------
    obi_signal_bundle<data_t> l3_bank[NUM_PHYS_BANKS];

    // -----------------------------------------------------------------------
    // Submodules
    // -----------------------------------------------------------------------
    sc_vector<crossbar<NUM_REQ, NUM_REQ, BYTES_PER_ROW, ROUTE_LSB, LOG_REQ>>          l1_rd_;
    sc_vector<crossbar<NUM_REQ, NUM_REQ, BYTES_PER_ROW, ROUTE_LSB, LOG_REQ>>          l1_wr_;
    sc_vector<crossbar<NUM_RPORT, NUM_BANK_GRP, BYTES_PER_ROW, L2_SEL, LOG_BANK_GRP>> l2_rd_;
    sc_vector<crossbar<NUM_WPORT, NUM_BANK_GRP, BYTES_PER_ROW, L2_SEL, LOG_BANK_GRP>> l2_wr_;
    sc_vector<crossbar<2, 2, BYTES_PER_ROW, L3_SEL, 1>>                               l3_;
    sc_vector<bank<NUM_ROW / 2, BYTES_PER_ROW>>                                       banks_;

    // All bank-hash/address-scrambling formulas live in addr_hash.hpp,
    // parameterized identically to this module's own derived constants —
    // see that file for every addr_hash()/addr_hash16()/etc. definition.
    using hash_ops = addr_hash_ops<ROUTE_LSB, LOG_REQ, LOG_BANK_GRP, NUM_BANK, BYTES_PER_ROW>;

    static uint64_t local_addr(uint64_t a) {
        const uint64_t below = a & ((1ULL << ROUTE_LSB) - 1);
        const uint64_t above = a >> (ROUTE_LSB + ROUTE_BITS);
        return (above << ROUTE_LSB) | below;
    }

    void hash_rd_addr() {
#if defined(XBAR_HASH_DYNAMIC)
        for (int j = 0; j < NUM_RPORT; ++j) {
            const uint64_t R = rport_map_r_i[j].read(), C = rport_map_c_i[j].read(),
                           L = rport_map_l_i[j].read(), sm = rport_map_store_mode_i[j].read();
            for (int m = 0; m < NUM_REQ; ++m) {
                const int ext = j * NUM_REQ + m;
                rport_haddr[ext].write(hash_ops::addr_hash_dynamic(rport_addr_i[ext].read(), R, C, L, sm));
            }
        }
#elif defined(XBAR_HASH16)
        for (int j = 0; j < NUM_RPORT; ++j) {
            const uint64_t R = rport_map_r_i[j].read(), C = rport_map_c_i[j].read(),
                           L = rport_map_l_i[j].read(), sm = rport_map_store_mode_i[j].read();
            const bool     hi_bank = rport_map_hi_bank_i[j].read();
            for (int m = 0; m < NUM_REQ; ++m) {
                const int      ext = j * NUM_REQ + m;
                uint64_t       h   = hash_ops::addr_hash16(rport_addr_i[ext].read(), R, C, L, sm, hi_bank);
#if defined(XBAR_HASH_L2_COMPOSE)
                h = hash_ops::addr_hash(hash_ops::addr_hash_inv(h)); // sanity check — see addr_hash_inv()'s comment
#endif
                rport_haddr[ext].write(h);
            }
        }
#elif defined(XBAR_HASH32)
        for (int j = 0; j < NUM_RPORT; ++j) {
            const uint64_t R = rport_map_r_i[j].read(), C = rport_map_c_i[j].read(),
                           L = rport_map_l_i[j].read(), sm = rport_map_store_mode_i[j].read();
            for (int m = 0; m < NUM_REQ; ++m) {
                const int      ext = j * NUM_REQ + m;
                uint64_t       h   = hash_ops::addr_hash32(rport_addr_i[ext].read(), R, C, L, sm);
#if defined(XBAR_HASH_L2_COMPOSE)
                h = hash_ops::addr_hash(hash_ops::addr_hash_inv(h)); // sanity check — see the HASH16 branch's comment
#endif
                rport_haddr[ext].write(h);
            }
        }
#elif defined(XBAR_HASH_L1_V2)
        // Per port-group: a vector-axis task takes the geometry-aware
        // repair (split by napa1), everything else takes the ordinary fixed
        // fold via addr_hash(). Every branch is a pure per-address function
        // — none reads another lane's address (see addr_hash()'s comment on
        // why an adaptive, cross-lane variant was rejected).
        for (int j = 0; j < NUM_RPORT; ++j) {
            const uint64_t R = rport_map_r_i[j].read(), C = rport_map_c_i[j].read();
            const bool     napa1  = rport_map_napa1_i[j].read();
            const bool     is_vec = hash_ops::is_vector_geometry(R, C);
            for (int m = 0; m < NUM_REQ; ++m) {
                const int      ext = j * NUM_REQ + m;
                const uint64_t a   = rport_addr_i[ext].read();
                rport_haddr[ext].write(
                    is_vec ? (napa1 ? hash_ops::vector_axis_fold_addr(a) : hash_ops::vector_multiport_addr(a))
                           : hash_ops::addr_hash(a));
            }
        }
#else
        for (int m = 0; m < NUM_RPORT_PORTS; ++m)
            rport_haddr[m].write(hash_ops::addr_hash(rport_addr_i[m].read()));
#endif
    }

    void hash_wr_addr() {
#if defined(XBAR_HASH_DYNAMIC)
        for (int j = 0; j < NUM_WPORT; ++j) {
            const uint64_t R = wport_map_r_i[j].read(), C = wport_map_c_i[j].read(),
                           L = wport_map_l_i[j].read(), sm = wport_map_store_mode_i[j].read();
            for (int m = 0; m < NUM_REQ; ++m) {
                const int ext = j * NUM_REQ + m;
                wport_haddr[ext].write(hash_ops::addr_hash_dynamic(wport_addr_i[ext].read(), R, C, L, sm));
            }
        }
#elif defined(XBAR_HASH16)
        for (int j = 0; j < NUM_WPORT; ++j) {
            const uint64_t R = wport_map_r_i[j].read(), C = wport_map_c_i[j].read(),
                           L = wport_map_l_i[j].read(), sm = wport_map_store_mode_i[j].read();
            const bool     hi_bank = wport_map_hi_bank_i[j].read();
            for (int m = 0; m < NUM_REQ; ++m) {
                const int      ext = j * NUM_REQ + m;
                uint64_t       h   = hash_ops::addr_hash16(wport_addr_i[ext].read(), R, C, L, sm, hi_bank);
#if defined(XBAR_HASH_L2_COMPOSE)
                h = hash_ops::addr_hash(hash_ops::addr_hash_inv(h)); // sanity check — see hash_rd_addr()'s comment
#endif
                wport_haddr[ext].write(h);
            }
        }
#elif defined(XBAR_HASH32)
        for (int j = 0; j < NUM_WPORT; ++j) {
            const uint64_t R = wport_map_r_i[j].read(), C = wport_map_c_i[j].read(),
                           L = wport_map_l_i[j].read(), sm = wport_map_store_mode_i[j].read();
            for (int m = 0; m < NUM_REQ; ++m) {
                const int      ext = j * NUM_REQ + m;
                uint64_t       h   = hash_ops::addr_hash32(wport_addr_i[ext].read(), R, C, L, sm);
#if defined(XBAR_HASH_L2_COMPOSE)
                h = hash_ops::addr_hash(hash_ops::addr_hash_inv(h)); // sanity check — see hash_rd_addr()'s comment
#endif
                wport_haddr[ext].write(h);
            }
        }
#elif defined(XBAR_HASH_L1_V2)
        for (int j = 0; j < NUM_WPORT; ++j) {
            const uint64_t R = wport_map_r_i[j].read(), C = wport_map_c_i[j].read();
            const bool     napa1  = wport_map_napa1_i[j].read();
            const bool     is_vec = hash_ops::is_vector_geometry(R, C);
            for (int m = 0; m < NUM_REQ; ++m) {
                const int      ext = j * NUM_REQ + m;
                const uint64_t a   = wport_addr_i[ext].read();
                wport_haddr[ext].write(
                    is_vec ? (napa1 ? hash_ops::vector_axis_fold_addr(a) : hash_ops::vector_multiport_addr(a))
                           : hash_ops::addr_hash(a));
            }
        }
#else
        for (int m = 0; m < NUM_WPORT_PORTS; ++m)
            wport_haddr[m].write(hash_ops::addr_hash(wport_addr_i[m].read()));
#endif
    }

    void compute_bank_addr() {
        for (int i = 0; i < NUM_PHYS_BANKS; ++i)
            bank_addr[i].write(local_addr(l3_bank[i].addr.read()));
    }

    void bind_l1_read() {
        for (int j = 0; j < NUM_RPORT; ++j) {
            l1_rd_[j].clk_i(clk_i);
            l1_rd_[j].rst_ni(rst_ni);
            for (int m = 0; m < NUM_REQ; ++m) {
                const int ext = j * NUM_REQ + m;
                l1_rd_[j].m_ports[m].req_i(rport_req_i[ext]);
                l1_rd_[j].m_ports[m].addr_i(rport_haddr[ext]);
                l1_rd_[j].m_ports[m].we_i(rport_we_i[ext]);
                l1_rd_[j].m_ports[m].be_i(rport_be_i[ext]);
                l1_rd_[j].m_ports[m].wdata_i(rport_wdata_i[ext]);
                l1_rd_[j].m_ports[m].gnt_o(rport_gnt_o[ext]);
                l1_rd_[j].m_ports[m].rvalid_o(rport_rvalid_o[ext]);
                l1_rd_[j].m_ports[m].rdata_o(rport_rdata_o[ext]);

                bind_obi(l1_rd_[j].b_ports[m], l1_l2_rd[ext]);
            }
        }
    }

    void bind_l1_write() {
        for (int j = 0; j < NUM_WPORT; ++j) {
            l1_wr_[j].clk_i(clk_i);
            l1_wr_[j].rst_ni(rst_ni);
            for (int m = 0; m < NUM_REQ; ++m) {
                const int ext = j * NUM_REQ + m;
                l1_wr_[j].m_ports[m].req_i(wport_req_i[ext]);
                l1_wr_[j].m_ports[m].addr_i(wport_haddr[ext]);
                l1_wr_[j].m_ports[m].we_i(wport_we_i[ext]);
                l1_wr_[j].m_ports[m].be_i(wport_be_i[ext]);
                l1_wr_[j].m_ports[m].wdata_i(wport_wdata_i[ext]);
                l1_wr_[j].m_ports[m].gnt_o(wport_gnt_o[ext]);
                l1_wr_[j].m_ports[m].rvalid_o(wport_rvalid_o[ext]);
                l1_wr_[j].m_ports[m].rdata_o(wport_rdata_o[ext]);

                bind_obi(l1_wr_[j].b_ports[m], l1_l2_wr[ext]);
            }
        }
    }

    void bind_l2_read() {
        for (int k = 0; k < NUM_REQ; ++k) {
            l2_rd_[k].clk_i(clk_i);
            l2_rd_[k].rst_ni(rst_ni);
            for (int j = 0; j < NUM_RPORT; ++j) {
                const int sig = j * NUM_REQ + k;
                bind_obi(l2_rd_[k].m_ports[j], l1_l2_rd[sig]);
            }
            for (int g = 0; g < NUM_BANK_GRP; ++g) {
                const int b = k * NUM_BANK_GRP + g;
                bind_obi(l2_rd_[k].b_ports[g], l2_l3_rd[b]);
            }
        }
    }

    void bind_l2_write() {
        for (int k = 0; k < NUM_REQ; ++k) {
            l2_wr_[k].clk_i(clk_i);
            l2_wr_[k].rst_ni(rst_ni);
            for (int j = 0; j < NUM_WPORT; ++j) {
                const int sig = j * NUM_REQ + k;
                bind_obi(l2_wr_[k].m_ports[j], l1_l2_wr[sig]);
            }
            for (int g = 0; g < NUM_BANK_GRP; ++g) {
                const int b = k * NUM_BANK_GRP + g;
                bind_obi(l2_wr_[k].b_ports[g], l2_l3_wr[b]);
            }
        }
    }

    void bind_l3_and_banks() {
        for (int b = 0; b < NUM_BANK; ++b) {
            l3_[b].clk_i(clk_i);
            l3_[b].rst_ni(rst_ni);

            bind_obi(l3_[b].m_ports[0], l2_l3_rd[b]);
            bind_obi(l3_[b].m_ports[1], l2_l3_wr[b]);

            for (int i = 0; i < 2; ++i) {
                const int ph = b * 2 + i;
                bind_obi(l3_[b].b_ports[i], l3_bank[ph]);
            }
        }

        for (int i = 0; i < NUM_PHYS_BANKS; ++i) {
            banks_[i].clk_i(clk_i);
            banks_[i].rst_ni(rst_ni);
            // Not bind_obi(banks_[i].obi, l3_bank[i]): addr_i must take the
            // routing-stripped bank_addr, not l3_bank[i].addr itself.
            banks_[i].obi.req_i(l3_bank[i].req);
            banks_[i].obi.addr_i(bank_addr[i]);
            banks_[i].obi.we_i(l3_bank[i].we);
            banks_[i].obi.be_i(l3_bank[i].be);
            banks_[i].obi.wdata_i(l3_bank[i].wdata);
            banks_[i].obi.gnt_o(l3_bank[i].gnt);
            banks_[i].obi.rvalid_o(l3_bank[i].rvalid);
            banks_[i].obi.rdata_o(l3_bank[i].rdata);
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
#if defined(XBAR_HASH_DYNAMIC) || defined(XBAR_HASH16) || defined(XBAR_HASH32) || defined(XBAR_HASH_L1_V2)
        for (int j = 0; j < NUM_RPORT; ++j)
            sensitive << rport_map_r_i[j] << rport_map_c_i[j] << rport_map_l_i[j]
                      << rport_map_store_mode_i[j];
#endif
#if defined(XBAR_HASH_L1_V2)
        for (int j = 0; j < NUM_RPORT; ++j)
            sensitive << rport_map_napa1_i[j];
#endif
#if defined(XBAR_HASH16)
        for (int j = 0; j < NUM_RPORT; ++j)
            sensitive << rport_map_hi_bank_i[j];
#endif

        SC_METHOD(hash_wr_addr);
        for (int m = 0; m < NUM_WPORT_PORTS; ++m)
            sensitive << wport_addr_i[m];
#if defined(XBAR_HASH_DYNAMIC) || defined(XBAR_HASH16) || defined(XBAR_HASH32) || defined(XBAR_HASH_L1_V2)
        for (int j = 0; j < NUM_WPORT; ++j)
            sensitive << wport_map_r_i[j] << wport_map_c_i[j] << wport_map_l_i[j]
                      << wport_map_store_mode_i[j];
#endif
#if defined(XBAR_HASH_L1_V2)
        for (int j = 0; j < NUM_WPORT; ++j)
            sensitive << wport_map_napa1_i[j];
#endif
#if defined(XBAR_HASH16)
        for (int j = 0; j < NUM_WPORT; ++j)
            sensitive << wport_map_hi_bank_i[j];
#endif

        SC_METHOD(compute_bank_addr);
        for (int i = 0; i < NUM_PHYS_BANKS; ++i)
            sensitive << l3_bank[i].addr;

        bind_l1_read();
        bind_l1_write();
        bind_l2_read();
        bind_l2_write();
        bind_l3_and_banks();
    }
};

#endif
