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

#ifdef XBAR_HASH_DYNAMIC
    // The dynamic hash below writes a map_func::map_one() bank_id (a fixed
    // 5-bit value, see doc/specs/map_func.md's "32 banks" note) into the
    // L1+L2 logical-bank field, so that field must be exactly 5 bits wide.
    static_assert(LOG_REQ + LOG_BANK_GRP == 5,
                  "XBAR_HASH_DYNAMIC needs a 5-bit logical bank field (NUM_BANK == 32)");
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

#ifdef XBAR_HASH_DYNAMIC
    // Per-port-group mapping geometry (experimental) — one scalar set per
    // read/write driver group, broadcast to that group's NUM_REQ lanes, same
    // shape as top_tdm.hpp's buf_map_r_i/etc. Driven by tb_top.cpp from each
    // AGU's current p_R_/p_C_/p_L_/p_store_mode_ (see agu.hpp's "TDM mapping
    // note"). Only present in this build; the default static addr_hash needs
    // no mapping geometry.
    sc_in<uint64_t> rport_map_r_i[NUM_RPORT];
    sc_in<uint64_t> rport_map_c_i[NUM_RPORT];
    sc_in<uint64_t> rport_map_l_i[NUM_RPORT];
    sc_in<uint64_t> rport_map_store_mode_i[NUM_RPORT];
    sc_in<uint64_t> wport_map_r_i[NUM_WPORT];
    sc_in<uint64_t> wport_map_c_i[NUM_WPORT];
    sc_in<uint64_t> wport_map_l_i[NUM_WPORT];
    sc_in<uint64_t> wport_map_store_mode_i[NUM_WPORT];
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

    static uint64_t addr_hash(uint64_t a) {
        const uint64_t hi  = (a >> 9) & 0x7;
        const uint64_t mid = (a >> 6) & 0x7;
        const uint64_t sum = (hi + mid) & 0x7;
        uint64_t       r   = (a & ~(static_cast<uint64_t>(0x7) << 6)) | (sum << 6);
#ifdef XBAR_HASH_L1
        // Experimental (doc/report Appendix A.8): also scramble the L1-select bits
        // addr[5:4] with addr[11:10] — the same construction the L2 stage
        // uses. Within-frame beat sets that agree in [5:4] (strided walks,
        // the dominant L1 self-collision class) then spread across L1
        // outputs whenever their higher bits differ. Bijective within the
        // stripped routing field, so bank/row decode is unaffected.
        const uint64_t l1 = ((a >> 4) + (a >> 10)) & 0x3;
        r                 = (r & ~(static_cast<uint64_t>(0x3) << 4)) | (l1 << 4);
#endif
        return r;
    }

#ifdef XBAR_HASH_DYNAMIC
    // Dynamic hash (experimental, doc/report Appendix A.8): reuses the TDM
    // XOR-skew placement (map_func.hpp, same scheme as tdm.hpp) to pick the
    // L1+L2 logical-bank field from the group's current R/C/L/store_mode,
    // instead of addr_hash()'s fixed bit-mixing above. L3's even/odd select
    // bit and everything above stay raw address bits, untouched — same scope
    // as addr_hash()'s own L2-only hash.
    static uint64_t addr_hash_dynamic(uint64_t a, uint64_t R, uint64_t C, uint64_t L,
                                      uint64_t store_mode) {
        uint64_t bank_id = 0, row_id = 0;
        map_func::map_one(a, static_cast<uint64_t>(NUM_BANK), static_cast<uint64_t>(BYTES_PER_ROW), R,
                          C, L, static_cast<tdm_stor_mode>(store_mode), bank_id, row_id);
        const uint64_t field_mask = (1ull << (LOG_REQ + LOG_BANK_GRP)) - 1;
        return (a & ~(field_mask << ROUTE_LSB)) | (bank_id << ROUTE_LSB);
    }
#endif

    static uint64_t local_addr(uint64_t a) {
        const uint64_t below = a & ((1ULL << ROUTE_LSB) - 1);
        const uint64_t above = a >> (ROUTE_LSB + ROUTE_BITS);
        return (above << ROUTE_LSB) | below;
    }

    void hash_rd_addr() {
#ifdef XBAR_HASH_DYNAMIC
        for (int j = 0; j < NUM_RPORT; ++j) {
            const uint64_t R = rport_map_r_i[j].read(), C = rport_map_c_i[j].read(),
                           L = rport_map_l_i[j].read(), sm = rport_map_store_mode_i[j].read();
            for (int m = 0; m < NUM_REQ; ++m) {
                const int ext = j * NUM_REQ + m;
                rport_haddr[ext].write(addr_hash_dynamic(rport_addr_i[ext].read(), R, C, L, sm));
            }
        }
#else
        for (int m = 0; m < NUM_RPORT_PORTS; ++m)
            rport_haddr[m].write(addr_hash(rport_addr_i[m].read()));
#endif
    }

    void hash_wr_addr() {
#ifdef XBAR_HASH_DYNAMIC
        for (int j = 0; j < NUM_WPORT; ++j) {
            const uint64_t R = wport_map_r_i[j].read(), C = wport_map_c_i[j].read(),
                           L = wport_map_l_i[j].read(), sm = wport_map_store_mode_i[j].read();
            for (int m = 0; m < NUM_REQ; ++m) {
                const int ext = j * NUM_REQ + m;
                wport_haddr[ext].write(addr_hash_dynamic(wport_addr_i[ext].read(), R, C, L, sm));
            }
        }
#else
        for (int m = 0; m < NUM_WPORT_PORTS; ++m)
            wport_haddr[m].write(addr_hash(wport_addr_i[m].read()));
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
#ifdef XBAR_HASH_DYNAMIC
        for (int j = 0; j < NUM_RPORT; ++j)
            sensitive << rport_map_r_i[j] << rport_map_c_i[j] << rport_map_l_i[j]
                      << rport_map_store_mode_i[j];
#endif

        SC_METHOD(hash_wr_addr);
        for (int m = 0; m < NUM_WPORT_PORTS; ++m)
            sensitive << wport_addr_i[m];
#ifdef XBAR_HASH_DYNAMIC
        for (int j = 0; j < NUM_WPORT; ++j)
            sensitive << wport_map_r_i[j] << wport_map_c_i[j] << wport_map_l_i[j]
                      << wport_map_store_mode_i[j];
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
