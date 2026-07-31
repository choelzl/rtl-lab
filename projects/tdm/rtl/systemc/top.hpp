// -----------------------------------------------------------------------------
// Author: Cedric Hoelzl
//
// Unified SystemC DUT wrapper for the TDM project. IMPL=<tokens> selects an
// implementation (comma-split, uppercased to -DIMPL_<UPPER>, e.g.
// IMPL=crossbar,sv -> -DIMPL_CROSSBAR -DIMPL_SV); the sv token builds the
// Verilated SV backend from SV_MODS=<top module>, else native SystemC.
// Recognised: IMPL=crossbar[,sv SV_MODS=top_crossbar], IMPL=tdm[,sv
// SV_MODS=top_tdm] (wrapper not yet present). IMPL=tdm_sc aliases native TDM.
//
// Packs RAGU/WAGU driver groups onto the flat port arrays the implementation
// tops expect: read 0..3=RAGU_A, 4..5=RAGU_B, 6=RAGU_C, 7=RAGU_D, 8=RAGU_E;
// write 0..3=WAGU_A, 4..5=WAGU_B, 6=WAGU_D, 7=WAGU_E — each *_PORTS =
// NUM_<driver> * NUM_REQ sub-ports (crossbar: 36 read / 32 write flat ports).
// -----------------------------------------------------------------------------

#ifndef TOP_HPP
#define TOP_HPP

#include "obi_data.hpp"
#include "obi_ports.hpp"
#include <cstdint>
#include <systemc.h>

#if !defined(IMPL_TDM) && !defined(IMPL_CROSSBAR)
#define IMPL_CROSSBAR
#endif

#if defined(IMPL_CROSSBAR)
#if defined(IMPL_SV)
#include "top_crossbar_sv.hpp"
#else
#include "top_crossbar.hpp"
#endif
#elif defined(IMPL_TDM)
#if defined(IMPL_SV)
#include "top_tdm_sv.hpp"
#else
#include "top_tdm.hpp"
#endif
#endif

template <int NUM_BANK = 32, int NUM_ROW = 1024, int BYTES_PER_WORD = 4, int WORDS_PER_ROW = 4>
SC_MODULE(top) {
    static constexpr int NUM_REQ = 4;

    // RAGU driver group counts
    static constexpr int NUM_RAGU_A = 4;
    static constexpr int NUM_RAGU_B = 2;
    static constexpr int NUM_RAGU_C = 1;
    static constexpr int NUM_RAGU_D = 1;
    static constexpr int NUM_RAGU_E = 1;
    static constexpr int NUM_RPORT =
        NUM_RAGU_A + NUM_RAGU_B + NUM_RAGU_C + NUM_RAGU_D + NUM_RAGU_E; // 9
    static constexpr int NUM_RPORT_FLAT = NUM_RPORT * NUM_REQ;          // 36

    // Flat OBI port counts per RAGU driver (driver_count × NUM_REQ)
    static constexpr int RAGU_A_PORTS = NUM_RAGU_A * NUM_REQ; // 16
    static constexpr int RAGU_B_PORTS = NUM_RAGU_B * NUM_REQ; // 8
    static constexpr int RAGU_C_PORTS = NUM_RAGU_C * NUM_REQ; // 4
    static constexpr int RAGU_D_PORTS = NUM_RAGU_D * NUM_REQ; // 4
    static constexpr int RAGU_E_PORTS = NUM_RAGU_E * NUM_REQ; // 4
    // aliases for testbench access (RPORT_* = RAGU_*)
    static constexpr int RPORT_A_PORTS   = RAGU_A_PORTS;
    static constexpr int RPORT_B_PORTS   = RAGU_B_PORTS;
    static constexpr int RPORT_C_PORTS   = RAGU_C_PORTS;
    static constexpr int RPORT_D_PORTS   = RAGU_D_PORTS;
    static constexpr int RPORT_DMA_PORTS = RAGU_E_PORTS;

    // WAGU driver group counts
    static constexpr int NUM_WAGU_A     = 4;
    static constexpr int NUM_WAGU_B     = 2;
    static constexpr int NUM_WAGU_D     = 1;
    static constexpr int NUM_WAGU_E     = 1;
    static constexpr int NUM_WPORT      = NUM_WAGU_A + NUM_WAGU_B + NUM_WAGU_D + NUM_WAGU_E; // 8
    static constexpr int NUM_WPORT_FLAT = NUM_WPORT * NUM_REQ;                               // 32

    static constexpr int WAGU_A_PORTS = NUM_WAGU_A * NUM_REQ; // 16
    static constexpr int WAGU_B_PORTS = NUM_WAGU_B * NUM_REQ; // 8
    static constexpr int WAGU_D_PORTS = NUM_WAGU_D * NUM_REQ; // 4
    static constexpr int WAGU_E_PORTS = NUM_WAGU_E * NUM_REQ; // 4
    // aliases for testbench access (WPORT_* = WAGU_*)
    static constexpr int WPORT_A_PORTS   = WAGU_A_PORTS;
    static constexpr int WPORT_B_PORTS   = WAGU_B_PORTS;
    static constexpr int WPORT_D_PORTS   = WAGU_D_PORTS;
    static constexpr int WPORT_DMA_PORTS = WAGU_E_PORTS;

    static constexpr int BYTES_PER_ROW = WORDS_PER_ROW * BYTES_PER_WORD;
    using data_t                       = obi_data<BYTES_PER_ROW>;

#if defined(IMPL_SV)
    using impl_data_t = uint64_t;
#else
    using impl_data_t = data_t;
#endif

    // Clock/reset
    sc_in<bool> clk_i;
    sc_in<bool> rst_ni;

    // RAGU driver ports: RAGU_X_PORTS = NUM_RAGU_X * NUM_REQ flat OBI buses each.
    obi_subordinate_ports<data_t> ragu_a[RAGU_A_PORTS];
    obi_subordinate_ports<data_t> ragu_b[RAGU_B_PORTS];
    obi_subordinate_ports<data_t> ragu_c[RAGU_C_PORTS];
    obi_subordinate_ports<data_t> ragu_d[RAGU_D_PORTS];
    obi_subordinate_ports<data_t> ragu_e[RAGU_E_PORTS];

    // WAGU driver ports
    obi_subordinate_ports<data_t> wagu_a[WAGU_A_PORTS];
    obi_subordinate_ports<data_t> wagu_b[WAGU_B_PORTS];
    obi_subordinate_ports<data_t> wagu_d[WAGU_D_PORTS];
    obi_subordinate_ports<data_t> wagu_e[WAGU_E_PORTS];

    // Internal flat signals: NUM_RPORT_FLAT = NUM_RPORT * NUM_REQ = 36
    obi_signal_bundle<impl_data_t> impl_rport[NUM_RPORT_FLAT];

    // Internal flat signals: NUM_WPORT_FLAT = NUM_WPORT * NUM_REQ = 32
    obi_signal_bundle<impl_data_t> impl_wport[NUM_WPORT_FLAT];

#if defined(IMPL_CROSSBAR)
#if defined(IMPL_SV)
    // SV_NUM_REQ=4 matches NUM_REQ above.
    top_crossbar_sv<NUM_RPORT, NUM_WPORT, NUM_REQ, NUM_BANK, NUM_ROW, BYTES_PER_WORD, WORDS_PER_ROW,
                    4>
        impl;
#else
    top_crossbar<NUM_RPORT, NUM_WPORT, NUM_REQ, NUM_BANK, NUM_ROW, BYTES_PER_WORD, WORDS_PER_ROW>
        impl;
#if defined(XBAR_HASH_DYNAMIC) || defined(XBAR_HASH16) || defined(XBAR_HASH32) || \
    defined(XBAR_HASH_L1_V2) || defined(XBAR_HASH_L1_V3)
    // Per-port-group mapping geometry for top_crossbar.hpp's dynamic-hash
    // experiment — one scalar set per read/write driver group (see
    // top_crossbar.hpp's rport_map_r_i/etc). Written by the testbench from
    // each AGU's current p_R_/p_C_/p_L_/p_store_mode_.
    sc_signal<uint64_t> impl_rport_map_r[NUM_RPORT];
    sc_signal<uint64_t> impl_rport_map_c[NUM_RPORT];
    sc_signal<uint64_t> impl_rport_map_l[NUM_RPORT];
    sc_signal<uint64_t> impl_rport_map_store_mode[NUM_RPORT];
    sc_signal<uint64_t> impl_wport_map_r[NUM_WPORT];
    sc_signal<uint64_t> impl_wport_map_c[NUM_WPORT];
    sc_signal<uint64_t> impl_wport_map_l[NUM_WPORT];
    sc_signal<uint64_t> impl_wport_map_store_mode[NUM_WPORT];
#endif
#if defined(XBAR_HASH_L1_V2)
    // Actual napa integer (ports_used_) driving XBAR_HASH_L1_V2's
    // R/C/napa-keyed fold-length rule — see top_crossbar.hpp's
    // rport_map_napa_i and addr_hash.hpp's addr_hash_l1_v2().
    sc_signal<uint64_t> impl_rport_map_napa[NUM_RPORT];
    sc_signal<uint64_t> impl_wport_map_napa[NUM_WPORT];
#endif
#if defined(XBAR_HASH16)
    // Fixed per-AGU "high bank half" selector — see top_crossbar.hpp's
    // rport_map_hi_bank_i.
    sc_signal<bool> impl_rport_map_hi_bank[NUM_RPORT];
    sc_signal<bool> impl_wport_map_hi_bank[NUM_WPORT];
#endif
#endif
#elif defined(IMPL_TDM)
#if defined(IMPL_SV)
    top_tdm_sv<NUM_RPORT, NUM_WPORT, NUM_REQ, NUM_BANK, NUM_ROW, BYTES_PER_WORD, WORDS_PER_ROW>
        impl;
#else
    top_tdm<NUM_RPORT, NUM_WPORT, NUM_REQ, NUM_BANK, NUM_ROW, BYTES_PER_WORD, WORDS_PER_ROW> impl;
    // Active-mode signals: one per buffer (r0..r4=0..4, w0..w3=5..8).
    // Written by the testbench after trace loading (from stimuli ports_used_groups).
    static constexpr int NUM_TDM_BUF = 9;
    sc_signal<uint32_t>  impl_buf_active_mode[NUM_TDM_BUF];
    sc_signal<uint64_t>  impl_buf_map_r[NUM_TDM_BUF];
    sc_signal<uint64_t>  impl_buf_map_c[NUM_TDM_BUF];
    sc_signal<uint64_t>  impl_buf_map_l[NUM_TDM_BUF];
    sc_signal<uint64_t>  impl_buf_map_store_mode[NUM_TDM_BUF];
#endif
#endif

    static impl_data_t to_impl_data(const data_t &data) {
#if defined(IMPL_SV)
        return data.to_uint64();
#else
        return data;
#endif
    }

    static data_t from_impl_data(const impl_data_t &data) {
#if defined(IMPL_SV)
        return data_t(static_cast<unsigned long long>(data));
#else
        return data;
#endif
    }

    // dst is impl_rport or impl_wport — the only difference between the
    // former rport/wport-specific pack/unpack pairs.
    template <int N>
    void pack_port_group(int base, obi_subordinate_ports<data_t>(&group)[N],
                         obi_signal_bundle<impl_data_t> *dst) {
        for (int i = 0; i < N; ++i) {
            dst[base + i].req.write(group[i].req_i.read());
            dst[base + i].addr.write(group[i].addr_i.read());
            dst[base + i].we.write(group[i].we_i.read());
            dst[base + i].be.write(group[i].be_i.read());
            dst[base + i].wdata.write(to_impl_data(group[i].wdata_i.read()));
        }
    }

    template <int N>
    void unpack_port_group(int base, obi_subordinate_ports<data_t>(&group)[N],
                           obi_signal_bundle<impl_data_t> *dst) {
        for (int i = 0; i < N; ++i) {
            group[i].gnt_o.write(dst[base + i].gnt.read());
            group[i].rvalid_o.write(dst[base + i].rvalid.read());
            group[i].rdata_o.write(from_impl_data(dst[base + i].rdata.read()));
        }
    }

    void pack_rport_inputs() {
        // Base = driver group offset × NUM_REQ
        pack_port_group(0 * NUM_REQ, ragu_a, impl_rport);
        pack_port_group(4 * NUM_REQ, ragu_b, impl_rport);
        pack_port_group(6 * NUM_REQ, ragu_c, impl_rport);
        pack_port_group(7 * NUM_REQ, ragu_d, impl_rport);
        pack_port_group(8 * NUM_REQ, ragu_e, impl_rport);
    }

    void unpack_rport_outputs() {
        unpack_port_group(0 * NUM_REQ, ragu_a, impl_rport);
        unpack_port_group(4 * NUM_REQ, ragu_b, impl_rport);
        unpack_port_group(6 * NUM_REQ, ragu_c, impl_rport);
        unpack_port_group(7 * NUM_REQ, ragu_d, impl_rport);
        unpack_port_group(8 * NUM_REQ, ragu_e, impl_rport);
    }

    void pack_wport_inputs() {
        pack_port_group(0 * NUM_REQ, wagu_a, impl_wport);
        pack_port_group(4 * NUM_REQ, wagu_b, impl_wport);
        pack_port_group(6 * NUM_REQ, wagu_d, impl_wport);
        pack_port_group(7 * NUM_REQ, wagu_e, impl_wport);
    }

    void unpack_wport_outputs() {
        unpack_port_group(0 * NUM_REQ, wagu_a, impl_wport);
        unpack_port_group(4 * NUM_REQ, wagu_b, impl_wport);
        unpack_port_group(6 * NUM_REQ, wagu_d, impl_wport);
        unpack_port_group(7 * NUM_REQ, wagu_e, impl_wport);
    }

    template <int N> void add_input_sensitivity(obi_subordinate_ports<data_t>(&group)[N]) {
        for (int i = 0; i < N; ++i)
            sensitive << group[i].req_i << group[i].addr_i << group[i].we_i << group[i].be_i
                      << group[i].wdata_i;
    }

    SC_CTOR(top) : impl("impl") {
        impl.clk_i(clk_i);
        impl.rst_ni(rst_ni);

        bind_obi_group(impl.rport_req_i, impl.rport_addr_i, impl.rport_we_i, impl.rport_be_i,
                       impl.rport_wdata_i, impl.rport_gnt_o, impl.rport_rvalid_o,
                       impl.rport_rdata_o, impl_rport);
        bind_obi_group(impl.wport_req_i, impl.wport_addr_i, impl.wport_we_i, impl.wport_be_i,
                       impl.wport_wdata_i, impl.wport_gnt_o, impl.wport_rvalid_o,
                       impl.wport_rdata_o, impl_wport);

#if defined(IMPL_CROSSBAR) && !defined(IMPL_SV) &&                                                \
    (defined(XBAR_HASH_DYNAMIC) || defined(XBAR_HASH16) || defined(XBAR_HASH32) ||                \
     defined(XBAR_HASH_L1_V2) || defined(XBAR_HASH_L1_V3))
        for (int i = 0; i < NUM_RPORT; ++i) {
            impl.rport_map_r_i[i](impl_rport_map_r[i]);
            impl.rport_map_c_i[i](impl_rport_map_c[i]);
            impl.rport_map_l_i[i](impl_rport_map_l[i]);
            impl.rport_map_store_mode_i[i](impl_rport_map_store_mode[i]);
        }
        for (int i = 0; i < NUM_WPORT; ++i) {
            impl.wport_map_r_i[i](impl_wport_map_r[i]);
            impl.wport_map_c_i[i](impl_wport_map_c[i]);
            impl.wport_map_l_i[i](impl_wport_map_l[i]);
            impl.wport_map_store_mode_i[i](impl_wport_map_store_mode[i]);
        }
#endif
#if defined(IMPL_CROSSBAR) && !defined(IMPL_SV) && defined(XBAR_HASH_L1_V2)
        for (int i = 0; i < NUM_RPORT; ++i)
            impl.rport_map_napa_i[i](impl_rport_map_napa[i]);
        for (int i = 0; i < NUM_WPORT; ++i)
            impl.wport_map_napa_i[i](impl_wport_map_napa[i]);
#endif
#if defined(IMPL_CROSSBAR) && !defined(IMPL_SV) && defined(XBAR_HASH16)
        for (int i = 0; i < NUM_RPORT; ++i)
            impl.rport_map_hi_bank_i[i](impl_rport_map_hi_bank[i]);
        for (int i = 0; i < NUM_WPORT; ++i)
            impl.wport_map_hi_bank_i[i](impl_wport_map_hi_bank[i]);
#endif

#if defined(IMPL_TDM) && !defined(IMPL_SV)
        for (int i = 0; i < NUM_TDM_BUF; ++i)
            impl.buf_active_mode_i[i](impl_buf_active_mode[i]);
        for (int i = 0; i < NUM_TDM_BUF; ++i) {
            impl.buf_map_r_i[i](impl_buf_map_r[i]);
            impl.buf_map_c_i[i](impl_buf_map_c[i]);
            impl.buf_map_l_i[i](impl_buf_map_l[i]);
            impl.buf_map_store_mode_i[i](impl_buf_map_store_mode[i]);
        }
#endif

        SC_METHOD(pack_rport_inputs);
        add_input_sensitivity(ragu_a);
        add_input_sensitivity(ragu_b);
        add_input_sensitivity(ragu_c);
        add_input_sensitivity(ragu_d);
        add_input_sensitivity(ragu_e);

        SC_METHOD(unpack_rport_outputs);
        for (int i = 0; i < NUM_RPORT_FLAT; ++i)
            sensitive << impl_rport[i].gnt << impl_rport[i].rvalid << impl_rport[i].rdata;

        SC_METHOD(pack_wport_inputs);
        add_input_sensitivity(wagu_a);
        add_input_sensitivity(wagu_b);
        add_input_sensitivity(wagu_d);
        add_input_sensitivity(wagu_e);

        SC_METHOD(unpack_wport_outputs);
        for (int i = 0; i < NUM_WPORT_FLAT; ++i)
            sensitive << impl_wport[i].gnt << impl_wport[i].rvalid << impl_wport[i].rdata;
    }
};

#endif // TOP_HPP
