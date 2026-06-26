// -----------------------------------------------------------------------------
// Author: Cedric Hoelzl
//
// Description:
//   Unified SystemC DUT wrapper for the TDM project.  Pass IMPL=<value>
//   to select an implementation; run.sh uppercases it to -DIMPL_<UPPER>.
//   top_* implementations use the matching SV file as Verilator --top-module; native
//   crossbar/tdm implementations build directly with g++.
//   Recognised IMPL values:
//     crossbar       IMPL=crossbar       → native SC crossbar
//     top_crossbar   IMPL=top_crossbar   → Verilated SV crossbar
//     tdm            IMPL=tdm            → native SC TDM
//     top_tdm        IMPL=top_tdm        → Verilated SV TDM
//
//   IMPL=tdm_sc is kept as a compatibility alias for the native SC TDM.
//
//   The wrapper packs the RAGU/WAGU driver port groups onto the flat port arrays
//   used by the implementation tops:
//     read : 0..3 RAGU_A, 4..5 RAGU_B, 6 RAGU_C, 7 RAGU_D, 8 RAGU_DMA
//     write: 0..3 WAGU_A, 4..5 WAGU_B, 6 WAGU_D, 7 WAGU_DMA
//
//   NUM_REQ sub-ports are exposed per driver, so RAGU_A_PORTS =
//   NUM_RAGU_A * NUM_REQ (e.g. 4*4=16 flat OBI connections for RAGU_A).
//   The crossbar receives NUM_RPORT=9 / NUM_WPORT=8 groups with NUM_REQ sub-ports
//   each (36 and 32 flat internal ports respectively).
// -----------------------------------------------------------------------------

#ifndef TOP_HPP
#define TOP_HPP

#include "obi_data.hpp"
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
    static constexpr int NUM_RAGU_A   = 4;
    static constexpr int NUM_RAGU_B   = 2;
    static constexpr int NUM_RAGU_C   = 1;
    static constexpr int NUM_RAGU_D   = 1;
    static constexpr int NUM_RAGU_DMA = 1;
    static constexpr int NUM_RPORT =
        NUM_RAGU_A + NUM_RAGU_B + NUM_RAGU_C + NUM_RAGU_D + NUM_RAGU_DMA; // 9
    static constexpr int NUM_RPORT_FLAT = NUM_RPORT * NUM_REQ;            // 36

    // Flat OBI port counts per RAGU driver (driver_count × NUM_REQ)
    static constexpr int RAGU_A_PORTS   = NUM_RAGU_A * NUM_REQ;   // 16
    static constexpr int RAGU_B_PORTS   = NUM_RAGU_B * NUM_REQ;   // 8
    static constexpr int RAGU_C_PORTS   = NUM_RAGU_C * NUM_REQ;   // 4
    static constexpr int RAGU_D_PORTS   = NUM_RAGU_D * NUM_REQ;   // 4
    static constexpr int RAGU_DMA_PORTS = NUM_RAGU_DMA * NUM_REQ; // 4
    // aliases for testbench access (RPORT_* = RAGU_*)
    static constexpr int RPORT_A_PORTS   = RAGU_A_PORTS;
    static constexpr int RPORT_B_PORTS   = RAGU_B_PORTS;
    static constexpr int RPORT_C_PORTS   = RAGU_C_PORTS;
    static constexpr int RPORT_D_PORTS   = RAGU_D_PORTS;
    static constexpr int RPORT_DMA_PORTS = RAGU_DMA_PORTS;

    // WAGU driver group counts
    static constexpr int NUM_WAGU_A     = 4;
    static constexpr int NUM_WAGU_B     = 2;
    static constexpr int NUM_WAGU_D     = 1;
    static constexpr int NUM_WAGU_DMA   = 1;
    static constexpr int NUM_WPORT      = NUM_WAGU_A + NUM_WAGU_B + NUM_WAGU_D + NUM_WAGU_DMA; // 8
    static constexpr int NUM_WPORT_FLAT = NUM_WPORT * NUM_REQ;                                 // 32

    static constexpr int WAGU_A_PORTS   = NUM_WAGU_A * NUM_REQ;   // 16
    static constexpr int WAGU_B_PORTS   = NUM_WAGU_B * NUM_REQ;   // 8
    static constexpr int WAGU_D_PORTS   = NUM_WAGU_D * NUM_REQ;   // 4
    static constexpr int WAGU_DMA_PORTS = NUM_WAGU_DMA * NUM_REQ; // 4
    // aliases for testbench access (WPORT_* = WAGU_*)
    static constexpr int WPORT_A_PORTS   = WAGU_A_PORTS;
    static constexpr int WPORT_B_PORTS   = WAGU_B_PORTS;
    static constexpr int WPORT_D_PORTS   = WAGU_D_PORTS;
    static constexpr int WPORT_DMA_PORTS = WAGU_DMA_PORTS;

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
    sc_in<bool>     ragu_a_req_i[RAGU_A_PORTS];
    sc_in<uint64_t> ragu_a_addr_i[RAGU_A_PORTS];
    sc_in<bool>     ragu_a_we_i[RAGU_A_PORTS];
    sc_in<uint32_t> ragu_a_be_i[RAGU_A_PORTS];
    sc_in<data_t>   ragu_a_wdata_i[RAGU_A_PORTS];
    sc_out<bool>    ragu_a_gnt_o[RAGU_A_PORTS];
    sc_out<bool>    ragu_a_rvalid_o[RAGU_A_PORTS];
    sc_out<data_t>  ragu_a_rdata_o[RAGU_A_PORTS];

    sc_in<bool>     ragu_b_req_i[RAGU_B_PORTS];
    sc_in<uint64_t> ragu_b_addr_i[RAGU_B_PORTS];
    sc_in<bool>     ragu_b_we_i[RAGU_B_PORTS];
    sc_in<uint32_t> ragu_b_be_i[RAGU_B_PORTS];
    sc_in<data_t>   ragu_b_wdata_i[RAGU_B_PORTS];
    sc_out<bool>    ragu_b_gnt_o[RAGU_B_PORTS];
    sc_out<bool>    ragu_b_rvalid_o[RAGU_B_PORTS];
    sc_out<data_t>  ragu_b_rdata_o[RAGU_B_PORTS];

    sc_in<bool>     ragu_c_req_i[RAGU_C_PORTS];
    sc_in<uint64_t> ragu_c_addr_i[RAGU_C_PORTS];
    sc_in<bool>     ragu_c_we_i[RAGU_C_PORTS];
    sc_in<uint32_t> ragu_c_be_i[RAGU_C_PORTS];
    sc_in<data_t>   ragu_c_wdata_i[RAGU_C_PORTS];
    sc_out<bool>    ragu_c_gnt_o[RAGU_C_PORTS];
    sc_out<bool>    ragu_c_rvalid_o[RAGU_C_PORTS];
    sc_out<data_t>  ragu_c_rdata_o[RAGU_C_PORTS];

    sc_in<bool>     ragu_d_req_i[RAGU_D_PORTS];
    sc_in<uint64_t> ragu_d_addr_i[RAGU_D_PORTS];
    sc_in<bool>     ragu_d_we_i[RAGU_D_PORTS];
    sc_in<uint32_t> ragu_d_be_i[RAGU_D_PORTS];
    sc_in<data_t>   ragu_d_wdata_i[RAGU_D_PORTS];
    sc_out<bool>    ragu_d_gnt_o[RAGU_D_PORTS];
    sc_out<bool>    ragu_d_rvalid_o[RAGU_D_PORTS];
    sc_out<data_t>  ragu_d_rdata_o[RAGU_D_PORTS];

    sc_in<bool>     ragu_dma_req_i[RAGU_DMA_PORTS];
    sc_in<uint64_t> ragu_dma_addr_i[RAGU_DMA_PORTS];
    sc_in<bool>     ragu_dma_we_i[RAGU_DMA_PORTS];
    sc_in<uint32_t> ragu_dma_be_i[RAGU_DMA_PORTS];
    sc_in<data_t>   ragu_dma_wdata_i[RAGU_DMA_PORTS];
    sc_out<bool>    ragu_dma_gnt_o[RAGU_DMA_PORTS];
    sc_out<bool>    ragu_dma_rvalid_o[RAGU_DMA_PORTS];
    sc_out<data_t>  ragu_dma_rdata_o[RAGU_DMA_PORTS];

    // WAGU driver ports
    sc_in<bool>     wagu_a_req_i[WAGU_A_PORTS];
    sc_in<uint64_t> wagu_a_addr_i[WAGU_A_PORTS];
    sc_in<bool>     wagu_a_we_i[WAGU_A_PORTS];
    sc_in<uint32_t> wagu_a_be_i[WAGU_A_PORTS];
    sc_in<data_t>   wagu_a_wdata_i[WAGU_A_PORTS];
    sc_out<bool>    wagu_a_gnt_o[WAGU_A_PORTS];
    sc_out<bool>    wagu_a_rvalid_o[WAGU_A_PORTS];
    sc_out<data_t>  wagu_a_rdata_o[WAGU_A_PORTS];

    sc_in<bool>     wagu_b_req_i[WAGU_B_PORTS];
    sc_in<uint64_t> wagu_b_addr_i[WAGU_B_PORTS];
    sc_in<bool>     wagu_b_we_i[WAGU_B_PORTS];
    sc_in<uint32_t> wagu_b_be_i[WAGU_B_PORTS];
    sc_in<data_t>   wagu_b_wdata_i[WAGU_B_PORTS];
    sc_out<bool>    wagu_b_gnt_o[WAGU_B_PORTS];
    sc_out<bool>    wagu_b_rvalid_o[WAGU_B_PORTS];
    sc_out<data_t>  wagu_b_rdata_o[WAGU_B_PORTS];

    sc_in<bool>     wagu_d_req_i[WAGU_D_PORTS];
    sc_in<uint64_t> wagu_d_addr_i[WAGU_D_PORTS];
    sc_in<bool>     wagu_d_we_i[WAGU_D_PORTS];
    sc_in<uint32_t> wagu_d_be_i[WAGU_D_PORTS];
    sc_in<data_t>   wagu_d_wdata_i[WAGU_D_PORTS];
    sc_out<bool>    wagu_d_gnt_o[WAGU_D_PORTS];
    sc_out<bool>    wagu_d_rvalid_o[WAGU_D_PORTS];
    sc_out<data_t>  wagu_d_rdata_o[WAGU_D_PORTS];

    sc_in<bool>     wagu_dma_req_i[WAGU_DMA_PORTS];
    sc_in<uint64_t> wagu_dma_addr_i[WAGU_DMA_PORTS];
    sc_in<bool>     wagu_dma_we_i[WAGU_DMA_PORTS];
    sc_in<uint32_t> wagu_dma_be_i[WAGU_DMA_PORTS];
    sc_in<data_t>   wagu_dma_wdata_i[WAGU_DMA_PORTS];
    sc_out<bool>    wagu_dma_gnt_o[WAGU_DMA_PORTS];
    sc_out<bool>    wagu_dma_rvalid_o[WAGU_DMA_PORTS];
    sc_out<data_t>  wagu_dma_rdata_o[WAGU_DMA_PORTS];

    // Internal flat signals: NUM_RPORT_FLAT = NUM_RPORT * NUM_REQ = 36

    sc_signal<bool>        impl_rport_req[NUM_RPORT_FLAT];
    sc_signal<uint64_t>    impl_rport_addr[NUM_RPORT_FLAT];
    sc_signal<bool>        impl_rport_we[NUM_RPORT_FLAT];
    sc_signal<uint32_t>    impl_rport_be[NUM_RPORT_FLAT];
    sc_signal<impl_data_t> impl_rport_wdata[NUM_RPORT_FLAT];
    sc_signal<bool>        impl_rport_gnt[NUM_RPORT_FLAT];
    sc_signal<bool>        impl_rport_rvalid[NUM_RPORT_FLAT];
    sc_signal<impl_data_t> impl_rport_rdata[NUM_RPORT_FLAT];

    // Internal flat signals: NUM_WPORT_FLAT = NUM_WPORT * NUM_REQ = 32
    sc_signal<bool>        impl_wport_req[NUM_WPORT_FLAT];
    sc_signal<uint64_t>    impl_wport_addr[NUM_WPORT_FLAT];
    sc_signal<bool>        impl_wport_we[NUM_WPORT_FLAT];
    sc_signal<uint32_t>    impl_wport_be[NUM_WPORT_FLAT];
    sc_signal<impl_data_t> impl_wport_wdata[NUM_WPORT_FLAT];
    sc_signal<bool>        impl_wport_gnt[NUM_WPORT_FLAT];
    sc_signal<bool>        impl_wport_rvalid[NUM_WPORT_FLAT];
    sc_signal<impl_data_t> impl_wport_rdata[NUM_WPORT_FLAT];

#if defined(IMPL_CROSSBAR)
#if defined(IMPL_SV)
    // SV_NUM_REQ=4 matches NUM_REQ above.
    top_crossbar_sv<NUM_RPORT, NUM_WPORT, NUM_REQ, NUM_BANK, NUM_ROW, BYTES_PER_WORD, WORDS_PER_ROW,
                    4>
        impl;
#else
    top_crossbar<NUM_RPORT, NUM_WPORT, NUM_REQ, NUM_BANK, NUM_ROW, BYTES_PER_WORD, WORDS_PER_ROW>
        impl;
#endif
#elif defined(IMPL_TDM)
#if defined(IMPL_SV)
    top_tdm_sv<NUM_RPORT, NUM_WPORT, NUM_REQ, NUM_BANK, NUM_ROW, BYTES_PER_WORD, WORDS_PER_ROW>
        impl;
#else
    top_tdm<NUM_RPORT, NUM_WPORT, NUM_REQ, NUM_BANK, NUM_ROW, BYTES_PER_WORD, WORDS_PER_ROW> impl;
    // Active-mode signals: one per buffer (r0..r4=0..4, w0..w3=5..8).
    // Written by the testbench after trace loading (from stimuli ports_used_groups).
    static constexpr int  NUM_TDM_BUF = 9;
    sc_signal<uint32_t>   impl_buf_active_mode[NUM_TDM_BUF];
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

    template <int N>
    void pack_rport_group(int base, sc_in<bool>(&req)[N], sc_in<uint64_t>(&addr)[N],
                          sc_in<bool>(&we)[N], sc_in<uint32_t>(&be)[N], sc_in<data_t>(&wdata)[N]) {
        for (int i = 0; i < N; ++i) {
            impl_rport_req[base + i].write(req[i].read());
            impl_rport_addr[base + i].write(addr[i].read());
            impl_rport_we[base + i].write(we[i].read());
            impl_rport_be[base + i].write(be[i].read());
            impl_rport_wdata[base + i].write(to_impl_data(wdata[i].read()));
        }
    }

    template <int N>
    void unpack_rport_group(int base, sc_out<bool>(&gnt)[N], sc_out<bool>(&rvalid)[N],
                            sc_out<data_t>(&rdata)[N]) {
        for (int i = 0; i < N; ++i) {
            gnt[i].write(impl_rport_gnt[base + i].read());
            rvalid[i].write(impl_rport_rvalid[base + i].read());
            rdata[i].write(from_impl_data(impl_rport_rdata[base + i].read()));
        }
    }

    template <int N>
    void pack_wport_group(int base, sc_in<bool>(&req)[N], sc_in<uint64_t>(&addr)[N],
                          sc_in<bool>(&we)[N], sc_in<uint32_t>(&be)[N], sc_in<data_t>(&wdata)[N]) {
        for (int i = 0; i < N; ++i) {
            impl_wport_req[base + i].write(req[i].read());
            impl_wport_addr[base + i].write(addr[i].read());
            impl_wport_we[base + i].write(we[i].read());
            impl_wport_be[base + i].write(be[i].read());
            impl_wport_wdata[base + i].write(to_impl_data(wdata[i].read()));
        }
    }

    template <int N>
    void unpack_wport_group(int base, sc_out<bool>(&gnt)[N], sc_out<bool>(&rvalid)[N],
                            sc_out<data_t>(&rdata)[N]) {
        for (int i = 0; i < N; ++i) {
            gnt[i].write(impl_wport_gnt[base + i].read());
            rvalid[i].write(impl_wport_rvalid[base + i].read());
            rdata[i].write(from_impl_data(impl_wport_rdata[base + i].read()));
        }
    }

    void pack_rport_inputs() {
        // Base = driver group offset × NUM_REQ
        pack_rport_group(0 * NUM_REQ, ragu_a_req_i, ragu_a_addr_i, ragu_a_we_i, ragu_a_be_i,
                         ragu_a_wdata_i);
        pack_rport_group(4 * NUM_REQ, ragu_b_req_i, ragu_b_addr_i, ragu_b_we_i, ragu_b_be_i,
                         ragu_b_wdata_i);
        pack_rport_group(6 * NUM_REQ, ragu_c_req_i, ragu_c_addr_i, ragu_c_we_i, ragu_c_be_i,
                         ragu_c_wdata_i);
        pack_rport_group(7 * NUM_REQ, ragu_d_req_i, ragu_d_addr_i, ragu_d_we_i, ragu_d_be_i,
                         ragu_d_wdata_i);
        pack_rport_group(8 * NUM_REQ, ragu_dma_req_i, ragu_dma_addr_i, ragu_dma_we_i, ragu_dma_be_i,
                         ragu_dma_wdata_i);
    }

    void unpack_rport_outputs() {
        unpack_rport_group(0 * NUM_REQ, ragu_a_gnt_o, ragu_a_rvalid_o, ragu_a_rdata_o);
        unpack_rport_group(4 * NUM_REQ, ragu_b_gnt_o, ragu_b_rvalid_o, ragu_b_rdata_o);
        unpack_rport_group(6 * NUM_REQ, ragu_c_gnt_o, ragu_c_rvalid_o, ragu_c_rdata_o);
        unpack_rport_group(7 * NUM_REQ, ragu_d_gnt_o, ragu_d_rvalid_o, ragu_d_rdata_o);
        unpack_rport_group(8 * NUM_REQ, ragu_dma_gnt_o, ragu_dma_rvalid_o, ragu_dma_rdata_o);
    }

    void pack_wport_inputs() {
        pack_wport_group(0 * NUM_REQ, wagu_a_req_i, wagu_a_addr_i, wagu_a_we_i, wagu_a_be_i,
                         wagu_a_wdata_i);
        pack_wport_group(4 * NUM_REQ, wagu_b_req_i, wagu_b_addr_i, wagu_b_we_i, wagu_b_be_i,
                         wagu_b_wdata_i);
        pack_wport_group(6 * NUM_REQ, wagu_d_req_i, wagu_d_addr_i, wagu_d_we_i, wagu_d_be_i,
                         wagu_d_wdata_i);
        pack_wport_group(7 * NUM_REQ, wagu_dma_req_i, wagu_dma_addr_i, wagu_dma_we_i, wagu_dma_be_i,
                         wagu_dma_wdata_i);
    }

    void unpack_wport_outputs() {
        unpack_wport_group(0 * NUM_REQ, wagu_a_gnt_o, wagu_a_rvalid_o, wagu_a_rdata_o);
        unpack_wport_group(4 * NUM_REQ, wagu_b_gnt_o, wagu_b_rvalid_o, wagu_b_rdata_o);
        unpack_wport_group(6 * NUM_REQ, wagu_d_gnt_o, wagu_d_rvalid_o, wagu_d_rdata_o);
        unpack_wport_group(7 * NUM_REQ, wagu_dma_gnt_o, wagu_dma_rvalid_o, wagu_dma_rdata_o);
    }

    template <int N>
    void add_input_sensitivity(sc_in<bool>(&req)[N], sc_in<uint64_t>(&addr)[N], sc_in<bool>(&we)[N],
                               sc_in<uint32_t>(&be)[N], sc_in<data_t>(&wdata)[N]) {
        for (int i = 0; i < N; ++i)
            sensitive << req[i] << addr[i] << we[i] << be[i] << wdata[i];
    }

    SC_CTOR(top) : impl("impl") {
        impl.clk_i(clk_i);
        impl.rst_ni(rst_ni);

        for (int i = 0; i < NUM_RPORT_FLAT; ++i) {
            impl.rport_req_i[i](impl_rport_req[i]);
            impl.rport_addr_i[i](impl_rport_addr[i]);
            impl.rport_we_i[i](impl_rport_we[i]);
            impl.rport_be_i[i](impl_rport_be[i]);
            impl.rport_wdata_i[i](impl_rport_wdata[i]);
            impl.rport_gnt_o[i](impl_rport_gnt[i]);
            impl.rport_rvalid_o[i](impl_rport_rvalid[i]);
            impl.rport_rdata_o[i](impl_rport_rdata[i]);
        }
        for (int i = 0; i < NUM_WPORT_FLAT; ++i) {
            impl.wport_req_i[i](impl_wport_req[i]);
            impl.wport_addr_i[i](impl_wport_addr[i]);
            impl.wport_we_i[i](impl_wport_we[i]);
            impl.wport_be_i[i](impl_wport_be[i]);
            impl.wport_wdata_i[i](impl_wport_wdata[i]);
            impl.wport_gnt_o[i](impl_wport_gnt[i]);
            impl.wport_rvalid_o[i](impl_wport_rvalid[i]);
            impl.wport_rdata_o[i](impl_wport_rdata[i]);
        }

#if defined(IMPL_TDM) && !defined(IMPL_SV)
        for (int i = 0; i < NUM_TDM_BUF; ++i)
            impl.buf_active_mode_i[i](impl_buf_active_mode[i]);
#endif

        SC_METHOD(pack_rport_inputs);
        add_input_sensitivity(ragu_a_req_i, ragu_a_addr_i, ragu_a_we_i, ragu_a_be_i,
                              ragu_a_wdata_i);
        add_input_sensitivity(ragu_b_req_i, ragu_b_addr_i, ragu_b_we_i, ragu_b_be_i,
                              ragu_b_wdata_i);
        add_input_sensitivity(ragu_c_req_i, ragu_c_addr_i, ragu_c_we_i, ragu_c_be_i,
                              ragu_c_wdata_i);
        add_input_sensitivity(ragu_d_req_i, ragu_d_addr_i, ragu_d_we_i, ragu_d_be_i,
                              ragu_d_wdata_i);
        add_input_sensitivity(ragu_dma_req_i, ragu_dma_addr_i, ragu_dma_we_i, ragu_dma_be_i,
                              ragu_dma_wdata_i);

        SC_METHOD(unpack_rport_outputs);
        for (int i = 0; i < NUM_RPORT_FLAT; ++i)
            sensitive << impl_rport_gnt[i] << impl_rport_rvalid[i] << impl_rport_rdata[i];

        SC_METHOD(pack_wport_inputs);
        add_input_sensitivity(wagu_a_req_i, wagu_a_addr_i, wagu_a_we_i, wagu_a_be_i,
                              wagu_a_wdata_i);
        add_input_sensitivity(wagu_b_req_i, wagu_b_addr_i, wagu_b_we_i, wagu_b_be_i,
                              wagu_b_wdata_i);
        add_input_sensitivity(wagu_d_req_i, wagu_d_addr_i, wagu_d_we_i, wagu_d_be_i,
                              wagu_d_wdata_i);
        add_input_sensitivity(wagu_dma_req_i, wagu_dma_addr_i, wagu_dma_we_i, wagu_dma_be_i,
                              wagu_dma_wdata_i);

        SC_METHOD(unpack_wport_outputs);
        for (int i = 0; i < NUM_WPORT_FLAT; ++i)
            sensitive << impl_wport_gnt[i] << impl_wport_rvalid[i] << impl_wport_rdata[i];
    }
};

#endif // TOP_HPP
