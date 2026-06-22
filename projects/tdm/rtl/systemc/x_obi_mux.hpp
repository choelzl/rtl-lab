// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   Native SystemC x-OBI channel mux/demux (see doc/specs/x_obi.md). It selects
//   one of NUM_AGU input x-OBI channels (each NUM_WORD words wide) onto a single
//   output channel:
//     - Request / forward path (steered by sel_req_i): the selected channel's
//       scalar addr/we/be and per-word req/wdata are driven onto the output, and
//       the output's per-word gnt is returned to that channel; non-selected
//       channels see gnt = 0.
//     - Response / return path (steered by sel_rsp_i): the output's per-word
//       rvalid/rdata are routed back to the channel chosen by sel_rsp_i. It is a
//       separate select so a response that lags its request by one cycle can be
//       steered with the earlier selection.
//   Purely combinational, no state. The per-word fields of channel i occupy the
//   flattened index range [i*NUM_WORD .. i*NUM_WORD + NUM_WORD); the scalar
//   fields (addr/we/be) are indexed by channel.
//
//   x-OBI signal set (see doc/specs/x_obi.md):
//     request  : req[N], wdata[N], addr (base), we, be
//     response : gnt[N], rvalid[N], rdata[N]
//
// Template parameters:
//   NUM_AGU  - number of input x-OBI channels to select among (default 2)
//   NUM_WORD - words per channel (default 8)
// -----------------------------------------------------------------------------

#ifndef X_OBI_MUX_HPP
#define X_OBI_MUX_HPP

#include <systemc.h>

#include <cstdint>

template <int NUM_AGU = 2, int NUM_WORD = 8> SC_MODULE(x_obi_mux) {
    static constexpr int NUM_IN = NUM_AGU * NUM_WORD;

    static_assert(NUM_AGU >= 1, "NUM_AGU must be >= 1");
    static_assert(NUM_WORD >= 1, "NUM_WORD must be >= 1");

    sc_in<int> sel_req_i;
    sc_in<int> sel_rsp_i;

    sc_in<bool>      m_req_i[NUM_IN];
    sc_in<uint64_t>  m_wdata_i[NUM_IN];
    sc_in<uint64_t>  m_addr_i[NUM_AGU];
    sc_in<bool>      m_we_i[NUM_AGU];
    sc_in<uint32_t>  m_be_i[NUM_AGU];
    sc_out<bool>     m_gnt_o[NUM_IN];
    sc_out<bool>     m_rvalid_o[NUM_IN];
    sc_out<uint64_t> m_rdata_o[NUM_IN];

    sc_out<bool>     s_req_o[NUM_WORD];
    sc_out<uint64_t> s_wdata_o[NUM_WORD];
    sc_out<uint64_t> s_addr_o;
    sc_out<bool>     s_we_o;
    sc_out<uint32_t> s_be_o;
    sc_in<bool>      s_gnt_i[NUM_WORD];
    sc_in<bool>      s_rvalid_i[NUM_WORD];
    sc_in<uint64_t>  s_rdata_i[NUM_WORD];

    void comb() {
        const int sr = sel_req_i.read();
        const int sp = sel_rsp_i.read();

        for (int w = 0; w < NUM_WORD; ++w) {
            s_req_o[w].write(false);
            s_wdata_o[w].write(0);
        }
        s_addr_o.write(0);
        s_we_o.write(false);
        s_be_o.write(0);
        for (int i = 0; i < NUM_IN; ++i) {
            m_gnt_o[i].write(false);
            m_rvalid_o[i].write(false);
            m_rdata_o[i].write(0);
        }

        if (sr >= 0 && sr < NUM_AGU) {
            s_addr_o.write(m_addr_i[sr].read());
            s_we_o.write(m_we_i[sr].read());
            s_be_o.write(m_be_i[sr].read());
            for (int w = 0; w < NUM_WORD; ++w) {
                const int idx = sr * NUM_WORD + w;
                s_req_o[w].write(m_req_i[idx].read());
                s_wdata_o[w].write(m_wdata_i[idx].read());
                m_gnt_o[idx].write(s_gnt_i[w].read());
            }
        }

        if (sp >= 0 && sp < NUM_AGU) {
            for (int w = 0; w < NUM_WORD; ++w) {
                const int idx = sp * NUM_WORD + w;
                m_rvalid_o[idx].write(s_rvalid_i[w].read());
                m_rdata_o[idx].write(s_rdata_i[w].read());
            }
        }
    }

    SC_CTOR(x_obi_mux) {
        SC_METHOD(comb);
        for (int i = 0; i < NUM_IN; ++i)
            sensitive << m_req_i[i] << m_wdata_i[i];
        for (int a = 0; a < NUM_AGU; ++a)
            sensitive << m_addr_i[a] << m_we_i[a] << m_be_i[a];
        for (int w = 0; w < NUM_WORD; ++w)
            sensitive << s_gnt_i[w] << s_rvalid_i[w] << s_rdata_i[w];
        sensitive << sel_req_i << sel_rsp_i;
    }
};

#endif
