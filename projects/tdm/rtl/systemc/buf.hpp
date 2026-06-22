// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   Native SystemC per-AGU buffer: an x-OBI width converter (NUM_REQ-wide AGU
//   side <-> NUM_WORD-wide memory side, NUM_WORD a multiple of NUM_REQ, see
//   doc/specs/x_obi.md). Every GROUPS = NUM_WORD/NUM_REQ consecutive AGU
//   sub-requests form one NUM_WORD-word memory group.
//
//   Three-stage pipeline (FILL -> MEM -> output FIFO) with read prefetch:
//     - FILL : forms the next memory group. A READ group is ready after its
//              FIRST sub-request (the whole NUM_WORD read is prefetched from the
//              base); the remaining sub-requests of the pair are *absorbed*
//              (counted by the pair phase, served from that same read). A WRITE
//              group gathers all GROUPS sub-requests' wdata before it is ready.
//     - MEM  : the group being issued to the crossbar and received. Drives m_req
//              on its still-unsecured words (partial issue under conflict),
//              secures on m_gnt, collects responses on m_rvalid.
//     - OUT  : a small FIFO of fully-received groups, delivered to the AGU
//              NUM_REQ words/cycle, in order (GROUPS chunks per group).
//   FILL moves to MEM as soon as it is ready and MEM is free, so FILL forms the
//   next group while MEM issues the current one — both reads and writes sustain
//   one sub-request/cycle with no conflict. The AGU grant a_gnt is "FILL can take
//   this sub-request"; a bank conflict stalls MEM, which backs FILL up and so
//   throttles a_gnt (the conflict penalty propagates to the AGU through the
//   pipeline).
//
//   No arbiter select is needed: the upstream x-OBI mux/demux grants/returns only
//   in this buffer's slot.
//
//   v1 scope: assumes each AGU streams full, consecutive NUM_WORD groups, so a
//   group's base + w*stride covers GROUPS consecutive AGU sub-requests (the read
//   prefetch relies on this).
//
//   Reset is active-low (rst_ni).
//
// Template parameters:
//   NUM_REQ  - AGU-side group width (default 4)
//   NUM_WORD - memory-side group width, a multiple of NUM_REQ (default 8)
// -----------------------------------------------------------------------------

#ifndef BUF_HPP
#define BUF_HPP

#include <systemc.h>

#include <cstdint>

template <int NUM_REQ = 4, int NUM_WORD = 8> SC_MODULE(buf) {
    static constexpr int GROUPS = NUM_WORD / NUM_REQ;
    static constexpr int DOUT   = 4;

    static_assert(NUM_REQ >= 1, "NUM_REQ must be >= 1");
    static_assert(NUM_WORD >= NUM_REQ && NUM_WORD % NUM_REQ == 0,
                  "NUM_WORD must be a positive multiple of NUM_REQ");

    sc_in<bool> clk_i;
    sc_in<bool> rst_ni;

    sc_in<bool>      a_req_i[NUM_REQ];
    sc_in<uint64_t>  a_addr_i;
    sc_in<bool>      a_we_i;
    sc_in<uint32_t>  a_be_i;
    sc_in<uint64_t>  a_wdata_i[NUM_REQ];
    sc_out<bool>     a_gnt_o[NUM_REQ];
    sc_out<bool>     a_rvalid_o[NUM_REQ];
    sc_out<uint64_t> a_rdata_o[NUM_REQ];

    sc_out<bool>     m_req_o[NUM_WORD];
    sc_out<uint64_t> m_addr_o;
    sc_out<bool>     m_we_o;
    sc_out<uint32_t> m_be_o;
    sc_out<uint64_t> m_wdata_o[NUM_WORD];
    sc_in<bool>      m_gnt_i[NUM_WORD];
    sc_in<bool>      m_rvalid_i[NUM_WORD];
    sc_in<uint64_t>  m_rdata_i[NUM_WORD];

    sc_signal<int> pp_;
    bool           pp_we_;

    sc_signal<bool> f_busy_;
    bool            f_ready_;
    bool            f_we_;
    uint64_t        f_base_;
    uint32_t        f_be_;
    uint64_t        f_wdata_[NUM_WORD];

    sc_signal<bool>     m_busy_;
    sc_signal<bool>     m_we_;
    sc_signal<uint64_t> m_base_;
    sc_signal<uint32_t> m_be_;
    sc_signal<bool>     m_sec_[NUM_WORD];
    sc_signal<uint64_t> m_wdata_[NUM_WORD];
    bool                m_got_[NUM_WORD];
    uint64_t            m_rdata_[NUM_WORD];

    bool     o_we_[DOUT];
    uint64_t o_rdata_[DOUT][NUM_WORD];
    int      o_rd_, o_cnt_, o_ndel_;

    void comb() {
        const bool g = rst_ni.read() && (pp_.read() != 0 || !f_busy_.read());
        for (int p = 0; p < NUM_REQ; ++p)
            a_gnt_o[p].write(g);

        const bool mb = m_busy_.read();
        for (int w = 0; w < NUM_WORD; ++w) {
            m_req_o[w].write(mb && !m_sec_[w].read());
            m_wdata_o[w].write(mb ? m_wdata_[w].read() : 0);
        }
        m_addr_o.write(mb ? m_base_.read() : 0);
        m_we_o.write(mb ? m_we_.read() : false);
        m_be_o.write(mb ? m_be_.read() : 0);
    }

    void step() {
        if (!rst_ni.read()) {
            pp_.write(0);
            pp_we_ = false;
            f_busy_.write(false);
            f_ready_ = false;
            f_we_    = false;
            f_base_  = 0;
            f_be_    = 0;
            m_busy_.write(false);
            m_we_.write(false);
            m_base_.write(0);
            m_be_.write(0);
            for (int w = 0; w < NUM_WORD; ++w) {
                m_sec_[w].write(false);
                m_wdata_[w].write(0);
                m_got_[w]   = false;
                m_rdata_[w] = 0;
                f_wdata_[w] = 0;
            }
            o_rd_   = 0;
            o_cnt_  = 0;
            o_ndel_ = 0;
            for (int p = 0; p < NUM_REQ; ++p) {
                a_rvalid_o[p].write(false);
                a_rdata_o[p].write(0);
            }
            return;
        }

        const bool present = a_req_i[0].read();
        int        pp      = pp_.read();
        const bool g       = (pp != 0 || !f_busy_.read());

        bool     f_busy = f_busy_.read(), f_ready = f_ready_;
        bool     m_busy = m_busy_.read(), m_we = m_we_.read();
        uint64_t m_base = m_base_.read();
        uint32_t m_be   = m_be_.read();
        bool     m_sec[NUM_WORD];
        uint64_t m_wd[NUM_WORD];
        for (int w = 0; w < NUM_WORD; ++w) {
            m_sec[w] = m_sec_[w].read();
            m_wd[w]  = m_wdata_[w].read();
        }
        int ocnt = o_cnt_;

        if (m_busy) {
            for (int w = 0; w < NUM_WORD; ++w) {
                if (m_rvalid_i[w].read()) {
                    m_got_[w] = true;
                    if (!m_we)
                        m_rdata_[w] = m_rdata_i[w].read();
                }
                if (!m_sec[w] && m_gnt_i[w].read())
                    m_sec[w] = true;
            }
        }

        bool m_all = m_busy;
        for (int w = 0; w < NUM_WORD; ++w)
            if (!m_got_[w]) {
                m_all = false;
                break;
            }
        if (m_busy && m_all && ocnt < DOUT) {
            const int wi = (o_rd_ + ocnt) % DOUT;
            o_we_[wi]    = m_we;
            for (int w = 0; w < NUM_WORD; ++w)
                o_rdata_[wi][w] = m_rdata_[w];
            ++ocnt;
            m_busy = false;
            m_we   = false;
            m_base = 0;
            m_be   = 0;
            for (int w = 0; w < NUM_WORD; ++w) {
                m_sec[w]    = false;
                m_wd[w]     = 0;
                m_got_[w]   = false;
                m_rdata_[w] = 0;
            }
        }

        if (present && g) {
            if (pp == 0) {
                f_busy  = true;
                f_we_   = a_we_i.read();
                f_base_ = a_addr_i.read();
                f_be_   = a_be_i.read();
                pp_we_  = a_we_i.read();
                for (int w = 0; w < NUM_WORD; ++w)
                    f_wdata_[w] = 0;
                if (a_we_i.read()) {
                    for (int p = 0; p < NUM_REQ; ++p)
                        f_wdata_[p] = a_wdata_i[p].read();
                    f_ready = (GROUPS == 1);
                } else {
                    f_ready = true;
                }
            } else if (pp_we_) {
                for (int p = 0; p < NUM_REQ; ++p)
                    f_wdata_[pp * NUM_REQ + p] = a_wdata_i[p].read();
                if (pp == GROUPS - 1)
                    f_ready = true;
            }
            pp = (pp + 1) % GROUPS;
        }

        if (f_busy && f_ready && !m_busy) {
            m_busy = true;
            m_we   = f_we_;
            m_base = f_base_;
            m_be   = f_be_;
            for (int w = 0; w < NUM_WORD; ++w) {
                m_sec[w]    = false;
                m_got_[w]   = false;
                m_wd[w]     = f_wdata_[w];
                m_rdata_[w] = 0;
            }
            f_busy  = false;
            f_ready = false;
        }

        const bool deliver = ocnt > 0 && o_ndel_ < NUM_WORD;
        for (int p = 0; p < NUM_REQ; ++p) {
            a_rvalid_o[p].write(deliver);
            a_rdata_o[p].write((deliver && !o_we_[o_rd_]) ? o_rdata_[o_rd_][o_ndel_ + p] : 0);
        }
        if (deliver) {
            o_ndel_ += NUM_REQ;
            if (o_ndel_ >= NUM_WORD) {
                o_rd_ = (o_rd_ + 1) % DOUT;
                --ocnt;
                o_ndel_ = 0;
            }
        }

        pp_.write(pp);
        f_busy_.write(f_busy);
        f_ready_ = f_ready;
        m_busy_.write(m_busy);
        m_we_.write(m_we);
        m_base_.write(m_base);
        m_be_.write(m_be);
        for (int w = 0; w < NUM_WORD; ++w) {
            m_sec_[w].write(m_sec[w]);
            m_wdata_[w].write(m_wd[w]);
        }
        o_cnt_ = ocnt;
    }

    SC_CTOR(buf) {
        pp_we_   = false;
        f_ready_ = false;
        f_we_    = false;
        f_base_  = 0;
        f_be_    = 0;
        for (int w = 0; w < NUM_WORD; ++w) {
            m_got_[w]   = false;
            m_rdata_[w] = 0;
            f_wdata_[w] = 0;
        }
        o_rd_   = 0;
        o_cnt_  = 0;
        o_ndel_ = 0;
        for (int d = 0; d < DOUT; ++d) {
            o_we_[d] = false;
            for (int w = 0; w < NUM_WORD; ++w)
                o_rdata_[d][w] = 0;
        }

        SC_METHOD(step);
        sensitive << clk_i.pos();
        dont_initialize();

        SC_METHOD(comb);
        sensitive << rst_ni << pp_ << f_busy_ << m_busy_ << m_we_ << m_base_ << m_be_;
        for (int w = 0; w < NUM_WORD; ++w)
            sensitive << m_sec_[w] << m_wdata_[w];
    }
};

#endif
