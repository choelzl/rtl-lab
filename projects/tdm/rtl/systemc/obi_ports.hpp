// -----------------------------------------------------------------------------
// Reusable OBI port bundles — every module in this design exposes at least
// one OBI-like req/addr/we/be/wdata/gnt/rvalid/rdata octet (see
// doc/specs/obi.md), and until now each one spelled out all 8 sc_in/sc_out
// declarations by hand. That's the single most repeated block in the whole
// codebase (bank.hpp, buffer_cell.hpp, buffer.hpp, crossbar.hpp, tdm.hpp,
// agu.hpp, lane_agu.hpp, obi_monitor.hpp, top.hpp, top_tdm.hpp,
// top_crossbar.hpp...).
//
// obi_subordinate_ports<DATA_T> / obi_manager_ports<DATA_T> group those 8
// ports into one named type, brought into a module as a NAMED MEMBER (not a
// base class): every module here is a class template parameterized on the
// data width, which would make the bundle a dependent base class if
// inherited — and C++ doesn't do unqualified lookup into a dependent base,
// so every reference inside the module's own method bodies would break
// (tried, reverted). A plain member sidesteps that: `obi.req_i` resolves
// through ordinary member lookup regardless of template-dependence.
//
// This DOES mean every external bind site changes shape, from
// `mod.req_i(sig)` to `mod.<member_name>.req_i(sig)` — there is no way to
// get the old flat names back without either the dependent-base problem
// above or a macro. Each module picks its own member name(s) for readability
// (e.g. bank<>'s single group is just `obi`; a module with two groups, like
// buffer<>'s port-facing and TDM-facing sides, needs two distinctly-named
// members).
//
//   subordinate (the OBI *target* — receives req, drives response):
//     in : req_i, addr_i, we_i, be_i, wdata_i
//     out: gnt_o, rvalid_o, rdata_o
//   manager (the OBI *initiator* — drives req, receives response):
//     out: req_o, addr_o, we_o, be_o, wdata_o
//     in : gnt_i, rvalid_i, rdata_i
//   observer (a passive wire-tap — drives nothing, all fields sc_in):
//     in : req_i, addr_i, we_i, be_i, wdata_i, gnt_i, rvalid_i, rdata_i
//   signal bundle (plain internal wiring between two submodules — neither
//   sc_in nor sc_out, just the 8 sc_signals a req/resp OBI link needs):
//     req, addr, we, be, wdata, gnt, rvalid, rdata
//
// Signal names are deliberately left auto-generated (no explicit string
// passed to each sc_in/sc_out constructor) rather than hardcoded to
// "req_i"/etc: a hardcoded name would collide the moment two bundle members
// exist as siblings under the same parent module, exactly like
// preload_ctrl_t's naming rule elsewhere in this codebase. SystemC's default
// per-object unique naming already produces distinct, if less immediately
// descriptive, names in waveforms; the C++ member path (`obi.req_i` etc.)
// remains fully descriptive in code.
// -----------------------------------------------------------------------------

#ifndef OBI_PORTS_HPP
#define OBI_PORTS_HPP

#include <systemc.h>

#include <cstdint>

template <typename DATA_T> struct obi_subordinate_ports {
    sc_in<bool>     req_i;
    sc_in<uint64_t> addr_i;
    sc_in<bool>     we_i;
    sc_in<uint32_t> be_i;
    sc_in<DATA_T>   wdata_i;
    sc_out<bool>    gnt_o;
    sc_out<bool>    rvalid_o;
    sc_out<DATA_T>  rdata_o;
};

template <typename DATA_T> struct obi_manager_ports {
    sc_out<bool>     req_o;
    sc_out<uint64_t> addr_o;
    sc_out<bool>     we_o;
    sc_out<uint32_t> be_o;
    sc_out<DATA_T>   wdata_o;
    sc_in<bool>      gnt_i;
    sc_in<bool>      rvalid_i;
    sc_in<DATA_T>    rdata_i;
};

// A passive wire-tap: taps an existing manager/subordinate pair without
// driving either side, so every field is an input regardless of which
// direction it would be on a real manager or subordinate (obi_monitor.hpp is
// the only user — it observes req/addr/we/be/wdata alongside gnt/rvalid/rdata
// on the same bus, never asserting any of them itself).
template <typename DATA_T> struct obi_observer_ports {
    sc_in<bool>     req_i;
    sc_in<uint64_t> addr_i;
    sc_in<bool>     we_i;
    sc_in<uint32_t> be_i;
    sc_in<DATA_T>   wdata_i;
    sc_in<bool>     gnt_i;
    sc_in<bool>     rvalid_i;
    sc_in<DATA_T>   rdata_i;
};

// Plain internal wiring: the 8 sc_signals connecting one submodule's OBI
// manager side to another's subordinate side (or a top-level wrapper's own
// internal req/resp plumbing). Unlike the port bundles above these are never
// bound to anything — they ARE the wire, read/written directly via
// .read()/.write() on each field.
template <typename DATA_T> struct obi_signal_bundle {
    sc_signal<bool>     req;
    sc_signal<uint64_t> addr;
    sc_signal<bool>     we;
    sc_signal<uint32_t> be;
    sc_signal<DATA_T>   wdata;
    sc_signal<bool>     gnt;
    sc_signal<bool>     rvalid;
    sc_signal<DATA_T>   rdata;
};

// Binds a flat subordinate-shaped port group (the *_req_i/../_rdata_o arrays
// every top_*<> wrapper still exposes, kept flat for SV-backend parity — see
// top_crossbar_sv.hpp/top_tdm_sv.hpp) to an obi_signal_bundle array one-shot,
// replacing the 8-line per-index bind loop that appeared at every such call
// site (top.hpp's own impl binding, and every unit test that instantiates
// top_tdm<>/top_crossbar<> directly).
template <int N, typename DATA_T>
void bind_obi_group(sc_in<bool> (&req_i)[N], sc_in<uint64_t> (&addr_i)[N], sc_in<bool> (&we_i)[N],
                    sc_in<uint32_t> (&be_i)[N], sc_in<DATA_T> (&wdata_i)[N],
                    sc_out<bool> (&gnt_o)[N], sc_out<bool> (&rvalid_o)[N],
                    sc_out<DATA_T> (&rdata_o)[N], obi_signal_bundle<DATA_T> (&sig)[N]) {
    for (int i = 0; i < N; ++i) {
        req_i[i](sig[i].req);
        addr_i[i](sig[i].addr);
        we_i[i](sig[i].we);
        be_i[i](sig[i].be);
        wdata_i[i](sig[i].wdata);
        gnt_o[i](sig[i].gnt);
        rvalid_o[i](sig[i].rvalid);
        rdata_o[i](sig[i].rdata);
    }
}

// Binds one obi_manager_ports/obi_subordinate_ports member to an
// obi_signal_bundle wire one-shot (the other half of the bundle-to-bundle
// wiring pattern that shows up at every crossbar/mux stage in
// top_crossbar.hpp and top_tdm.hpp — L1/L2/L3 inter-level wires, TDM
// buf<->mux<->map<->crossbar<->bank chains — replacing an 8-line
// field-by-field bind with one call). A third overload binds two
// obi_manager_ports directly together for the hierarchical port-export case
// (a parent module's own sc_out bound straight to a child submodule's sc_out,
// e.g. buffer<>'s cells[t]->m export to its own m[t] — see buffer.hpp).
template <typename DATA_T>
void bind_obi(obi_manager_ports<DATA_T> &port, obi_signal_bundle<DATA_T> &sig) {
    port.req_o(sig.req);
    port.addr_o(sig.addr);
    port.we_o(sig.we);
    port.be_o(sig.be);
    port.wdata_o(sig.wdata);
    port.gnt_i(sig.gnt);
    port.rvalid_i(sig.rvalid);
    port.rdata_i(sig.rdata);
}

template <typename DATA_T>
void bind_obi(obi_subordinate_ports<DATA_T> &port, obi_signal_bundle<DATA_T> &sig) {
    port.req_i(sig.req);
    port.addr_i(sig.addr);
    port.we_i(sig.we);
    port.be_i(sig.be);
    port.wdata_i(sig.wdata);
    port.gnt_o(sig.gnt);
    port.rvalid_o(sig.rvalid);
    port.rdata_o(sig.rdata);
}

template <typename DATA_T>
void bind_obi(obi_manager_ports<DATA_T> &child, obi_manager_ports<DATA_T> &parent) {
    child.req_o(parent.req_o);
    child.addr_o(parent.addr_o);
    child.we_o(parent.we_o);
    child.be_o(parent.be_o);
    child.wdata_o(parent.wdata_o);
    child.gnt_i(parent.gnt_i);
    child.rvalid_i(parent.rvalid_i);
    child.rdata_i(parent.rdata_i);
}

// Binds an obi_observer_ports wire-tap to an obi_signal_bundle (all fields
// are sc_in on the observer side, so this is a plain field-by-field bind —
// see obi_monitor.hpp, the only user).
template <typename DATA_T>
void bind_obi(obi_observer_ports<DATA_T> &port, obi_signal_bundle<DATA_T> &sig) {
    port.req_i(sig.req);
    port.addr_i(sig.addr);
    port.we_i(sig.we);
    port.be_i(sig.be);
    port.wdata_i(sig.wdata);
    port.gnt_i(sig.gnt);
    port.rvalid_i(sig.rvalid);
    port.rdata_i(sig.rdata);
}

#endif // OBI_PORTS_HPP
