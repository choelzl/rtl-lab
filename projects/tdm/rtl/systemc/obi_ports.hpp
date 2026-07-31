// -----------------------------------------------------------------------------
// Reusable OBI port bundles (see doc/specs/obi.md) — every module here
// needs the same req/addr/we/be/wdata/gnt/rvalid/rdata octet; these types
// group it into one named MEMBER (not a base class: these modules are
// class templates on data width, and unqualified lookup doesn't reach into
// a dependent base — tried, reverted). Bind sites are `mod.<member>.req_i`
// instead of the old flat `mod.req_i`.
//
//   subordinate (target): in req/addr/we/be/wdata_i, out gnt/rvalid/rdata_o
//   manager (initiator): out req/addr/we/be/wdata_o, in gnt/rvalid/rdata_i
//   observer (passive tap): all 8 fields sc_in
//   signal bundle: the 8 plain sc_signals wiring two submodules together
//
// Signal names are left auto-generated rather than hardcoded ("req_i" etc.)
// since two bundle members as siblings would otherwise collide.
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
// every top_*<> wrapper exposes, kept flat for SV-backend parity) to an
// obi_signal_bundle array one-shot, replacing an 8-line per-index bind loop.
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
// obi_signal_bundle wire one-shot — the bundle-to-bundle pattern used at
// every crossbar/mux stage (top_crossbar.hpp/top_tdm.hpp). A third overload
// binds two obi_manager_ports directly for the hierarchical port-export
// case (a child submodule's sc_out bound to its parent's own, e.g.
// buffer<>'s cells[t]->m to its own m[t]).
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
