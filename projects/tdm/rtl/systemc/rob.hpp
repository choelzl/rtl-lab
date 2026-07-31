// -----------------------------------------------------------------------------
// rob.hpp — read-side reorder-buffer complex for the crossbar (XBAR_ROB
// experiment, doc/report §4.4).
//
// One ROB per agu<>-driven read port plus the shared cross-ROB scheduler,
// packaged as one module: the scheduler is global by design (it synchronizes
// all ROBs into a single conflict-free issue set per cycle), so the per-port
// buffers and the arbitration belong together. The module sits BETWEEN the
// (already-hashed) ports and the untouched L1/L2/L3 read fabric:
//
//   port face  (p_*):  the external rport OBI of the covered ports — served
//                      by the ROB's delivery logic (combinational grants,
//                      registered rvalid exactly 1 cycle later);
//   fabric face (f_*): OBI masters into the L1 read switches — driven by the
//                      scheduler, conflict-free by construction;
//   fetch face:        group-granular prefetch buses from each AGU's
//                      lookahead cursor (wired by the tb like the TDM
//                      buffers' lookahead buses), plus the LOOKAHEAD task's
//                      geometry for descriptor-keyed hashes.
//
// Scheduler: hardware-feasible by default (per-lane head-only proposals, a
// per-port L1-field filter, per-bank rotating-priority arbiters — the same
// primitive classes the fabric itself uses); XBAR_ROB_SCHED_IDEAL restores
// the idealized global age-sorted greedy for A/B reference (measured within
// noise of each other, and the feasible scheduler's per-bank rotation also
// fixes the ideal one's fairness collapse at synthetic full saturation —
// see tb/unit/stim_bank_common.hpp's phase-8 XBAR_ROB comment). DEPTH counts
// resident groups INCLUDING the one being delivered; 4 is the streaming
// break-even in this model (see doc/report §4.4). Correctness — OBI on both
// faces, NOPs never routed into the fabric, scheduler invariants, exact
// read-data delivery, demonstrated out-of-order issue — is regression-locked
// by tb_stim_bank_xbar_rob.cpp.
// -----------------------------------------------------------------------------

#ifndef ROB_HPP
#define ROB_HPP

#include <algorithm>
#include <cstdint>
#include <systemc.h>

template <int N_PORTS, int NUM_REQ, int NUM_BANK_GRP, int DEPTH, typename DATA_T,
          typename HASH_OPS>
SC_MODULE(rob_complex) {
    static constexpr int ROB_PORTS = N_PORTS;
    static constexpr int ROB_DEPTH = DEPTH;
    static constexpr int ROB_LANES = N_PORTS * NUM_REQ;

    sc_in<bool> clk_i;
    sc_in<bool> rst_ni;

    // --- port face (external rport OBI of the covered ports) ---
    sc_in<bool>     p_req_i[ROB_LANES];
    sc_in<uint64_t> p_addr_i[ROB_LANES];  // raw (pad requests carry 0)
    sc_in<uint64_t> p_haddr_i[ROB_LANES]; // hashed request address (match key)
    sc_out<bool>    p_gnt_o[ROB_LANES];
    sc_out<bool>    p_rvalid_o[ROB_LANES];
    sc_out<DATA_T>  p_rdata_o[ROB_LANES];

    // --- fabric face (OBI masters into the L1 read switches) ---
    sc_out<bool>     f_req_o[ROB_LANES];
    sc_out<uint64_t> f_addr_o[ROB_LANES];
    sc_out<bool>     f_we_o[ROB_LANES];
    sc_out<uint32_t> f_be_o[ROB_LANES];
    sc_out<DATA_T>   f_wdata_o[ROB_LANES];
    sc_in<bool>      f_gnt_i[ROB_LANES];
    sc_in<bool>      f_rvalid_i[ROB_LANES];
    sc_in<DATA_T>    f_rdata_i[ROB_LANES];

    // --- fetch face: the tb drives each port's 4-lane slice of its AGU's
    // oldest un-ingested group (raw addresses; 0 = NOP hole) and holds valid
    // while ALL sibling ROB ports of that AGU are ready, so siblings ingest
    // the same group on the same edge; ack pulses one cycle per ingest so
    // the tb can advance the AGU's lookahead cursor. la_* carry the
    // LOOKAHEAD task's geometry (descriptor-keyed hashes must not use the
    // in-service task's fields — they differ once prefetch has rolled into
    // the next task).
    sc_in<uint64_t> fetch_addr_i[ROB_LANES];
    sc_in<bool>     fetch_valid_i[ROB_PORTS];
    sc_out<bool>    fetch_ready_o[ROB_PORTS];
    sc_out<bool>    fetch_ack_o[ROB_PORTS];
    sc_in<uint64_t> la_r_i[ROB_PORTS];
    sc_in<uint64_t> la_c_i[ROB_PORTS];
    sc_in<uint64_t> la_l_i[ROB_PORTS];
    sc_in<uint64_t> la_sm_i[ROB_PORTS];
    sc_in<uint64_t> la_napa_i[ROB_PORTS];
#if defined(XBAR_HASH16)
    // Fixed per-AGU bank-half selector, forwarded from the parent's
    // rport_map_hi_bank_i (address-independent, task-independent).
    sc_in<bool> hi_bank_i[ROB_PORTS];
#endif

    // Entry lifecycle, TDM-buffer-cell style: INVALID (empty, or hole, or
    // already delivered) -> FETCHING (ingested; the request/response phase
    // within the fabric is tracked by the WIRE bookkeeping below, not by
    // extra entry states) -> VALID (data resident, deliverable) -> back to
    // INVALID at delivery. NOP holes ingest as INVALID with hole=true
    // (grantable against a raw addr-0 pad request; nothing to fetch).
    enum state_t { ROB_INVALID, ROB_FETCHING, ROB_VALID };
    struct entry_t {
        uint64_t    haddr = 0;
        state_t st    = ROB_INVALID;
        bool        hole  = false;
        DATA_T      data{};
    };
    entry_t ent_[ROB_PORTS][ROB_DEPTH][NUM_REQ];
    uint64_t    slot_seq_[ROB_PORTS][ROB_DEPTH] = {};
    int         head_[ROB_PORTS]  = {};
    int         count_[ROB_PORTS] = {};
    uint64_t    seq_next_ = 1;
    // Per-lane-wire bookkeeping. The fabric pipelines: a wire can carry a
    // NEW request while earlier responses are still in flight (rvalids
    // return in grant order, one per cycle), so a wire holds one
    // awaiting-gnt slot plus a short in-order queue of granted slots
    // awaiting rvalid — freeing the wire only at rvalid would halve the
    // sustainable per-lane bandwidth.
    int wire_req_[ROB_LANES];               // slot awaiting gnt (-1 none)
    int wire_wd_[ROB_LANES][2];             // granted slots awaiting rvalid (FIFO)
    int wire_wd_n_[ROB_LANES] = {};
    // Delivery grants are COMBINATIONAL (deliver_comb below): the tb's
    // conflict tally samples the port signals between the posedge and the
    // negedge (each sc_start window ends mid-cycle), exactly where the
    // fabric's combinational grants are visible — a clock-edge-driven gnt
    // would be invisible to it and tally every beat as delayed.
    // state_tick_ toggles every posedge so the comb method re-evaluates
    // after the clocked state updates; the method computes gnt purely from
    // state (no side effects), so re-firing within deltas is harmless.
    sc_signal<bool> state_tick_;

    // Counters (public; tb_top writes them to stats.log).
    uint64_t underrun_wait = 0;  // port req cycles waiting on a not-yet-VALID head entry
    uint64_t fabric_hold   = 0;  // held request cycles not granted (L3 R/W backstop)
    uint64_t sched_eligible = 0, sched_issued = 0;
    uint64_t ingest_groups = 0, mismatch_cnt = 0;
    // Issues accepted AFTER an older candidate was skipped for a bank/L1
    // conflict in the same pass — i.e. genuinely out-of-order fetches (the
    // mechanism that dissolves same-bank pileups). Wire-busy skips don't
    // count (structural, not reordering).
    uint64_t sched_ooo = 0;
    // Per-bank rotating priority for the hardware-feasible scheduler (the
    // default; XBAR_ROB_SCHED_IDEAL restores the global age-sorted greedy
    // model for A/B comparison).
    int bank_rr_[NUM_REQ * NUM_BANK_GRP] = {};

    // Same dispatch as hash_rd_addr(), for one prefetched address with the
    // LOOKAHEAD task's geometry.
    uint64_t hash_one(uint64_t a, int j) const {
#if defined(XBAR_HASH_DYNAMIC)
        return HASH_OPS::addr_hash_dynamic(a, la_r_i[j].read(), la_c_i[j].read(),
                                           la_l_i[j].read(), la_sm_i[j].read());
#elif defined(XBAR_HASH16)
        return HASH_OPS::addr_hash16(a, la_r_i[j].read(), la_c_i[j].read(),
                                     la_l_i[j].read(), la_sm_i[j].read(),
                                     hi_bank_i[j].read());
#elif defined(XBAR_HASH32)
        return HASH_OPS::addr_hash32(a, la_r_i[j].read(), la_c_i[j].read(),
                                     la_l_i[j].read(), la_sm_i[j].read());
#elif defined(XBAR_HASH_L1_V2)
        return HASH_OPS::addr_hash_l1_v2(a, la_r_i[j].read(), la_c_i[j].read(),
                                         la_napa_i[j].read());
#elif defined(XBAR_HASH_L1_V3)
        return HASH_OPS::addr_hash_l1_v3(a, la_c_i[j].read());
#else
        (void)j;
        return HASH_OPS::addr_hash(a);
#endif
    }

    static int l1f(uint64_t h) { return static_cast<int>((h >> 4) & 0x3); }
    static int l2f(uint64_t h) { return static_cast<int>((h >> 6) & 0x7); }

    // Posedge half: sample fabric gnt/rvalid, drive delivery rvalid for
    // grants decided on the previous negedge, retire completed slots,
    // ingest fetched groups.
    void step_pos() {
        if (!rst_ni.read()) {
            for (int j = 0; j < ROB_PORTS; ++j) {
                head_[j] = count_[j] = 0;
                fetch_ready_o[j].write(false);
                fetch_ack_o[j].write(false);
                for (int s = 0; s < ROB_DEPTH; ++s)
                    for (int m = 0; m < NUM_REQ; ++m)
                        ent_[j][s][m] = entry_t{};
            }
            for (int w = 0; w < ROB_LANES; ++w) {
                wire_req_[w]  = -1;
                wire_wd_n_[w] = 0;
                f_we_o[w].write(false);
                f_be_o[w].write(0xffffffffu);
                f_wdata_o[w].write(DATA_T(0));
                p_rvalid_o[w].write(false);
            }
            state_tick_.write(!state_tick_.read());
            return;
        }
        // 0) launch rvalids for lanes granted during the LAST cycle (comb
        // gnt held high across it; the AGU's req is still sampled high
        // here) and classify waited lanes for the underrun counters.
        for (int j = 0; j < ROB_PORTS; ++j) {
            const int h = head_[j];
            for (int m = 0; m < NUM_REQ; ++m) {
                const int w = j * NUM_REQ + m;
                if (p_req_i[w].read() && p_gnt_o[w].read() && count_[j] > 0) {
                    entry_t &e = ent_[j][h][m];
                    p_rvalid_o[w].write(true);
                    p_rdata_o[w].write(e.data);
                    e.st = ROB_INVALID; // delivered (holes are INVALID already)
                } else {
                    p_rvalid_o[w].write(false);
                    if (p_req_i[w].read() && !p_gnt_o[w].read()) {
                        if (count_[j] > 0 && ent_[j][h][m].st == ROB_VALID &&
                            !ent_[j][h][m].hole &&
                            ent_[j][h][m].haddr != p_haddr_i[w].read())
                            ++mismatch_cnt;
                        ++underrun_wait;
                    }
                }
            }
        }
        // 1) fabric handshakes: rvalid pops the oldest awaiting-data slot;
        // gnt moves the awaiting-gnt slot into the awaiting-data queue.
        for (int j = 0; j < ROB_PORTS; ++j) {
            for (int m = 0; m < NUM_REQ; ++m) {
                const int w = j * NUM_REQ + m;
                if (f_rvalid_i[w].read() && wire_wd_n_[w] > 0) {
                    entry_t &e = ent_[j][wire_wd_[w][0]][m];
                    e.data = f_rdata_i[w].read();
                    e.st   = ROB_VALID;
                    wire_wd_[w][0] = wire_wd_[w][1];
                    --wire_wd_n_[w];
                }
                const int s = wire_req_[w];
                if (s >= 0) {
                    if (f_gnt_i[w].read()) {
                        // entry stays FETCHING; the wire moves it from the
                        // awaiting-gnt stage to the awaiting-data queue
                        wire_wd_[w][wire_wd_n_[w]++] = s;
                        wire_req_[w]                     = -1;
                    } else {
                        ++fabric_hold;
                    }
                }
            }
        }
        // 3) retire the head slot once every entry is INVALID (real lanes
        // reach INVALID exactly at delivery; holes are INVALID from ingest
        // — idle ports never request them, and a pad lane an ACTIVE port
        // will request is always granted no later than that group's
        // slowest real lane, so retirement cannot outrun a pending pad
        // request).
        for (int j = 0; j < ROB_PORTS; ++j) {
            if (count_[j] == 0)
                continue;
            const int h   = head_[j];
            bool      all = true;
            for (int m = 0; m < NUM_REQ; ++m) {
                if (ent_[j][h][m].st != ROB_INVALID)
                    all = false;
            }
            if (all) {
                for (int m = 0; m < NUM_REQ; ++m)
                    ent_[j][h][m] = entry_t{};
                head_[j] = (h + 1) % ROB_DEPTH;
                --count_[j];
            }
        }
        // 4) ingest
        for (int j = 0; j < ROB_PORTS; ++j) {
            bool ack = false;
            if (fetch_valid_i[j].read() && count_[j] < ROB_DEPTH) {
                const int s = (head_[j] + count_[j]) % ROB_DEPTH;
                for (int m = 0; m < NUM_REQ; ++m) {
                    entry_t &e = ent_[j][s][m];
                    const uint64_t raw = fetch_addr_i[j * NUM_REQ + m].read();
                    e = entry_t{};
                    if (raw == 0) {
                        e.hole = true; // INVALID; grantable via the pad rule
                    } else {
                        e.haddr = hash_one(raw, j);
                        e.st    = ROB_FETCHING;
                    }
                }
                slot_seq_[j][s] = seq_next_++;
                ++count_[j];
                ++ingest_groups;
                ack = true;
            }
            fetch_ack_o[j].write(ack);
            fetch_ready_o[j].write(count_[j] < ROB_DEPTH);
        }
        state_tick_.write(!state_tick_.read());
    }

    // Combinational delivery grants: pure function of ROB state + the
    // port's request signals, no side effects (safe to re-fire within
    // deltas). Sensitive to state_tick_ (posedge state updates) and
    // every covered rport req/addr signal.
    void deliver_comb() {
        if (!rst_ni.read()) {
            for (int w = 0; w < ROB_LANES; ++w)
                p_gnt_o[w].write(false);
            return;
        }
        for (int j = 0; j < ROB_PORTS; ++j) {
            const int h = head_[j];
            for (int m = 0; m < NUM_REQ; ++m) {
                const int w = j * NUM_REQ + m;
                bool      g = false;
                if (p_req_i[w].read() && count_[j] > 0) {
                    const entry_t &e = ent_[j][h][m];
                    if (e.hole) {
                        // pad request: raw addr 0 (matched RAW — descriptor
                        // hashes need not map 0 to 0)
                        g = p_addr_i[w].read() == 0;
                    } else if (e.st == ROB_VALID) {
                        g = e.haddr == p_haddr_i[w].read();
                    }
                }
                p_gnt_o[w].write(g);
            }
        }
    }

    // Negedge half: the cross-ROB conflict-ahead scheduler (fabric-side
    // request driving only; port-side grants are deliver_comb()).
    void step_neg() {
        if (!rst_ni.read()) {
            for (int w = 0; w < ROB_LANES; ++w)
                f_req_o[w].write(false);
            return;
        }
        // Scheduler. Constraint sets seed from still-arbitrating
        // (awaiting-gnt) wires only — granted requests have left the
        // arbitration stage.
        bool l1_used[ROB_PORTS][NUM_REQ] = {};
        bool bank_used[NUM_REQ * NUM_BANK_GRP] = {};
        for (int j = 0; j < ROB_PORTS; ++j)
            for (int m = 0; m < NUM_REQ; ++m) {
                const int w = j * NUM_REQ + m;
                const int s = wire_req_[w];
                if (s >= 0) {
                    const uint64_t hh = ent_[j][s][m].haddr;
                    l1_used[j][l1f(hh)]                             = true;
                    bank_used[l1f(hh) * NUM_BANK_GRP + l2f(hh)] = true;
                }
            }
#if defined(XBAR_ROB_SCHED_IDEAL)
        // IDEALIZED reference scheduler (not hardware-shaped): global
        // age-sort over EVERY un-issued entry plus a serial greedy walk —
        // up to ROB_LANES*ROB_DEPTH candidates and a dependency chain the
        // length of the walk. Kept for A/B against the feasible default.
        struct cand_t { uint64_t seq; int j, s, m; };
        cand_t cands[ROB_LANES * ROB_DEPTH];
        int    nc = 0;
        for (int j = 0; j < ROB_PORTS; ++j)
            for (int s0 = 0; s0 < count_[j]; ++s0) {
                const int s = (head_[j] + s0) % ROB_DEPTH;
                for (int m = 0; m < NUM_REQ; ++m) {
                    const int w = j * NUM_REQ + m;
                    if (ent_[j][s][m].st != ROB_FETCHING || wire_req_[w] >= 0)
                        continue;
                    bool in_flight = false; // already granted, awaiting rvalid
                    for (int k = 0; k < wire_wd_n_[w]; ++k)
                        in_flight = in_flight || wire_wd_[w][k] == s;
                    if (!in_flight)
                        cands[nc++] = {slot_seq_[j][s], j, s, m};
                }
            }
        sched_eligible += nc;
        std::sort(cands, cands + nc,
                  [](const cand_t &a, const cand_t &b) { return a.seq < b.seq; });
        bool wire_taken[ROB_LANES] = {};
        bool older_blocked         = false;
        for (int i = 0; i < nc; ++i) {
            const cand_t &c = cands[i];
            const int     w = c.j * NUM_REQ + c.m;
            if (wire_taken[w])
                continue; // wire claimed by an older slot this cycle
            entry_t &e  = ent_[c.j][c.s][c.m];
            const int    f1 = l1f(e.haddr);
            const int    bk = f1 * NUM_BANK_GRP + l2f(e.haddr);
            if (l1_used[c.j][f1] || bank_used[bk]) {
                older_blocked = true; // conflict skip: younger accepts below are OOO
                continue;
            }
            l1_used[c.j][f1] = true;
            bank_used[bk]    = true;
            wire_req_[w] = c.s;
            wire_taken[w]    = true;
            ++sched_issued;
            if (older_blocked)
                ++sched_ooo;
        }
#else
        // HARDWARE-FEASIBLE scheduler (default) — no global sort, no
        // long dependency chain; three fixed stages built from the same
        // arbitration primitives the fabric itself uses:
        //   1) per LANE: propose only the oldest un-issued entry (slots
        //      are already age-ordered, so this is the lane FIFO's head —
        //      free), <=1 proposal per wire;
        //   2) per PORT: filter its <=4 proposals to distinct L1 fields,
        //      oldest wins (a 4-way compare);
        //   3) per BANK: a rotating-priority arbiter over the surviving
        //      proposals (32 arbiters, same class as the L2 stage's
        //      per-output arbiters).
        // Reordering ACROSS lanes/groups survives head-only candidacy
        // (each lane independently advances past its issued beats); what
        // is lost is only intra-lane lookpast (a lane whose head is
        // bank-blocked cannot offer its younger entry that cycle).
        int prop_slot[ROB_LANES];
        for (int w = 0; w < ROB_LANES; ++w)
            prop_slot[w] = -1;
        int n_props = 0;
        for (int j = 0; j < ROB_PORTS; ++j)
            for (int m = 0; m < NUM_REQ; ++m) {
                const int w = j * NUM_REQ + m;
                if (wire_req_[w] >= 0)
                    continue;
                for (int s0 = 0; s0 < count_[j]; ++s0) { // oldest first
                    const int s = (head_[j] + s0) % ROB_DEPTH;
                    if (ent_[j][s][m].st != ROB_FETCHING)
                        continue;
                    bool in_flight = false;
                    for (int k = 0; k < wire_wd_n_[w]; ++k)
                        in_flight = in_flight || wire_wd_[w][k] == s;
                    if (!in_flight) {
                        prop_slot[w] = s;
                        ++n_props;
                        break; // head-only: one proposal per lane
                    }
                }
            }
        sched_eligible += n_props;
        // stage 2: per-port L1-field filter, oldest wins
        for (int j = 0; j < ROB_PORTS; ++j)
            for (int f = 0; f < NUM_REQ; ++f) {
                int      best_m  = -1;
                uint64_t best_sq = 0;
                for (int m = 0; m < NUM_REQ; ++m) {
                    const int w = j * NUM_REQ + m;
                    const int s = prop_slot[w];
                    if (s < 0 || l1f(ent_[j][s][m].haddr) != f)
                        continue;
                    if (l1_used[j][f]) { // field held by an in-flight request
                        prop_slot[w] = -1;
                        continue;
                    }
                    const uint64_t sq = slot_seq_[j][s];
                    if (best_m < 0 || sq < best_sq) {
                        if (best_m >= 0)
                            prop_slot[j * NUM_REQ + best_m] = -1;
                        best_m = m; best_sq = sq;
                    } else {
                        prop_slot[w] = -1;
                    }
                }
            }
        // stage 3: per-bank rotating-priority arbiter. With
        // XBAR_ROB_URGENT, arbitration is two-class: proposals for a
        // lane's HEAD slot (the group next to be consumed at the port —
        // an underrun is imminent if it isn't fetched) win over pure
        // prefetch top-ups, urgency first, rotation within each class —
        // a standard two-level arbiter, one extra priority bit per
        // proposal in hardware.
        for (int bk = 0; bk < NUM_REQ * NUM_BANK_GRP; ++bk) {
            if (bank_used[bk])
                continue;
            int winner = -1;
#if defined(XBAR_ROB_URGENT)
            for (int urgent = 1; urgent >= 0 && winner < 0; --urgent)
#else
            const int urgent = -1; // single class
#endif
            for (int off = 0; off < ROB_LANES; ++off) {
                const int w = (bank_rr_[bk] + off) % ROB_LANES;
                const int s = prop_slot[w];
                if (s < 0)
                    continue;
                const int j = w / NUM_REQ, m = w % NUM_REQ;
#if defined(XBAR_ROB_URGENT)
                if ((s == head_[j]) != (urgent == 1))
                    continue; // wrong class for this pass
#endif
                const uint64_t hh = ent_[j][s][m].haddr;
                if (l1f(hh) * NUM_BANK_GRP + l2f(hh) == bk) {
                    winner = w;
                    break;
                }
            }
            if (winner < 0)
                continue;
            const int j = winner / NUM_REQ, m = winner % NUM_REQ;
            const int s = prop_slot[winner];
            // OOO bookkeeping: issuing while ANY lane of the machine holds
            // an older still-unissued proposal that lost its bank/L1 race
            // this cycle counts as out-of-order progress.
            wire_req_[winner] = s;
            prop_slot[winner]     = -1;
            (void)m;
            ++sched_issued;
            bank_rr_[bk] = (winner + 1) % ROB_LANES;
        }
        // any surviving proposal lost a race this cycle; if a younger
        // entry issued anywhere while an older proposal remained, that is
        // out-of-order progress (approximate, for the demonstration
        // counter only)
        for (int w = 0; w < ROB_LANES; ++w)
            if (prop_slot[w] >= 0) {
                ++sched_ooo;
                break;
            }
#endif
        // 3) drive fabric request wires
        for (int j = 0; j < ROB_PORTS; ++j)
            for (int m = 0; m < NUM_REQ; ++m) {
                const int w = j * NUM_REQ + m;
                const int s = wire_req_[w];
                f_req_o[w].write(s >= 0);
                if (s >= 0)
                    f_addr_o[w].write(ent_[j][s][m].haddr);
            }
    }

    SC_CTOR(rob_complex) {
        for (int w = 0; w < ROB_LANES; ++w)
            wire_req_[w] = -1;
        SC_METHOD(step_pos);
        sensitive << clk_i.pos();
        dont_initialize();
        SC_METHOD(step_neg);
        sensitive << clk_i.neg();
        dont_initialize();
        SC_METHOD(deliver_comb);
        sensitive << state_tick_;
        for (int w = 0; w < ROB_LANES; ++w)
            sensitive << p_req_i[w] << p_addr_i[w] << p_haddr_i[w];
    }
};

#endif // ROB_HPP
