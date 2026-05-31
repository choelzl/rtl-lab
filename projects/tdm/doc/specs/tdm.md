# TDM architecture

Reference diagram: [tdm.png](../diagrams/tdm.png)

Time-Division-Multiplexed interconnect: instead of a full [crossbar](crossbar.md), requests are **buffered, serialized by a round-robin arbiter, mapped to (bank, row) by a conflict-minimizing mapping function, and conflict-checked** before reaching the banks. The goal is to reach the same eight banks with far less interconnect hardware, trading the crossbar's spatial parallelism for time multiplexing.

**What the comparison measures:** for each architecture, the **percentage delay penalty caused by bank conflicts** relative to an ideal conflict-free execution of the same trace (extra cycles spent resolving conflicts vs. a run where every access completes without stalling). See [crossbar.md](crossbar.md) for the baseline. The two designs differ in *how* a word lands in a bank — the crossbar uses a fixed word-interleave, the TDM uses a mapping function that tries to spread a group's words across banks — so this number captures the cost of each scheme.

> **Note on the flow.** All modules are SystemC (`.hpp`/`.cpp`, `sc_main` testbench), so this design targets the `make sim-sc` flow. Design modules live under `rtl/systemc/`, the harness under `tb/systemc/`. The bank ([bank.hpp](../../rtl/systemc/bank.hpp)) is **shared with the crossbar** unchanged; the AGU is the TDM design's **own** driver, [tdm_agu.hpp](../../tb/systemc/tdm_agu.hpp) — it currently **starts as a copy** of the crossbar's `cros_agu.hpp` and is to be specialized (single base address per group, see below).

## Hierarchy

```
Testbench  (tb_top_tdm.cpp)
├── mem_0.log ──▶ AGU[0] (tdm_agu.hpp)
├── mem_1.log ──▶ AGU[1] (tdm_agu.hpp)
└── DUT  (top_tdm.cpp)
    ├── Buffer[0] (buf.hpp)        ◀── AGU[0]   (4 words)
    ├── Buffer[1] (buf.hpp)        ◀── AGU[1]   (4 words)
    ├── OBI Mux (obi_mux.hpp)      ◀── Buffer[0]/Buffer[1] (8 words each)   [OBI request mux]
    ├── Arbiter – Round Robin (arbiter.hpp)   ── drives obi_mux / tdm_mux / buf_demux
    ├── TDM Mux (tdm_mux.hpp)      ◀── AGU[0]/AGU[1]                        [access-pattern config mux]
    ├── TDM Mapping Function (tdm.hpp)         ── emits Mapping Vector (16 elements)
    ├── Conflicts Checker (conf_check.hpp)
    ├── Buffer Demux (buf_demux.hpp)  ──▶ Buffer[0]/Buffer[1]               [per-word completion ack]
    └── Bank[0..7] (bank.hpp)
```

> **Diagram label:** the image still labels the response block **"Buffer Mux (buf_mux.hpp)"**. That is a mistake — it is the **Buffer Demux (buf_demux.hpp)** described here; the PNG label predates this correction.

## Modules

| Block                 | File             | Role                                                                                                                                                                                                                                                                                                                                                              |
| --------------------- | ---------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Testbench             | `tb_top_tdm.cpp` | `sc_main` harness. Reads `mem_0.log` / `mem_1.log`, instantiates the DUT and the `N_AGU` AGUs, runs to completion, prints statistics. Mirrors `tb_top_crossbar.cpp`.                                                                                                                                                                                              |
| AGU[0], AGU[1]        | `tdm_agu.hpp`    | Address Generation Units (OBI managers). The TDM design's **own** driver — currently a copy of the crossbar's `cros_agu.hpp`, to be specialized. Each replays its CSV trace, issuing a group of `N_REQ = 4` words per cycle. In the TDM path the group is carried forward as a **base address** (the words of a group share one base), so the mapping function works from one base address per group rather than four independent ports.       |
| DUT                   | `top_tdm.cpp`    | Device under test: the full TDM datapath plus the eight banks. Exposes the manager-side OBI ports the harness attaches the AGUs to (mirrors `top_crossbar.cpp`).                                                                                                                                                                                                  |
| Buffer[0], Buffer[1]  | `buf.hpp`        | One **8-deep** buffer per AGU. The AGU pushes 4 words/cycle; the buffer **accumulates two cycles' worth (8 words)** and drains them in the single slot the arbiter grants this AGU, so the AGU never stalls during the cycle its slot belongs to the *other* AGU. Tracks which words remain outstanding after a conflict and **re-requests them in the AGU's next slot**. |
| OBI Mux               | `obi_mux.hpp`    | Selects the active buffer's **8-word** OBI request (one base address) and forwards it to the mapping function; controlled by the arbiter. Output: **8 words** over OBI.                                                                                                                                                                                            |
| Arbiter – Round Robin | `arbiter.hpp`    | Grants a **time slot to one AGU per cycle**, alternating round-robin between the two AGUs (so each AGU is served every 2 cycles). Drives the OBI mux, the TDM-config mux, and the buffer demux (the latter steered by the slot whose response is now returning).                                                                                                    |
| TDM Mux               | `tdm_mux.hpp`    | A **separate** mux (not an OBI path): selects the `AGU[0]`/`AGU[1]` **access-pattern configuration** that parametrizes the mapping function (stride/mode/descriptor — see *Open*); controlled by the arbiter.                                                                                                                                                      |
| TDM Mapping Function  | `tdm.hpp`        | Core scheduler. From the group's **base address** (OBI mux) and its **access pattern** (TDM mux), computes where each of the 8 words lands and emits the **Mapping Vector (16 elements)**: `{BID0, RID0, … , BID7, RID7}` (bank ID + row ID per word). It places words **to minimize same-bank collisions**; this is **not** the crossbar's fixed word-interleave. **Internals are a black box for now.** |
| Conflicts Checker     | `conf_check.hpp` | Reads the 16-element mapping vector and detects banks targeted by more than one of the slot's 8 words. It issues the **collision-free subset** to the banks this slot; words that still share a bank are **left outstanding** for the buffer to re-request in the AGU's **next round-robin slot** — the source of the 2-cycle TDM penalty.                          |
| Buffer Demux          | `buf_demux.hpp`  | Response-side fan-out (arbiter-steered). Routes each issued word's **completion ack** (OBI `rvalid`/grant, plus `rdata` on reads) back to the **owning buffer**, telling it exactly which of its words were actually read/written this slot. The buffer clears completed words, holds the conflicted ones (so it can wait and re-request), and forwards read data up to its AGU. |
| Bank[0..7]            | `bank.hpp`       | Eight memory banks (OBI subordinates), **shared with the crossbar**: single-port RAM, 1-cycle access latency, 1 word (32-bit) per access. They receive a **bank-local** address (the mapping function supplies the row ID).                                                                                                                                        |

## Interfaces & data flow

- **Protocol:** simplified single-channel OBI on the request/response links — see [obi.md](obi.md). 32-bit (1 word) at each bank.
- **Word counts on the diagram:** AGU → Buffer = **4 words**; Buffer → OBI Mux = **8 words**; OBI Mux → TDM Mapping Function = **8 words**; TDM Mapping Function → Conflicts Checker = **Mapping Vector (16 elements)** = 8 × `{bank ID, row ID}`; Conflicts Checker → Banks = **1 word** each (up to 8 banks); Banks → Buffer Demux → Buffers = **per-word completion ack** (+ `rdata` on reads).

End-to-end pipeline (per slot, one AGU served per cycle, alternating):

1. The Arbiter grants this cycle's slot to one AGU.
2. That AGU's Buffer presents its group — **8 words** (two cycles' accumulated production) as one base address over OBI.
3. The OBI Mux forwards the 8-word request and the TDM Mux forwards the same AGU's access-pattern config to the mapping function.
4. The TDM Mapping Function (black box) maps the 8 words to `{bank ID, row ID}` each — **spreading them across banks to minimize collisions** — and emits the 16-element Mapping Vector.
5. The Conflicts Checker issues the **collision-free words this slot**; any words the mapping still had to place on an already-taken bank stay outstanding and are re-requested in the AGU's **next slot** (round-robin → 2 cycles later) → **2-cycle penalty** for that group.
6. The Buffer Demux acks the originating buffer which words completed (and returns read data); the buffer drains those, keeps the conflicted ones, and re-issues them on its next slot.

So a conflicting group's fetch is **split across two of that AGU's slots**. The accumulated penalty cycles are reported as the **% delay penalty** for this architecture vs. a conflict-free run.

## Addressing, traces & penalty model

- **Traces:** `mem_N.log` (under [tb/traces/](../../tb/traces/)) is the **same CSV** as the crossbar — `addr,we,data` (hex byte address; `1` = write / `0` = read; write value, empty on reads), one row per access — `tdm_agu.hpp` started as a copy of the crossbar's `cros_agu.hpp`, so the trace format is identical. R/W and write data are carried **per access** in the trace.
- **Banking (key difference from the crossbar):** the (bank, row) of each word is **decided by the TDM mapping function**, not by a fixed word-interleave decode. Given a group's base address and its access pattern, the mapping spreads the words over banks to avoid collisions. The mapping must be **deterministic and consistent** so that a write and a later read of the same address resolve to the same bank+row (read-back correctness).
- **Conflict source:** the **residual same-bank collisions** within a slot's 8 words that the mapping function could not avoid for the given access pattern. A friendly pattern maps to 8 distinct banks (no penalty); an adverse pattern (e.g. a stride that funnels several words onto one bank) forces collisions the checker must serialize. Conflicts are therefore **intra-slot / intra-group** by construction (only one AGU is active per slot); a future variant may explore other sources, so the design should not hard-wire "one AGU per slot."
- **Penalty:** **2 cycles per deferral** (vs. 1 cycle for the crossbar): the collision-free words go out in the current slot and a conflicted word waits for the AGU's next round-robin slot. If `k` of a group's words land on the same bank, the bank serves one per slot, so they serialize over `k` of that AGU's slots → `2·(k − 1)` stall cycles. The penalty **scales with the worst-case bank over-subscription** in the group.

## Resolved

- **Sizing — aligned to the crossbar baseline / diagram:** 8 banks (`N_BANK = 8`), 4 words/AGU/cycle (`N_REQ = 4`), buffer drains **8 words/slot**, Mapping Vector **16 elements**. ✔
- **Buffer:** depth 8; accumulates two cycles' production (8 words) to cover the AGU's skipped slot; tracks outstanding words after a conflict and re-requests them next slot. ✔
- **Arbiter:** round-robin, one AGU per cycle (each served every 2 cycles); drives `obi_mux`, `tdm_mux`, `buf_demux`. ✔
- **Mapping Vector (16 elements):** 8 words × `{bank ID, row ID}`. ✔
- **Bank placement:** decided by the TDM mapping function to minimize collisions — **not** fixed word-interleave (that is the crossbar's scheme). ✔
- **Conflict source:** residual same-bank collisions among a slot's 8 words after the mapping (intra-slot); extensible later. ✔
- **Conflict penalty:** 2 cycles per deferral (split fetch across two of the AGU's slots), scaling with worst-case bank over-subscription. ✔
- **Response/feedback:** **Buffer Demux** (`buf_demux.hpp`) acks each buffer which of its words actually read/written this slot, so it can wait on and re-request the conflicted ones. ✔
- **Muxes — three distinct modules:** `obi_mux.hpp` (OBI request path), `tdm_mux.hpp` (access-pattern config), `buf_demux.hpp` (per-word completion ack back to buffers). ✔
- **Metric:** % delay penalty from conflicts vs. ideal conflict-free execution. ✔
- **Banks / traces:** banks shared with the crossbar (`bank.hpp`, single-port 1-cycle OBI RAM); trace format identical (CSV `addr,we,data`). **AGU:** the TDM design's own `tdm_agu.hpp` (starts as a copy of `cros_agu.hpp`, to be specialized for the single-base-address group). ✔

## Open / to investigate

1. **TDM Mapping Function (`tdm.hpp`) internals** — the exact `(base address, access pattern) → {bank, row}` placement is **deliberately a black box for now**. This is the part that distinguishes TDM and sets its conflict rate. To be defined later.
2. **Access-pattern config** — what descriptor the AGU passes through `tdm_mux` (stride? per-word offsets? mode bits?) and how the mapping consumes it. To be defined together with the mapping.
3. **Read-back consistency** — confirm, once the mapping is specified, that it is deterministic so a write and a later read of the same address resolve to the same bank+row.
4. **Ideal-cycle constant for the harness** — the TDM pipeline is deeper than the crossbar's and the round-robin halves each AGU's issue rate, so the conflict-free `ideal` (and the fixed pipeline-fill constant) must be **re-derived / measured** for `tb_top_tdm`; the crossbar's `G_max + 2` does not transfer directly. See [crossbar.md](crossbar.md) and the [README](../../README.md#statistics--the-conflict-metric).
```