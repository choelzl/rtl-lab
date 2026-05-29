# TDM architecture

Reference diagram: [tdm.png](../diagrams/tdm.png)

Time-Division-Multiplexed interconnect: instead of a full [crossbar](crossbar.md), requests are **buffered, serialized by a round-robin arbiter, mapped to (bank, row), and conflict-checked** before reaching the banks. The goal is to reach the same four banks with far less interconnect hardware, trading spatial parallelism for time multiplexing.

**What the comparison measures:** for each architecture, the **percentage delay penalty caused by bank conflicts** relative to an ideal conflict-free execution of the same trace (extra cycles spent resolving conflicts vs. a run where every access completes in one cycle). See [crossbar.md](crossbar.md) for the baseline.

> **Note on the flow.** All modules are SystemC (`.hpp`/`.cpp`, `sc_main` testbench), so this design targets the `make sim-sc` flow. Design modules would live under `rtl/systemc/`, the harness under `tb/systemc/`.

## Hierarchy

```
Testbench  (tb_top_tdm.cpp)
├── mem_0.log ──▶ AGU[0] (agu.hpp)
├── mem_1.log ──▶ AGU[1] (agu.hpp)
└── DUT  (top_tdm.cpp)
    ├── Buffer[0] (buf.hpp)      ◀── AGU[0]   (2 words)
    ├── Buffer[1] (buf.hpp)      ◀── AGU[1]   (2 words)
    ├── Mux (mux.hpp)            ◀── Buffer[0]/Buffer[1] (4 words each)   [OBI request mux]
    ├── Arbiter – Round Robin (arbiter.hpp)   ── drives the OBI mux/demux + config mux
    ├── Mux (mux.hpp)            ◀── AGU[0]/AGU[1]                        [TDM-config mux]
    ├── TDM Mapping Function (tdm.hpp)         ── emits Mapping Vector (8 elements)
    ├── Conflicts Checker (conf_check.hpp)
    ├── Demux (demux.hpp)        ──▶ AGU[0]/AGU[1]
    └── Bank[0..3] (bank.hpp)
```

## Modules

| Block                 | File             | Role                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| --------------------- | ---------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Testbench             | `tb_top_tdm.cpp` | Top-level harness. Reads `mem_0.log` / `mem_1.log` and drives the two AGUs; instantiates the DUT.                                                                                                                                                                                                                                                                                                                                                                                         |
| AGU[0], AGU[1]        | `agu.hpp`        | Address Generation Units. Each reads **2 consecutive addresses** (rows) per cycle from its trace. Because the two words are consecutive by design, in the TDM path they are issued as a **single request** (one base address) rather than two separate ports.                                                                                                                                                                                                                             |
| DUT                   | `top_tdm.cpp`    | Device under test: the full TDM datapath plus the four banks.                                                                                                                                                                                                                                                                                                                                                                                                                             |
| Buffer[0], Buffer[1]  | `buf.hpp`        | One **4-deep** buffer per AGU. On the AGU's slot it **prefetches 4 consecutive words** (one base address) — enough for two cycles of consumption — so that during the next cycle (when the slot belongs to the *other* AGU) this AGU does not stall. Tracks which words remain outstanding after a conflict and **re-requests them in the AGU's next slot**.                                                                                                                              |
| Mux (OBI request)     | `mux.hpp`        | The **OBI mux** half of an OBI mux/demux pair (the Demux is its response-side half). Selects the active buffer's 4-word OBI request (single base address); controlled by the arbiter. Output: **4 words** over OBI to the mapping function.                                                                                                                                                                                                                                               |
| Arbiter – Round Robin | `arbiter.hpp`    | Grants a **time slot to one AGU per cycle**, alternating round-robin between the two AGUs (so each AGU is served every 2 cycles). Drives the OBI mux/demux and the TDM-config mux.                                                                                                                                                                                                                                                                                                        |
| Mux (TDM config)      | `mux.hpp`        | A **separate** mux (not an OBI path): selects the `AGU[0]`/`AGU[1]` **configuration signals** that parametrize the TDM mapping function; controlled by the arbiter.                                                                                                                                                                                                                                                                                                                       |
| TDM Mapping Function  | `tdm.hpp`        | Core scheduler. From the single base address of the 4 consecutive words, computes where each word lands and emits the **Mapping Vector (8 elements)**: `{BID0, RID0, BID1, RID1, BID2, RID2, BID3, RID3}` (bank ID + row ID per word). **Internal mapping is treated as a black box for now.**                                                                                                                                                                                            |
| Conflicts Checker     | `conf_check.hpp` | Reads the 8-element mapping vector and checks for bank conflicts. It issues the **non-conflicting subset** of the 4 words to the banks in the current slot; the words that lost a conflict are **left outstanding** for the buffer to re-request in the AGU's **next round-robin slot** — the source of the 2-cycle TDM penalty. The buffer must learn which words completed: ideally inferred from the **OBI ack/grant** (to be verified) rather than a dedicated checker→buffer signal. |
| Demux                 | `demux.hpp`      | The **response-side half** of the OBI mux/demux: routes bank read responses back to `AGU[0]` / `AGU[1]`, steered by the arbiter's current selection.                                                                                                                                                                                                                                                                                                                                      |
| Bank[0..3]            | `bank.hpp`       | Four memory banks, 1 word (32-bit) per access.                                                                                                                                                                                                                                                                                                                                                                                                                                            |

## Interfaces & data flow

- **Protocol:** OBI on the main request/response links; 32-bit (1 word) at the banks.
- **Word counts on the diagram:** AGU → Buffer = **2 words**; Buffer → request Mux = **4 words**; request Mux → TDM Mapping Function = **4 words**; TDM Mapping Function → Conflicts Checker = **Mapping Vector (8 elements)** = 4 × `{bank ID, row ID}`; Conflicts Checker → Banks = **1 word** each.

End-to-end pipeline (per slot, one AGU served per cycle, alternating):

1. The Arbiter grants this cycle's slot to one AGU.
2. That AGU's Buffer prefetches 4 consecutive words (one base address) — two cycles' worth, so the AGU survives the cycle it is not granted a slot.
3. The OBI request Mux forwards the 4-word group and the TDM-config Mux forwards the same AGU's configuration signals to the mapping function.
4. The TDM Mapping Function (black box) maps the 4 words to `{bank ID, row ID}` each, emitting the 8-element Mapping Vector.
5. The Conflicts Checker issues the **non-conflicting words this slot**; words that lost a conflict stay outstanding and are re-requested in the AGU's **next slot** (round-robin → 2 cycles later) → **2-cycle penalty** for that group.
6. The Demux returns each bank response to the originating AGU.

So a conflicting group's fetch is **split across two of its slots**. The accumulated penalty cycles are reported as the **% delay penalty** for this architecture vs. a conflict-free run.

## Addressing, traces & penalty model

- **Traces:** `mem_N.log` is a single column of hexadecimal addresses, one per memory access. Each AGU reads **2 addresses (rows) per cycle**. Read-vs-write for the whole run is chosen in the testbench (uniform across all transactions in a simulation).
- **Banking:** word-interleaved across the 4 banks (consistent with the [crossbar](crossbar.md) baseline). The 2 words an AGU issues per cycle are consecutive, and the 4 words a buffer prefetches per slot are consecutive — so a group is carried as a single base address.
- **Conflict source (for now):** **only across the two AGUs** — an AGU's own consecutive, interleaved words never self-collide. Other use-cases (intra-group conflicts via the mapping) will be explored later, so the design should not hard-wire the "no self-conflict" assumption.
- **Penalty:** **2 cycles per conflict** between two masters (vs. 1 cycle for the crossbar): the non-conflicting words go out in the current slot and the conflicting ones wait for the AGU's next round-robin slot. The penalty **scales with the number of conflicting masters**.

## Resolved

- **Buffer:** depth 4; prefetches 4 consecutive words/slot to cover the AGU's skipped cycle. ✔
- **Mapping Vector (8 elements):** 4 words × `{bank ID, row ID}`. ✔
- **Arbiter:** round-robin, one AGU per cycle (each served every 2 cycles). ✔
- **Conflict penalty:** 2 cycles/conflict (split fetch across two slots), scaling with conflicting masters. ✔
- **Conflict source:** across the two AGUs for now (extensible later). ✔
- **Metric:** % delay penalty from conflicts vs. ideal conflict-free execution. ✔
- **Traces / addressing:** single-column hex; 2 rows/cycle/AGU; word-interleaved; R/W set per simulation. ✔
- **Muxes:** two distinct `mux.hpp` modules — an **OBI mux** (paired with `demux.hpp` for responses) and a separate **TDM-config mux** for the AGU configuration signals. ✔

## Open / to investigate

1. **TDM Mapping Function (`tdm.hpp`) internals** — the exact word→`{bank, row}` mapping is **deliberately a black box for now**. To be defined later.
2. **Conflict feedback to the buffer** — after a partial (conflicted) issue, the buffer must know which words are still outstanding. Check whether the **OBI ack/grant** already conveys this (preferred), so no dedicated checker→buffer signal is needed.
