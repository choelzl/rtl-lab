# TDM architecture

Reference diagram: [tdm.png](../diagrams/tdm.png)

Time-Division-Multiplexed interconnect: instead of giving every request port its own spatial path into the banks, requests are **buffered per AGU, serialized by a round-robin arbiter onto one shared 8-wide port, address-mapped, and routed to the banks through a reused [crossbar](crossbar.md)** whose per-bank round-robin arbiter resolves any bank conflicts. The goal is to reach the same eight banks with one time-shared port instead of a full spatial fan-out.

**What the comparison measures:** for each architecture, the **percentage delay penalty caused by bank conflicts** relative to an ideal conflict-free execution of the same trace (extra cycles spent resolving conflicts vs. a run where every access completes without stalling). See [crossbar.md](crossbar.md) for the baseline.

> **Note on the flow.** All modules are SystemC, built with `make sim-sc`. Design modules live under `rtl/systemc/`, the harness under `tb/systemc/`. The TDM design **reuses `crossbar.hpp` and `bank.hpp` from the crossbar design unchanged**; the AGU is the TDM design's **own** driver, [tdm_agu.hpp](../../tb/systemc/tdm_agu.hpp) (started as a copy of `cros_agu.hpp`, to be specialized for the x-OBI group interface).

## Hierarchy

```
Testbench  (tb_top_tdm.cpp)
├── mem_0.log ──▶ AGU[0] (tdm_agu.hpp)
├── mem_1.log ──▶ AGU[1] (tdm_agu.hpp)
└── DUT  (top_tdm.cpp)
    ├── Buffer[0] (buf.hpp)         ◀── AGU[0]   (x-OBI, 4 words)
    ├── Buffer[1] (buf.hpp)         ◀── AGU[1]   (x-OBI, 4 words)
    ├── Arbiter – Round Robin (arbiter.hpp)   ── sel_req / sel_rsp
    ├── OBI Mux (x_obi_mux.hpp)     ◀── Buffer[0]/Buffer[1] (x-OBI, 8 words)   [mux request + demux response]
    ├── TDM Mux (tdm_mux.hpp)       ◀── AGU[0]/AGU[1]   (stride config)
    ├── TDM Mapping Function (tdm.hpp)         ── base + stride → 8 word addresses
    ├── Crossbar 8x8 (crossbar.hpp)            ── decodes bank/row, per-bank round-robin arbiter
    └── Bank[0..7] (bank.hpp)
```

> **Note:** the diagram matches this v1 datapath — reused crossbar, the TDM mux carrying the stride config, the mapping emitting addresses, and no conflicts-checker / buffer-demux.

## Modules

| Block                 | File             | Role                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| --------------------- | ---------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Testbench             | `tb_top_tdm.cpp` | `sc_main` harness. Reads `mem_0.log` / `mem_1.log`, instantiates the DUT and the `N_AGU` AGUs, runs to completion, prints statistics. Mirrors `tb_top_crossbar.cpp`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| AGU[0], AGU[1]        | `tdm_agu.hpp`    | Address Generation Units (x-OBI managers). The TDM design's **own** driver (a specialization of `cros_agu.hpp`). Each replays its CSV trace, issuing a group of `N_REQ = 4` words per cycle as one x-OBI sub-request (single base address). **To be built last.**                                                                                                                                                                                                                                                                                                                                                                                                       |
| DUT                   | `top_tdm.cpp`    | Device under test: the buffers + arbiter + OBI mux + mapping + the reused crossbar + eight banks. Exposes the AGU-facing x-OBI ports the harness attaches the AGUs to.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| Buffer[0], Buffer[1]  | `buf.hpp`        | One per AGU. An x-OBI width converter (4-wide AGU side ↔ 8-wide memory side) with a **3-stage pipeline** — FILL → MEM (issue to the crossbar, partial issue under conflict, receive) → output FIFO (deliver `N_REQ`/cycle). **Read prefetch:** a read group fires the whole 8-word read on its *first* sub-request; the pair's other sub-request is absorbed (served from that read). **Writes** gather both sub-requests then issue. FILL forms the next group while MEM issues the current, so both reads and writes sustain **one sub-request/cycle** with no conflict; a conflict back-pressures through the pipeline (`a_gnt` = "FILL can take this sub-request"). |
| Arbiter – Round Robin | `arbiter.hpp`    | Free-running counter granting a **time slot to one AGU per cycle** (each served every `N_AGU` cycles). Emits `sel_req` (request path) and `sel_rsp` (= `sel_req` delayed one cycle, response path); both steer the OBI mux.                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| OBI Mux               | `x_obi_mux.hpp`  | x-OBI mux **and** demux: forwards the `sel_req` buffer's 8-word request onto the shared port, and routes the response back to the `sel_rsp` buffer. Combinational.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| TDM Mux               | `tdm_mux.hpp`    | Selects the active AGU's TDM-mapping config by the arbiter's `sel_req` — **v1: just the address stride**. Combinational; not an OBI path.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |
| TDM Mapping Function  | `tdm.hpp`        | **v1 (simple):** generates **8 word addresses** `base + w·stride` (stride from the TDM mux; `0x4` = consecutive words) and adapts the x-OBI group to 8 single-word OBI manager ports. Combinational. The real conflict-minimizing mapping (arbitrary per-word placement) replaces the address generation later.                                                                                                                                                                                                                                                                                                                                                         |
| Crossbar 8x8          | `crossbar.hpp`   | **Reused from the crossbar design.** Decodes bank/row by word-interleave, routes each of the 8 requests to its bank, and **arbitrates same-bank collisions round-robin (one grant per bank per cycle)** — this is the TDM's conflict resolver. Steers each bank's response (one cycle later, via per-bank owner registers) back to the winning port.                                                                                                                                                                                                                                                                                                                    |
| Bank[0..7]            | `bank.hpp`       | **Reused from the crossbar design.** Eight single-port OBI RAM banks, 1-cycle latency, 1 word per access.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               |

## Interfaces & data flow

- **Protocol:** [x-OBI](x_obi.md) (multi-word) on the AGU/Buffer/Mux/TDM datapath; **single-word [OBI](obi.md)** from the mapping into the crossbar (8 manager ports) and from the crossbar to each bank. 32-bit (1 word) per bank access.
- **Word counts:** AGU → Buffer = **4 words** (x-OBI); Buffer → OBI Mux → TDM = **8 words** (x-OBI, one base address); TDM Mux → TDM = the AGU's **stride**; TDM → Crossbar = **8 single-word OBI requests** (addresses `base + w·stride`); Crossbar → Banks = **1 word** each.

End-to-end pipeline (one AGU served per cycle, alternating):

1. The Arbiter grants this cycle's slot (`sel_req`) to one AGU.
2. That AGU's Buffer presents its 8-word group (one base address) over x-OBI.
3. The OBI Mux forwards the selected buffer's group to the mapping function.
4. `tdm.hpp` generates the 8 word addresses (`base + w·stride`, stride selected by `tdm_mux`) and drives them as 8 OBI requests into the crossbar.
5. The Crossbar decodes bank/row and arbitrates per bank: words on distinct banks are all granted; if **two or more words hit the same bank**, one wins (round-robin) and the others' `gnt` stays low. The buffer leaves those words **unsecured** and re-requests them in a later slot — the source of the conflict penalty.
6. Each bank's response returns one cycle later; the crossbar steers it back (owner register) and the OBI Mux routes it (via `sel_rsp`) to the originating buffer, which delivers the whole group to its AGU once complete.

## Addressing, traces & penalty model

- **Traces:** `mem_N.log` (under [tb/traces/](../../tb/traces/)) is the **same CSV** as the crossbar — `addr,we,data` (hex byte address; `1` = write / `0` = read; write value, empty on reads), one row per access.
- **Banking:** the crossbar decodes `bank = (addr/WORD_BYTES) % N_BANK`, `row = (addr/WORD_BYTES) / N_BANK` from the addresses the mapping produces (same word-interleave as the crossbar baseline). What differs between the designs is only *which* addresses reach the crossbar and *how many ports at once* — the TDM time-shares one 8-wide port.
- **Conflict source:** a conflict is when **≥2 of a slot's 8 mapped addresses decode to the same bank**; the crossbar's per-bank arbiter serializes them and the buffer re-requests the losers over later slots. With the **v1 consecutive mapping**, 8 consecutive words interleave onto 8 distinct banks → **no conflict** (v1 validates the datapath and read-back at ~0% penalty). Conflicts appear once the mapping is non-trivial (or the access pattern is adverse).
- **Penalty:** the stall cycles the buffer spends re-requesting conflicted words over additional round-robin slots, reported as the **% delay penalty** vs. an ideal conflict-free run. The exact per-conflict cost follows from the slot cadence and is measured by the harness.

## Resolved

- **Datapath:** AGU → `buf` → `x_obi_mux` → `tdm` → `crossbar` → banks; responses reverse via the crossbar owner registers + `x_obi_mux` `sel_rsp`. ✔
- **Conflict resolver:** the **reused `crossbar.hpp`** (per-bank round-robin arbiter) — no separate conflicts checker. ✔
- **No buffer demux:** responses route back through the crossbar + OBI mux; the buffer learns which words completed from its own `gnt`/`rvalid`. ✔
- **v1 mapping:** `tdm.hpp` generates 8 consecutive addresses (`base + w·WORD_BYTES`); the crossbar decodes bank/row. ✔
- **x-OBI:** multi-word group protocol on the AGU/buffer/mux/TDM links (see [x_obi.md](x_obi.md)). ✔
- **Sizing:** `N_BANK = 8`, `N_REQ = 4`, buffer 4↔8 width convert, crossbar 8×8. ✔
- **Reuse:** `crossbar.hpp` and `bank.hpp` reused unchanged from the crossbar design. ✔
- **Metric:** % delay penalty from conflicts vs. ideal conflict-free execution. ✔
- **AGU:** `tdm_agu.hpp` drives the x-OBI group (one base + `req[4]`/`wdata[4]`, stride out), grant-based (issues a group every cycle), collecting responses in order. ✔
- **Simulation & correctness:** the full datapath runs (`make sim-sc TOP_LEVEL=top_tdm`); read-back is correct — each read returns its written value. ✔
- **Throughput (req every cycle):** with no conflict the buffer pipeline sustains **1 group/cycle** — same slope as the crossbar (measured TDM 10/22 cycles at 4/16 groups vs crossbar 7/19). TDM = `G_max + 6` (a fixed +6 pipeline fill); `kPipeFill = 6` in the harness ⇒ conflict-free run reads 0%. ✔

## Open / to investigate

1. **Real TDM mapping** — replace `tdm.hpp`'s consecutive address generation with the conflict-minimizing mapping (arbitrary per-word placement from base + access pattern). Must be deterministic so a write and a later read of the same address resolve to the same bank+row. **This is what actually exercises the conflict-penalty comparison** — v1 consecutive ⇒ zero conflicts.
2. **Richer access-pattern config** beyond the v1 stride (delivered via `tdm_mux`), added with the real mapping.
3. **Calibration:** `kPipeFill = 6` is measured for the default `N_AGU=2` config; re-measure if the config / pipeline depth changes.
```