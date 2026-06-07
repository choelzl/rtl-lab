# TDM architecture

Reference diagram: [tdm.png](../diagrams/tdm.png)

Time-Division-Multiplexed interconnect: instead of giving every request port its own spatial path into the banks, requests are **buffered per AGU, serialized by a round-robin arbiter onto one shared 8-wide port, address-mapped, and routed to the banks through a reused [crossbar](crossbar.md)** whose per-bank round-robin arbiter resolves any bank conflicts. The goal is to reach the same eight banks with one time-shared port instead of a full spatial fan-out.

**What the comparison measures:** for each architecture, the **percentage delay penalty caused by bank conflicts** relative to an ideal conflict-free execution of the same trace (extra cycles spent resolving conflicts vs. a run where every access completes without stalling). See [crossbar.md](crossbar.md) for the baseline.

> **Note on the flow.** All modules are SystemC, built with `make sim-sc`. Design modules live under `rtl/systemc/`, the harness under `tb/systemc/`. The TDM design **reuses `crossbar.hpp` and `bank.hpp` from the crossbar design unchanged**; the AGU is the TDM design's **own** driver, [agu_tdm.hpp](../../tb/systemc/agu_tdm.hpp) (started as a copy of `agu_crossbar.hpp`, to be specialized for the x-OBI group interface).

## Hierarchy

```
Testbench  (tb_top_tdm.cpp)
├── mem_0.log ──▶ AGU[0] (agu_tdm.hpp)
├── mem_1.log ──▶ AGU[1] (agu_tdm.hpp)
└── DUT  (top_tdm.cpp)
    ├── Buffer[0] (buf.hpp)         ◀── AGU[0]   (x-OBI, 4 words)
    ├── Buffer[1] (buf.hpp)         ◀── AGU[1]   (x-OBI, 4 words)
    ├── Arbiter – Round Robin (arbiter.hpp)   ── sel_req / sel_rsp
    ├── OBI Mux (x_obi_mux.hpp)     ◀── Buffer[0]/Buffer[1] (x-OBI, 8 words)   [mux request + demux response]
    ├── TDM Mux (tdm_mux.hpp)       ◀── AGU[0]/AGU[1]   (mapping parameters)
    ├── TDM Mapping Function (tdm.hpp)         ── base + params → 8 (bank,row) placements
    ├── Crossbar 8×N_BANK (crossbar.hpp)       ── decodes bank/row, per-bank round-robin arbiter
    └── Bank[0..N_BANK-1] (bank.hpp)
```

> **Note:** the diagram matches this datapath — reused crossbar, the TDM mux carrying the mapping parameters, the mapping emitting addresses, and no conflicts-checker / buffer-demux.

## Modules

| Block                 | File             | Role                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| --------------------- | ---------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Testbench             | `tb_top_tdm.cpp` | `sc_main` harness. Reads `mem_0.log` / `mem_1.log`, instantiates the DUT and the `N_AGU` AGUs, runs to completion, prints statistics. Mirrors `tb_top_crossbar.cpp`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| AGU[0], AGU[1]        | `agu_tdm.hpp`    | Address Generation Units (x-OBI managers). The TDM design's **own** driver. Each parses the trace's first line for the kernel-wide mapping parameters (driven to the TDM mux), then replays its CSV trace, issuing a group of `N_REQ = 4` words per cycle as one x-OBI sub-request (single base address).                                                                                                                                                                                                                                                                                                                                                               |
| DUT                   | `top_tdm.cpp`    | Device under test: the buffers + arbiter + OBI mux + mapping + the reused crossbar + eight banks. Exposes the AGU-facing x-OBI ports the harness attaches the AGUs to.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |
| Buffer[0], Buffer[1]  | `buf.hpp`        | One per AGU. An x-OBI width converter (4-wide AGU side ↔ 8-wide memory side) with a **3-stage pipeline** — FILL → MEM (issue to the crossbar, partial issue under conflict, receive) → output FIFO (deliver `N_REQ`/cycle). **Read prefetch:** a read group fires the whole 8-word read on its *first* sub-request; the pair's other sub-request is absorbed (served from that read). **Writes** gather both sub-requests then issue. FILL forms the next group while MEM issues the current, so both reads and writes sustain **one sub-request/cycle** with no conflict; a conflict back-pressures through the pipeline (`a_gnt` = "FILL can take this sub-request"). |
| Arbiter – Round Robin | `arbiter.hpp`    | Free-running counter granting a **time slot to one AGU per cycle** (each served every `N_AGU` cycles). Emits `sel_req` (request path) and `sel_rsp` (= `sel_req` delayed one cycle, response path); both steer the OBI mux.                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| OBI Mux               | `x_obi_mux.hpp`  | x-OBI mux **and** demux: forwards the `sel_req` buffer's 8-word request onto the shared port, and routes the response back to the `sel_rsp` buffer. Combinational.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| TDM Mux               | `tdm_mux.hpp`    | Selects the active AGU's **mapping parameters** (`num_banks`, `bank_width`, `R`, `C`, `L`, `store_mode`) by the arbiter's `sel_req`. Combinational; not an OBI path.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| TDM Mapping Function  | `tdm.hpp`        | For each of the 8 group words (logical address `base + w·WORD_BYTES`) computes a `(bank_id, row_id)` placement with an XOR-skewed banking scheme (full specification in [map_func.md](map_func.md)), then emits the **8 single-word OBI requests** at the re-encoded byte address `(row_id·N_BANK + bank_id)·WORD_BYTES` — so the reused crossbar's word-interleave decode recovers exactly that `(bank_id, row_id)`. Combinational. A bank collision (≥2 words sharing a `bank_id`) is left for the crossbar's per-bank arbiter to serialize — the source of the conflict penalty. `bank_id` is 5-bit, so **`N_BANK` must be ≥ 32**.                                   |
| Crossbar 8×N_BANK     | `crossbar.hpp`   | **Reused from the crossbar design.** Decodes bank/row by word-interleave, routes each of the 8 requests to its bank, and **arbitrates same-bank collisions round-robin (one grant per bank per cycle)** — this is the TDM's conflict resolver. Steers each bank's response (one cycle later, via per-bank owner registers) back to the winning port.                                                                                                                                                                                                                                                                                                                    |
| Bank[0..N_BANK-1]     | `bank.hpp`       | **Reused from the crossbar design.** `N_BANK` single-port OBI RAM banks, 1-cycle latency, 1 word per access.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            |

## Interfaces & data flow

- **Protocol:** [x-OBI](x_obi.md) (multi-word) on the AGU/Buffer/Mux/TDM datapath; **single-word [OBI](obi.md)** from the mapping into the crossbar (8 manager ports) and from the crossbar to each bank. 32-bit (1 word) per bank access.
- **Word counts:** AGU → Buffer = **4 words** (x-OBI); Buffer → OBI Mux → TDM = **8 words** (x-OBI, one base address); TDM Mux → TDM = the AGU's **mapping parameters**; TDM → Crossbar = **8 single-word OBI requests** (re-encoded `(bank,row)` addresses); Crossbar → Banks = **1 word** each.

End-to-end pipeline (one AGU served per cycle, alternating):

1. The Arbiter grants this cycle's slot (`sel_req`) to one AGU.
2. That AGU's Buffer presents its 8-word group (one base address) over x-OBI.
3. The OBI Mux forwards the selected buffer's group to the mapping function.
4. `tdm.hpp` maps each of the 8 logical addresses (`base + w·WORD_BYTES`) to a `(bank, row)` placement (XOR-skew, parameters selected by `tdm_mux`) and drives the re-encoded addresses as 8 OBI requests into the crossbar.
5. The Crossbar decodes bank/row and arbitrates per bank: words on distinct banks are all granted; if **two or more words hit the same bank**, one wins (round-robin) and the others' `gnt` stays low. The buffer leaves those words **unsecured** and re-requests them in a later slot — the source of the conflict penalty.
6. Each bank's response returns one cycle later; the crossbar steers it back (owner register) and the OBI Mux routes it (via `sel_rsp`) to the originating buffer, which delivers the whole group to its AGU once complete.

## Addressing, traces & penalty model

- **Stimuli:** `mem_N.log` (under [tb/stimuli/](../../tb/stimuli/)) is the **same CSV** as the crossbar, with one extra **first line** carrying the kernel-wide mapping parameters as `num_banks,bank_width,C,R,L,store_mode` (`store_mode` is the integer index, 0..14). The access rows follow — an `addr,we,data` header then one row per access (hex byte address; `1` = write / `0` = read; write value, empty on reads). The crossbar AGU skips the parameter line; only the TDM AGU consumes it.
- **Banking:** the mapping emits the re-encoded address `(row_id·N_BANK + bank_id)·WORD_BYTES`, and the crossbar then decodes `bank = (addr/WORD_BYTES) % N_BANK`, `row = (addr/WORD_BYTES) / N_BANK` — recovering exactly the mapping's `(bank_id, row_id)`. So the TDM controls the bank placement (via the XOR-skew of [map_func.md](map_func.md)), while the crossbar baseline uses the raw addresses (plain word-interleave).
- **Conflict source:** a conflict is when **≥2 of a slot's 8 mapped words share a `bank_id`**; the crossbar's per-bank arbiter serializes them and the buffer re-requests the losers over later slots. Whether conflicts arise depends on the mapping parameters and the access pattern; a placement that scatters a slot's 8 words onto 8 distinct banks runs at ~0% penalty (and a multi-row trace keeps same-bank collisions on **different rows**, so they serialize without aliasing).
- **Penalty:** the stall cycles the buffer spends re-requesting conflicted words over additional round-robin slots, reported as the **% delay penalty** vs. an ideal conflict-free run. The exact per-conflict cost follows from the slot cadence and is measured by the harness.

## Resolved

- **Datapath:** AGU → `buf` → `x_obi_mux` → `tdm` → `crossbar` → banks; responses reverse via the crossbar owner registers + `x_obi_mux` `sel_rsp`. ✔
- **Conflict resolver:** the **reused `crossbar.hpp`** (per-bank round-robin arbiter) — no separate conflicts checker. ✔
- **No buffer demux:** responses route back through the crossbar + OBI mux; the buffer learns which words completed from its own `gnt`/`rvalid`. ✔
- **Mapping:** `tdm.hpp` places each word with an XOR-skewed banking scheme (parameters `num_banks/bank_width/R/C/L/store_mode` from the trace's first line, selected per slot by `tdm_mux`) and re-encodes `(bank,row)` as the crossbar's word-interleave address. ✔
- **x-OBI:** multi-word group protocol on the AGU/buffer/mux/TDM links (see [x_obi.md](x_obi.md)). ✔
- **Sizing:** `N_BANK = 32` (the mapping's `bank_id` is 5-bit, so N_BANK ≥ 32), `N_REQ = 4`, buffer 4↔8 width convert, crossbar 8×32. ✔
- **Reuse:** `crossbar.hpp` and `bank.hpp` reused unchanged from the crossbar design. ✔
- **Metric:** % delay penalty from conflicts vs. ideal conflict-free execution. ✔
- **AGU:** `agu_tdm.hpp` drives the x-OBI group (one base + `req[4]`/`wdata[4]`, stride out), grant-based (issues a group every cycle), collecting responses in order. ✔
- **Simulation & correctness:** the full datapath runs (`make sim-sc TOP_LEVEL=top_tdm`); read-back is correct — each read returns its written value. ✔
- **Throughput (req every cycle):** with no conflict the buffer pipeline sustains **1 group/cycle** — same slope as the crossbar (measured TDM 10 cycles at 4 groups). TDM = `G_max + 6` (a fixed +6 pipeline fill); `kPipeFill = 6` in the harness ⇒ conflict-free run reads 0%. ✔
- **Mapping integrated & verified:** the XOR-skew mapping runs end-to-end (`make sim-sc TOP_LEVEL=top_tdm`, default `N_BANK=32`); read-back is correct (the mapping is a bijection over the trace's addresses) and the sample trace (`num_banks=32,bank_width=4,C=R=4,L=8,store_mode=0`) scatters each slot's 8 words onto 8 distinct banks ⇒ 0% penalty. ✔

## Open / to investigate

1. **Adverse traces / parameter sweep** — pick mapping parameters and access patterns that place ≥2 of a slot's words on the same bank (across multiple rows, so no aliasing) to actually exercise the conflict penalty, and compare against the crossbar baseline on the same trace.
2. **Fair comparison sizing** — the mapping needs `N_BANK ≥ 32`; both tops now default to `N_BANK=32`, so the crossbar baseline and the TDM design run at the same `N_BANK` out of the box for an apples-to-apples penalty comparison.
3. **`bank_width` semantics** — reproduced faithfully as `e = log2(bank_width)` (see [map_func.md](map_func.md)); confirm the intended unit/value for the real kernels.
4. **Calibration:** `kPipeFill = 6` is measured for the default `N_AGU=2` config; re-measure if the config / pipeline depth changes.
```