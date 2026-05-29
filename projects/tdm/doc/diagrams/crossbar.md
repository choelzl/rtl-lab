# Crossbar architecture

Reference diagram: [crossbar.png](crossbar.png)

Baseline interconnect: a full **4×4 crossbar** that connects every request port to every memory bank in parallel (spatial interconnect, single cycle). This is the reference design the [TDM architecture](tdm.md) is compared against.

**What the comparison measures:** for each architecture, the **percentage delay penalty caused by bank conflicts** relative to an ideal conflict-free execution of the same trace (i.e. how many extra cycles conflicts cost vs. a run where every access completes in one cycle). The crossbar and the TDM design resolve conflicts differently, so this number captures the cost of each scheme.

> **Note on the flow.** All modules are SystemC (`.hpp`/`.cpp`, `sc_main` testbench), so this design targets the `make sim-sc` flow. Design modules would live under `rtl/systemc/`, the harness under `tb/systemc/`.

## Hierarchy

```
Testbench  (tb_top_crossbar.cpp)
├── mem_0.log ──▶ AGU[0] (agu.hpp)
├── mem_1.log ──▶ AGU[1] (agu.hpp)
└── DUT  (top_crossbar.cpp)
    ├── Crossbar 4x4 – OBI (crossbar.hpp)
    └── Bank[0..3] (bank.hpp)
```

## Modules

| Block          | File                  | Role                                                                                                                                                                                                                                                                                                                                                                                              |
| -------------- | --------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Testbench      | `tb_top_crossbar.cpp` | Top-level harness. Reads the two memory-access traces `mem_0.log` / `mem_1.log` and drives them into the two AGUs; instantiates the DUT.                                                                                                                                                                                                                                                          |
| AGU[0], AGU[1] | `agu.hpp`             | OBI managers. Each reads **2 addresses (rows) per cycle** from its `mem_N.log` trace and issues them as **2 separate OBI requests** (2 ports into the crossbar). The two words are needed **together** for the same computation, so the AGU runs in **lock-step**: it does not read the next trace lines or issue a new pair until **both** current requests have completed (responses received). |
| DUT            | `top_crossbar.cpp`    | Device under test: wraps the crossbar and the four banks.                                                                                                                                                                                                                                                                                                                                         |
| Crossbar 4x4   | `crossbar.hpp`        | 4 request inputs × 4 bank outputs. Routes each request to its target bank by address: it **opens a master↔bank connection and holds it open until the transaction closes** (request through response), then frees it. Arbitrates same-bank collisions.                                                                                                                                            |
| Bank[0..3]     | `bank.hpp`            | Four memory banks (OBI subordinates). Each accepts one 32-bit word per access.                                                                                                                                                                                                                                                                                                                    |

## Interfaces & data flow

- **Protocol:** a simplified single-channel OBI on every link — see [doc/protocols/obi.md](../protocols/obi.md) (no outstanding transactions; a connection is held open from grant until the response closes it).
- **Granularity:** 32-bit (1 word) per bank access.
- **Traces:** `mem_N.log` is a single column of hexadecimal addresses, one per access. Each AGU reads **2 addresses/cycle**. Read-vs-write for the whole run is chosen in the testbench (uniform across all transactions in a simulation).
- **AGU output:** each AGU drives **2 words** as **2 request ports** — the diagram shows two arrows from each AGU into the crossbar. (Unlike TDM, the two words are *not* grouped; they stay independent ports.)
- **Crossbar:** 2 AGUs × 2 words = **4 request ports in**, **4 bank ports out** (hence 4×4). Each of the 4 requests is routed to one of the 4 banks in the same cycle.
- **Banking:** word-interleaved across the 4 banks.
- **Banks:** 1 word (32-bit) each per access.

## Conflict handling

When two request ports target the **same bank** in one cycle, the crossbar **arbitrates round-robin**: one request wins and opens its connection; the loser's `gnt` stays low until that connection **closes** and the bank frees. This needs a **round-robin per-bank arbiter** on the crossbar's bank-side outputs.

Because an AGU's two words are needed together, the AGU runs in **lock-step**: it holds both requests and does not fetch the next trace pair until **both** complete. So a single conflicted port delays that whole AGU's progress by **1 cycle** — it simply waits a cycle to issue the next pair while the stalled request is served.

With word-interleaving, an AGU's own two consecutive addresses always hit different banks, so a single AGU never self-conflicts — **for now, conflicts arise only across the two AGUs** (other use-cases come later). Each conflict between two masters costs **1 cycle** (vs. 2 for the [TDM](tdm.md) design) and **scales with the number of conflicting masters**. The accumulated stall cycles are the **delay penalty** the experiment reports, relative to an ideal run where the ports never collide.
