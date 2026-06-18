# Crossbar architecture

Reference diagram: [crossbar.png](../diagrams/crossbar.png)

Baseline interconnect: a full **8×8 crossbar** that connects every request port to every memory bank in parallel (spatial interconnect, single cycle). This is the reference design the [TDM architecture](tdm.md) is compared against.

**What the comparison measures:** for each architecture, the **percentage delay penalty caused by bank conflicts** relative to an ideal conflict-free execution of the same trace (i.e. how many extra cycles conflicts cost vs. a run where every access completes in one cycle). The crossbar and the TDM design resolve conflicts differently, so this number captures the cost of each scheme.

> **Note on the flow.** All modules are SystemC (`.hpp`/`.cpp`, `sc_main` testbench), so this design targets the `make sim-sc` flow. The synthesizable DUT (crossbar + banks) lives under `rtl/systemc/`; the verification drivers (AGUs) and the `sc_main` harness live under `tb/systemc/`.

## Hierarchy

```
Testbench  (tb_top_crossbar.cpp)
├── mem_0.log ──▶ AGU[0] (agu_crossbar.hpp)
├── mem_1.log ──▶ AGU[1] (agu_crossbar.hpp)
└── DUT  (top_crossbar.cpp)
    ├── Crossbar 8x8 – OBI (crossbar.hpp)
    └── Bank[0..7] (bank.hpp)
```

## Modules

| Block          | File                  | Role                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| -------------- | --------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Testbench      | `tb_top_crossbar.cpp` | Top-level harness. Reads the two memory-access CSV traces `mem_0.log` / `mem_1.log` and drives them into the two AGUs; instantiates the DUT.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           |
| AGU[0], AGU[1] | `agu_crossbar.hpp`    | OBI managers. Each reads **4 rows per cycle** from its `mem_N.log` CSV (each row carries `addr`, `we`, `data`) and issues them as **4 separate OBI requests** (4 ports into the crossbar). The four words are needed **together**, so the AGU is **group-synchronized and pipelined**: it advances to the next group as soon as **all four** ports are **granted** (it does not wait for the responses, which return one cycle later), keeping requests asserted so it issues a new group **every cycle** when there is no conflict. It keeps a 1-deep in-flight record per port to collect the overlapping responses. The AGU holds **registers** for each request's `addr`, `we`, `be`, `wdata` and the returned `rdata` — the first register stage in the datapath. |
| DUT            | `top_crossbar.cpp`    | Device under test: wraps the crossbar and the eight banks.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             |
| Crossbar 8x8   | `crossbar.hpp`        | 8 request inputs × 8 bank outputs. Routes each request to its target bank by address and steers each bank's response (one cycle later, via a 1-deep per-bank owner register) back to the requesting port. Arbitrates same-bank collisions round-robin, one grant per bank per cycle.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
| Bank[0..7]     | `bank.hpp`            | Eight memory banks (OBI subordinates) holding the **bank registers** — the next register stage after the AGU. Each bank's logic converts the registered OBI request into an access to its internal memory array (a read when `we=0`, a write when `we=1`), returning `rdata`.                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          |

## Interfaces & data flow

- **Protocol:** a simplified single-channel OBI on every link — see [obi.md](obi.md) (in-order, pipelined depth ≤ 2; each bank response returns one cycle after the grant via a per-bank owner register).
- **Granularity:** 32-bit (1 word) per bank access.
- **Stimuli:** `mem_N.log` (stored under `tb/stimuli/`) is a **CSV**, one row per access, with three columns: `addr` (hexadecimal byte address), `we` (`1` = write, `0` = read), and `data` (the value written on a write; left empty on a read). The file's **first line is metadata** (the shared format carries the TDM mapping parameters there); the crossbar AGU **skips it** and does not use it. R/W and write data are carried **per access** in the trace — the testbench no longer forces a uniform mode — so reads and writes can be interleaved for future experiments. Each AGU consumes **4 rows/cycle** (one per request port).
- **AGU output:** each AGU drives **4 words** as **4 request ports** into the crossbar. (Unlike TDM, the four words are *not* grouped; they stay independent ports.)
- **Crossbar:** 2 AGUs × 4 words = **8 request ports in**, **8 bank ports out** (hence 8×8). Each of the 8 requests is routed to one of the 8 banks in the same cycle.
- **Banking:** word-interleaved across the 8 banks.
- **Banks:** 1 word (32-bit) each per access.

Example `mem_N.log` (CSV; `addr`/`data` = `0x`-prefixed hex, `we` = `1` write / `0` read, `data` empty on reads):

```
addr,we,data
0x00000010,0,
0x00000024,1,0xdeadbeef
0x00000040,0,
0x0000005c,1,0x0000002a
```

## Configuration parameters

The design described above is the **default**; every size is an RTL parameter, so the crossbar can be re-scaled without structural changes.

| Parameter    | Meaning                                    | Default    |
| ------------ | ------------------------------------------ | ---------- |
| `N_RAGU`     | number of AGUs (OBI managers)              | 2          |
| `N_WAGU`     | number of AGUs (OBI managers)              | 2          |
| `N_REQ`      | requests (ports) issued per AGU per cycle  | 4          |
| `WORD_BYTES` | size of one word / OBI data beat, in bytes | 4 (32-bit) |
| `N_BANK`     | number of memory banks                     | 32         |
| `N_ROW`      | rows per bank (one word each)              | 1024       |

Derived quantities:

- **Crossbar size** = `(N_RAGU × N_REQ)` request ports × `N_BANK` bank ports (default **8×32**).
- **Total capacity** = `N_BANK × N_ROW × WORD_BYTES` bytes (default 32 × 1024 × 4 = **128 KiB**).

Address decode for a byte address `a` (word-interleaved across banks):

```
word = a / WORD_BYTES   // global word index
bank = word % N_BANK    // low bits select the bank (interleave)
row  = word / N_BANK    // bank-local index (= the bank's row), one word per row
```

The crossbar routes the request to `bank` and presents the **bank-local** byte address `row × WORD_BYTES` to the selected bank (the bank-select field stripped); each bank holds `N_ROW` words.

Notes / assumptions:
- **Independent requests:** the `N_REQ` addresses an AGU issues are **independent** (arbitrary, not assumed consecutive or distinct in any particular way). So two of an AGU's own requests can decode to the same bank — **self-conflicts (within one AGU) and cross-AGU conflicts are both possible** and are handled identically by the per-bank round-robin arbiter.
- **Capacity / range:** a trace byte-address that falls outside the total capacity is an **error (assert)**.
- **Banks** are zero-initialised; **bank access latency is 1 cycle**; write data comes from the trace's `data` column, so a write phase followed by a read phase lets you check read-back by inspecting the per-AGU output logs (see [Verification](#verification)).

## Datapath & registers

The path is registered at **two stages only** (AGU and bank); the 8×8 crossbar between them is combinational routing plus a per-bank round-robin arbiter. It adds **no data-path pipeline register** — only small per-bank *control* state (the round-robin pointer and a one-deep response-owner register that steers each bank's response, which arrives one cycle after the grant, back to the manager that won it):

- **AGU registers** — each AGU stores `addr`, `we`, `be`, `wdata` for each request and captures the returned `rdata`. These drive the request into the crossbar.
- **Bank registers** — the next stage. Each bank registers the incoming OBI request; its logic then converts the request into an access to the bank's internal memory array (a read when `we=0`, a write when `we=1`) and returns `rdata`.

## Conflict handling

When **two or more** request ports target the **same bank** in one cycle, the crossbar **arbitrates round-robin**: one request wins (its `gnt` asserts); the losers' `gnt` stays low and they retry the next cycle (the bank serves **one request per cycle**). This needs a **round-robin per-bank arbiter** on the crossbar's bank-side outputs.

Because an AGU's group of words is needed together, the AGU is **group-synchronized**: it advances to the next group as soon as **all** `N_REQ` ports are **granted** (overlapping the responses), so with no conflict it issues one group per cycle. A conflict that stalls even one port holds the whole group back until that port is granted — that wait is the delay penalty.

The `N_REQ` addresses an AGU issues are **independent**, so collisions on a bank can arise **both within one AGU's own group and across AGUs** — any of the `N_AGU × N_REQ` ports that decode to the same bank in a cycle conflict. A bank that receives `k` simultaneous requests serializes them over `k` cycles, i.e. **`k − 1` stall cycles**. The accumulated stall cycles are the **delay penalty** the experiment reports, relative to an ideal run where the ports never collide.

## Verification

For now verification is **inspection-based**, not self-checking:

- Each trace is a **write phase** (writes with random `data`) followed by a **read phase** that reads the same addresses back.
- Each AGU dumps its completed accesses to an **output log** in the run's output dir (`sim/<OUT_DIR>/output/out_0.log` / `out_1.log`) as `cycle,addr,we,data`, where `data` is the value written (writes) or the value returned (reads). Because the read phase targets the addresses written earlier, a correct run shows each read returning the value of the matching write — you confirm this by inspecting the output logs.
- After the run the testbench prints **statistics only** (cycle count, accesses, conflicts / delay penalty); there is **no automated correctness assertion**.

An automated golden-model scoreboard can be added later, but is intentionally out of scope for now.
