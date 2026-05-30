# Tdm

Memory-interconnect designs that compare a full **crossbar** against a
**time-division-multiplexed (TDM)** scheme, measuring the delay penalty that
bank conflicts cost each architecture.

This project plugs into the repository-level EDA flow. See the [root README](../../README.md) for the `make` targets, their generic parameters, and the typical pipeline. This document covers the parts specific to `tdm`. The crossbar architecture is the one implemented so far — full design notes are in [doc/specs/crossbar.md](doc/specs/crossbar.md) (protocol: [doc/specs/obi.md](doc/specs/obi.md)).

The design is **pure SystemC** (no SV): the synthesizable DUT lives under `rtl/systemc/`, the verification drivers + harness under `tb/systemc/`. It runs through the `make sim-sc` flow, which builds a pure-SystemC project directly with `g++`.

## Quick start

```bash
source ../../sourceme.sh   # or: source sourceme.sh from the repository root

make sim-sc PROJECT=tdm TOP_LEVEL=top_crossbar OUT_DIR=run0
```

This builds [tb/systemc/tb_top_crossbar.cpp](tb/systemc/tb_top_crossbar.cpp), runs it on the traces in [tb/traces/](tb/traces/), and prints timing statistics. `tdm` is selected with `PROJECT=tdm` on every `make` command.

Override the design size with `PARAMS` (each becomes a `-D`):

```bash
make sim-sc PROJECT=tdm TOP_LEVEL=top_crossbar OUT_DIR=big \
    PARAMS="N_BANK=16 N_REQ=8"
```

## Top-level modules

| Top-level | File | Description |
| --- | --- | --- |
| `top_crossbar` | [rtl/systemc/top_crossbar.hpp](rtl/systemc/top_crossbar.hpp) | DUT: the `N_MGR × N_BANK` crossbar interconnect plus `N_BANK` memory banks. Exposes the manager-side OBI ports; the harness attaches the AGUs. |

DUT submodules: [crossbar.hpp](rtl/systemc/crossbar.hpp) (round-robin per-bank arbiter, word-interleaved routing) and [bank.hpp](rtl/systemc/bank.hpp) (single-port OBI RAM, 1-cycle latency).

## RTL elaboration parameters

Passed via `PARAMS="NAME=VALUE …"` (forwarded as `-DNAME=VALUE`); defaults below.

| Parameter | Meaning | Default |
| --- | --- | --- |
| `N_AGU` | number of AGUs (managers) | 2 |
| `N_REQ` | request ports per AGU | 4 |
| `N_BANK` | number of memory banks | 8 |
| `N_ROW` | rows (words) per bank | 1024 |
| `WORD_BYTES` | bytes per word / OBI data beat | 4 |

`N_MGR = N_AGU · N_REQ` request ports; total capacity `N_BANK · N_ROW · WORD_BYTES` bytes. See [doc/specs/crossbar.md](doc/specs/crossbar.md#configuration-parameters) for the address decode and full semantics.

## Testbenches

| Testbench | File | Drives |
| --- | --- | --- |
| `tb_top_crossbar` | [tb/systemc/tb_top_crossbar.cpp](tb/systemc/tb_top_crossbar.cpp) | `sc_main`: instantiates `top_crossbar` + `N_AGU` [AGUs](tb/systemc/agu.hpp), runs to completion, prints statistics. |

**Traces** ([tb/traces/](tb/traces/)): `mem_<i>.log`, one CSV per AGU — `addr,we,data` (hex byte address, 1=write/0=read, write value or empty on reads). The sample traces write random data to a set of addresses then read them back.

**Outputs** (in `sim/<OUT_DIR>/output/`): `out_<i>.log` (per-AGU completed accesses `cycle,addr,we,data` for inspection — a read should return the value of its matching write), plus `compile.log` / `run.log`.

## Statistics & the conflict metric

The harness reports:

- **actual cycles** — measured cycles from reset release until every AGU has drained its trace.
- **ideal cycles** — the conflict-free analytical estimate (derivation below).
- **delay penalty** — `100 · (actual − ideal) / ideal` %, i.e. the cost of bank conflicts.

### How the ideal (conflict-free) cycles are computed

The AGU runs in lock-step **groups** of `N_REQ` requests: it issues all `N_REQ` at once and waits for every response before issuing the next group. With no bank conflict, all `N_REQ` requests of a group hit distinct banks, are granted in the same cycle, and complete together — so a group has a **fixed pipeline latency**:

```
L = (address phase) + (1-cycle bank response) = 1 + 1 = 2 cycles
```

`L` is independent of `N_REQ`/`N_BANK` (all ports are parallel). The first group is issued one cycle after reset (+1 startup). Groups are serial (lock-step), so a single AGU with `G` groups finishes at `L·G + 1 = 2G + 1` (measured exactly: G=2→5, G=4→9, G=8→17).

In the conflict-free ideal the AGUs never interfere, so they run fully in parallel and the run ends with the slowest one:

```
ideal = L · G_max + 1 = 2 · G_max + 1,   G_a = ceil(len_a / N_REQ),  G_max = max_a G_a
```

Any cycles beyond `ideal` are arbitration stalls — the conflict penalty. (`L` is a property of the module timing; it is a named constant in the harness and must be updated if that timing changes.)

## Experiments (automation scripts)

_None yet. Add experiment subfolders under `scripts/flow/`._
