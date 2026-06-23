# Crossbar architecture

Reference diagram: [crossbar.png](../diagrams/crossbar.png)

Baseline interconnect: a full crossbar that connects request ports to memory banks in parallel. This is the reference design the [TDM architecture](tdm.md) is compared against.

**What the comparison measures:** for each architecture, the percentage delay/overhead caused by structural conflicts relative to an ideal conflict-free execution of the same trace. The crossbar and TDM implementations resolve conflicts differently, so this number captures the cost of each scheme.

> **Note on the flow.** Use the unified SystemC simulation top: `make sim-sc PROJECT=tdm TOP_LEVEL=top IMPL=crossbar` for the native SystemC crossbar, or `IMPL=top_crossbar` for the Verilated SV crossbar. The harness is [../../tb/systemc/tb_top.cpp](../../tb/systemc/tb_top.cpp), and the wrapper that selects the backend is [../../rtl/systemc/top.hpp](../../rtl/systemc/top.hpp).

## Hierarchy

```text
Testbench  (tb_top.cpp)
├── ragu_a.log ... ragu_dma.log ──▶ read AGUs  (agu.hpp, crossbar mode)
├── wagu_a.log ... wagu_dma.log ──▶ write AGUs (agu.hpp, crossbar mode)
└── DUT wrapper (top.hpp, TOP_LEVEL=top, IMPL=crossbar/top_crossbar)
    └── Crossbar backend
        ├── Crossbar hierarchy (top_crossbar.hpp or top_crossbar.sv)
        └── Bank[0..N_BANK-1]
```

## Modules

| Block | File | Role |
| ----- | ---- | ---- |
| Testbench | `tb/systemc/tb_top.cpp` | Unified `sc_main`. Reads RAGU/WAGU traces, drives them through `top`, runs to completion, and writes per-trace CSVs plus `stats.log`. |
| Wrapper | `rtl/systemc/top.hpp` | Packs the RAGU/WAGU driver port groups onto the flat arrays used by the selected backend. |
| Native crossbar | `rtl/systemc/top_crossbar.hpp` | Native SystemC backend selected with `IMPL=crossbar` or by omitting `IMPL`. |
| SV crossbar | `rtl/top_crossbar.sv` plus `rtl/systemc/top_crossbar_sv.hpp` | Verilated backend selected with `IMPL=top_crossbar`. |
| AGU | `tb/systemc/agu.hpp` | OBI managers. In crossbar mode the first metadata line is skipped, then access rows are replayed as independent OBI requests. |
| Banks | `bank.hpp` / `bank.sv` | Single-port OBI RAM banks with one-cycle response latency. |

The unified top exposes this fixed port map:

| Driver | Ports | Flat OBI buses |
| ------ | ----- | -------------- |
| `RAGU_A` | 4 | 16 |
| `RAGU_B` | 2 | 8 |
| `RAGU_C` | 1 | 4 |
| `RAGU_D` | 1 | 4 |
| `RAGU_DMA` | 1 | 4 |
| `WAGU_A` | 4 | 16 |
| `WAGU_B` | 2 | 8 |
| `WAGU_D` | 1 | 4 |
| `WAGU_DMA` | 1 | 4 |

Each driver owns `NUM_REQ=4` independent OBI buses per port. The crossbar backend therefore sees 9 read ports and 8 write ports.

## Interfaces And Data Flow

- **Protocol:** a simplified single-channel OBI on every link; see [obi.md](obi.md).
- **Stimuli:** RAGU/WAGU trace logs under `tb/stimuli/<case>/`, for example `ragu_a.log` and `wagu_a.log`. The shared trace format starts with the TDM mapping metadata line, which crossbar-mode AGUs skip.
- **Access rows:** `addr,we,data`, where `addr` is a hex byte address, `we` is `1` for write and `0` for read, and `data` is the write value or empty on reads.
- **Routing:** each request is routed to `bank = (addr / WORD_BYTES) % N_BANK`; the bank-local row is `(addr / WORD_BYTES) / N_BANK`.
- **Conflicts:** any simultaneous requests that target the same bank are serialized by the per-bank round-robin arbiter. Losers keep retrying until granted.

## Configuration Parameters

The user-facing flow parameters are passed through `PARAMS="NAME=VALUE ..."`.

| Parameter | Meaning | Default |
| --------- | ------- | ------- |
| `N_BANK` | number of memory banks | 32 |
| `N_ROW` | rows per bank | 1024 |
| `WORD_BYTES` | bytes per word / OBI data beat | 4 |
| `WORDS_PER_ROW` | words per row in the wide data type | 4 |

Total bank capacity is `N_BANK * N_ROW * WORDS_PER_ROW * WORD_BYTES` bytes.

## Conflict Metric

The AGU is pipelined and group-synchronized: it advances to the next group as soon as all ports in the current group are granted, while responses return one cycle later and overlap the next address phase. The harness reports:

```text
ideal = max_groups + 2
```

where `max_groups` is the largest group count across all connected AGUs. Cycles beyond this ideal are reported as overhead.

## Verification

The harness writes one completed-access CSV per AGU in `sim/<OUT_DIR>/output/`, for example `ragu_a.csv`, plus `stats.log`, `compile.log`, and `run.log`. A write/readback trace can be checked by confirming that each read returns the value written earlier for the same address.
