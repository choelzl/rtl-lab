# TDM architecture

Reference diagram: [tdm.png](../diagrams/tdm.png)

Time-Division-Multiplexed interconnect: instead of giving every request port its own spatial path into the banks, read requests are buffered, scheduled, mapped, and routed through a reused crossbar/bank backend. The goal is to compare the conflict cost of this time-shared path against the crossbar baseline.

**What the comparison measures:** for each architecture, the percentage delay/overhead caused by structural conflicts relative to an ideal conflict-free execution of the same trace. See [crossbar.md](crossbar.md) for the baseline.

> **Note on the flow.** Use the unified SystemC simulation top: `edaf sim IMPL=tdm MODE=sc` for the native SystemC TDM backend, or `edaf sim IMPL=tdm,sv MODE=sc SV_MODS=top_tdm` for the Verilated SV TDM backend. The harness is [../../tb/systemc/tb_top.cpp](../../tb/systemc/tb_top.cpp), and the wrapper that selects the backend is [../../rtl/systemc/top.hpp](../../rtl/systemc/top.hpp).

## Hierarchy

```text
Testbench  (tb_top.cpp)
├── ragu_a.log ... ragu_e.log ──▶ read AGUs  (agu.hpp, TDM mode for IMPL=tdm)
├── wagu_a.log ... wagu_e.log ──▶ write AGUs (agu.hpp, crossbar mode)
└── DUT wrapper (top.hpp, IMPL=tdm / tdm,sv)
    └── TDM backend
        ├── Buffers ×9  (buffer.hpp: 5 read-prefetch + 4 write, 32 cells each)
        ├── Arbiter 1-of-9 (arbiter.hpp free-running round-robin, or
        │                   arbiter_adaptive.hpp request-aware — IMPL_ARB_ADAPTIVE)
        ├── TDM mapping function (tdm.hpp, static XOR-skew map_one)
        ├── Reused beat-interleaved crossbar (crossbar.hpp, bank = beat % N_BANK)
        └── Bank[0..N_BANK-1]  (full-depth: 32 banks of N_ROW rows)
```

One buffer owns the 32-lane bus per cycle (`sel_req`); the arbiter's one-cycle
delayed `sel_rsp` steers grants and read data back to the owning buffer. The
default free-running arbiter burns a slot on idle buffers (a lone requester
gets 1 turn in 9); the adaptive arbiter skips idle buffers and closes that gap.

## Modules

| Block | File | Role |
| ----- | ---- | ---- |
| Testbench | `tb/systemc/tb_top.cpp` | Unified `sc_main`. Reads RAGU/WAGU traces, drives them through `top`, runs to completion, and writes per-trace CSVs plus `stats.log`. |
| Wrapper | `rtl/systemc/top.hpp` | Packs the RAGU/WAGU driver port groups onto the flat arrays used by the selected backend. |
| Native TDM | `rtl/systemc/top_tdm.hpp` | Native SystemC backend selected with `IMPL=tdm`. The old `IMPL=tdm_sc` spelling remains a compatibility alias. |
| SV TDM | `rtl/top_tdm.sv` | Verilated backend selected with `IMPL=tdm,sv SV_MODS=top_tdm`. |
| AGU | `tb/systemc/agu.hpp` | In native TDM mode, RAGUs parse the first trace line as mapping metadata and issue grouped addresses; WAGUs use crossbar-style independent requests. |
| Arbiter | `arbiter.hpp` / `arbiter_adaptive.hpp` | Grants one buffer the shared bus per cycle: free-running round-robin, or request-aware (skip idle) with `IMPL_ARB_ADAPTIVE`. |
| Mapping | `tdm.hpp` / `tdm.sv` blocks | Maps each logical group into bank/row placements (`map_one`, a static pure function). `R/C/L/store_mode` come from the trace descriptor per buffer; `num_banks`/`bank_width` are the build-time `N_BANK`/`BYTES_PER_ROW`. |
| Crossbar and banks | `crossbar.hpp` / `bank.hpp` and SV counterparts | Reused conflict resolver and memory banks. |

The unified top exposes the same fixed port map as the crossbar backend:

| Driver | Ports | Flat OBI buses |
| ------ | ----- | -------------- |
| `RAGU_A` | 4 | 16 |
| `RAGU_B` | 2 | 8 |
| `RAGU_C` | 1 | 4 |
| `RAGU_D` | 1 | 4 |
| `RAGU_E` | 1 | 4 |
| `WAGU_A` | 4 | 16 |
| `WAGU_B` | 2 | 8 |
| `WAGU_D` | 1 | 4 |
| `WAGU_E` | 1 | 4 |

## Interfaces And Data Flow

- **Read path:** RAGU drivers feed the TDM buffers/mapping path. The mapping emits re-encoded addresses so the reused crossbar's beat-interleaved decode recovers the intended `(bank,row)` placement — the crossbar routes whole OBI beats and never touches words.
- **Write path:** WAGU drivers are exposed through the same wrapper shape and use crossbar-style traces.
- **Stimuli:** RAGU/WAGU trace logs under `tb/stimuli/<case>/`, for example `ragu_a.log` and `wagu_a.log`. Select with `SEL_IN_DIR` (defaults to `sample`).
- **Descriptor:** each task starts with `#cycle,num_port_active,R,C,L,storemode` (or the short geometry-less form `#cycle,num_port_active,storemode` — see [stimuli.md](stimuli.md)). TDM-mode RAGUs use it to drive the mapping; crossbar-mode drivers use only the timing fields.
- **Access rows:** RAGU rows are `addr` (implicit read); WAGU rows are `addr,data` (implicit write). See [stimuli.md](stimuli.md).
- **Conflict source:** a conflict occurs when two or more mapped words in a slot target the same bank. The reused crossbar serializes those requests, and the buffer re-requests the losers later.

## Configuration Parameters

The user-facing flow parameters are passed through `PARAMS="NAME=VALUE ..."`.

| Parameter | Meaning | Default |
| --------- | ------- | ------- |
| `N_BANK` | number of memory banks | 32 |
| `N_ROW` | rows per bank | 1024 |
| `WORD_BYTES` | bytes per word / OBI data beat | 4 |
| `WORDS_PER_ROW` | words per row in the wide data type | 4 |

The mapping takes `C`, `R`, `L`, and `store_mode` from each buffer's trace descriptor (muxed by the arbiter's selection); `num_banks` and `bank_width` are fixed at build time to `N_BANK` and `BYTES_PER_ROW`.

## Current Status

- **Unified flow:** run with `TOP_LEVEL=top` and choose `IMPL=tdm` or `IMPL=top_tdm`.
- **Reuse:** the TDM backend reuses the crossbar and bank blocks as its conflict resolver and memory system.
- **Metric:** the harness reports actual cycles, ideal cycles, and overhead in `stats.log`.
- **Stimulus shape:** traces are named by AGU group rather than the old `mem_N.log` convention.

## Open / To Investigate

1. ~~Adverse traces / parameter sweep~~ — **done**: the `stim_bank` suite's conflict phases (5: per-group none/partial/full conflicts, 6: 128-beat same-bank streams; both also cross-mapped against the other backend's routing) pin exact cycle spans per build, and `doc/report/` compares the backends (all conflict-free tasks are cycle-exact parity; TDM-adaptive keeps a single bank 100% utilized on same-bank streams).
2. **`bank_width` semantics:** reproduced as `e = log2(bank_width)` in [map_func.md](map_func.md); confirm the intended unit/value for the real kernels.
3. **Pipeline calibration:** re-measure the ideal pipeline fill if the backend pipeline depth or wrapper/harness timing changes.
