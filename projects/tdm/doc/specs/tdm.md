# TDM architecture

Reference diagram: [tdm.png](../diagrams/tdm.png)

Time-Division-Multiplexed interconnect: instead of giving every request port its own spatial path into the banks, read requests are buffered, scheduled, mapped, and routed through a reused crossbar/bank backend. The goal is to compare the conflict cost of this time-shared path against the crossbar baseline.

**What the comparison measures:** for each architecture, the percentage delay/overhead caused by structural conflicts relative to an ideal conflict-free execution of the same trace. See [crossbar.md](crossbar.md) for the baseline.

> **Note on the flow.** Use the unified SystemC simulation top: `make sim-sc PROJECT=tdm TOP_LEVEL=top IMPL=tdm` for the native SystemC TDM backend, or `IMPL=top_tdm` for the Verilated SV TDM backend. The harness is [../../tb/systemc/tb_top.cpp](../../tb/systemc/tb_top.cpp), and the wrapper that selects the backend is [../../rtl/systemc/top.hpp](../../rtl/systemc/top.hpp).

## Hierarchy

```text
Testbench  (tb_top.cpp)
├── ragu_a.log ... ragu_dma.log ──▶ read AGUs  (agu.hpp, TDM mode for IMPL=tdm)
├── wagu_a.log ... wagu_dma.log ──▶ write AGUs (agu.hpp, crossbar mode)
└── DUT wrapper (top.hpp, TOP_LEVEL=top, IMPL=tdm/top_tdm)
    └── TDM backend
        ├── Buffers / scheduling
        ├── TDM mapping function
        ├── Reused crossbar conflict resolver
        └── Bank[0..N_BANK-1]
```

## Modules

| Block | File | Role |
| ----- | ---- | ---- |
| Testbench | `tb/systemc/tb_top.cpp` | Unified `sc_main`. Reads RAGU/WAGU traces, drives them through `top`, runs to completion, and writes per-trace CSVs plus `stats.log`. |
| Wrapper | `rtl/systemc/top.hpp` | Packs the RAGU/WAGU driver port groups onto the flat arrays used by the selected backend. |
| Native TDM | `rtl/systemc/top_tdm.hpp` | Native SystemC backend selected with `IMPL=tdm`. The old `IMPL=tdm_sc` spelling remains a compatibility alias. |
| SV TDM | `rtl/top_tdm.sv` | Verilated backend selected with `IMPL=top_tdm`. |
| AGU | `tb/systemc/agu.hpp` | In native TDM mode, RAGUs parse the first trace line as mapping metadata and issue grouped addresses; WAGUs use crossbar-style independent requests. |
| Mapping | `tdm.hpp` / `tdm.sv` blocks | Maps each logical group into bank/row placements using the parameters from the trace metadata. |
| Crossbar and banks | `crossbar.hpp` / `bank.hpp` and SV counterparts | Reused conflict resolver and memory banks. |

The unified top exposes the same fixed port map as the crossbar backend:

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

## Interfaces And Data Flow

- **Read path:** RAGU drivers feed the TDM buffers/mapping path. The mapping emits re-encoded addresses so the reused crossbar's word-interleaved decode recovers the intended `(bank,row)` placement.
- **Write path:** WAGU drivers are exposed through the same wrapper shape and use crossbar-style traces.
- **Stimuli:** RAGU/WAGU trace logs under `tb/stimuli/<case>/`, for example `ragu_a.log` and `wagu_a.log`. Native TDM defaults to the `sample_tdm` stimuli if `IN_DIR` is unset.
- **Metadata:** the first trace line is `num_banks,bank_width,C,R,L,store_mode`. TDM-mode RAGUs parse and validate it; crossbar-mode AGUs skip it.
- **Access rows:** `addr,we,data`, where `addr` is a hex byte address, `we` is `1` for write and `0` for read, and `data` is the write value or empty on reads.
- **Conflict source:** a conflict occurs when two or more mapped words in a slot target the same bank. The reused crossbar serializes those requests, and the buffer re-requests the losers later.

## Configuration Parameters

The user-facing flow parameters are passed through `PARAMS="NAME=VALUE ..."`.

| Parameter | Meaning | Default |
| --------- | ------- | ------- |
| `N_BANK` | number of memory banks | 32 |
| `N_ROW` | rows per bank | 1024 |
| `WORD_BYTES` | bytes per word / OBI data beat | 4 |
| `WORDS_PER_ROW` | words per row in the wide data type | 4 |

The mapping uses the per-trace metadata values (`num_banks`, `bank_width`, `C`, `R`, `L`, and `store_mode`) in addition to the build-time parameters above.

## Current Status

- **Unified flow:** run with `TOP_LEVEL=top` and choose `IMPL=tdm` or `IMPL=top_tdm`.
- **Reuse:** the TDM backend reuses the crossbar and bank blocks as its conflict resolver and memory system.
- **Metric:** the harness reports actual cycles, ideal cycles, and overhead in `stats.log`.
- **Stimulus shape:** traces are named by AGU group rather than the old `mem_N.log` convention.

## Open / To Investigate

1. **Adverse traces / parameter sweep:** pick mapping parameters and access patterns that place multiple slot words on the same bank to exercise the conflict penalty, then compare against the crossbar baseline on the same trace.
2. **`bank_width` semantics:** reproduced as `e = log2(bank_width)` in [map_func.md](map_func.md); confirm the intended unit/value for the real kernels.
3. **Pipeline calibration:** re-measure the ideal pipeline fill if the backend pipeline depth or wrapper/harness timing changes.
