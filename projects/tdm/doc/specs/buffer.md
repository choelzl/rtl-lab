# Buffer

Reference implementation:

- SystemC model: [`buffer.hpp`](../../rtl/systemc/buffer.hpp) (window
  orchestration) composed of [`buffer_cell.hpp`](../../rtl/systemc/buffer_cell.hpp)
  (all per-slot state — one cell per TDM slot).
- SystemVerilog RTL: not yet implemented (this spec is the target for it).

One `buffer` instance sits between one requester group's port-facing OBI
lanes and the 32-lane TDM-side OBI interface, in both directions:

- **Read mode** (`IS_WRITE = false`): a prefetch buffer. Cells fetch a full
  `NUM_TDM` window of addresses supplied by the AGU's lookahead bus and drain
  it to the ports one group per cycle, windows streaming back to back.
- **Write mode** (`IS_WRITE = true`): a write-combining buffer. Ports fill a
  full window one group per cycle; a one-cycle snapshot hands the window to
  per-cell shadow engines that burst it to the banks while the next window
  is already filling; port acks are posted.

Both modes achieve the same pinned property: on conflict-free traffic the
task span equals `ceil(n_data / lanes)` — cycle-exact parity with the
three-level crossbar backend (see `doc/report/`).

## Structure

All per-slot state lives in the 32 `buffer_cell` instances; the parent
`buffer` owns only window bookkeeping (two group pointers, the latched
window geometry, a few flags) plus a combinational port⇄group router.
Control between them is deliberately narrow:

| Wire | Direction | Meaning |
| ---- | --------- | ------- |
| `cell_reset_window_s` | buffer → all cells | ONE broadcast pulse = "a window boundary happened": read wrap, read boot-from-idle, and write snapshot all drive it. The same pulse tells the caller to advance its lookahead one window. |
| `cell_all_valid_s[w]` | buffer → cell w | Cell `w`'s own group is draining this cycle (arms the per-cell refetch and the same-cycle forward path). |
| `valid_o[w]` | cell w → buffer | Read: presentable now. Write: shadow flush done. |
| `invalid_o[w]` | cell w → buffer | Read: idle (`!valid && !pending`). Write: primary latch free. |

The buffer AND-reduces the collectors into the only three decisions it
makes: is the current group drain-ready, are all shadows free, is the whole
array idle (boot).

## Interface

| Signal | Direction | Meaning |
| ------ | --------- | ------- |
| `clk_i`, `rst_ni` | in | Clock, active-low reset. |
| `active_mode` | in | Drain/fill width (encoding below). Latched into `window_mode_q` at each window boundary, so a mid-stream change never tears a window. |
| `p[NUM_IO]` | subordinate | Port-side OBI, one full `obi_subordinate_ports` bundle per lane (`NUM_IO = PORT_COUNT * NUM_REQ`). `p[i].we_i` is wired but ignored — direction is fixed per instance by `IS_WRITE`. Write mode: `gnt_o` = fill accepted, `rvalid_o` = posted ack, `rdata_o` = 0. |
| `m[NUM_TDM]` | manager | TDM-side OBI, one lane per slot; cell `w` owns `m[w]` outright. |
| `fetch_addr_i[NUM_TDM]`, `fetch_addr_valid_i` | in | Read mode: the lookahead window's addresses (`addr == 0` = NOP lane, never issued). Unused in write mode. |

## Parameters

| Parameter | Default | Meaning |
| --------- | ------- | ------- |
| `NUM_REQ` | 4 | OBI lanes per port. |
| `PORT_COUNT` | 1/2/4 | Ports connected to this buffer instance. |
| `NUM_TDM` | 32 | TDM lanes = slots per window = cells. |
| `BYTES_PER_ROW` | 16 | Data beat width (128 b). |
| `IS_WRITE` | false | Selects read-prefetch or write-combining behavior. |

`active_mode` encoding (`lanes = active_ports * NUM_REQ`):

| `active_mode` | Active ports | Lanes / drain group |
| ------------- | ------------ | ------------------- |
| 0 | 1 | 4 |
| 1 | 2 | 8 |
| 2 or 3 | 4 (clamped to `PORT_COUNT`) | 16 |

## Read mode

Each cell runs a private two-cycle fetch (arbiter grant, then bank
response) and holds one value; per-cell flags are orthogonal bits
(`valid_q / pending_q / granted_q / fetched_q / primed_q`), not a state
machine. The group at `rd_ptr_q` drains when every cell in it is valid and
all active lanes assert `p_req_i`; a per-cell `is_fwd` mux forwards a value
arriving on the drain edge directly to the port.

**Per-cell refetch (the zero-bubble property).** A cell starts its next
fetch by the single rule

```
start = !pending_q && en_i && (all_valid_i || !valid_q)
```

- `all_valid_i` — this cell's own group is draining this edge: refetch
  starts immediately, overlapping the other groups' turns. The slot is not
  needed again for `n_groups − 1` cycles, which hides the two-cycle round
  trip for every real configuration, so windows stream with no transition
  gap.
- `!valid_q` — the cell holds nothing: covers the boot fetch after reset
  and a parked cell (drained while `en_i` was low, e.g. behind a fence)
  restarting the moment `en_i` returns. A start wipes nothing, so `en_i`
  needs no edge detection or gap thresholds.

**Boot latch.** Restart from idle is the wrap path, not a special case:
when every cell reports idle and the fetch bus is enabled, the buffer snaps
the staged window under the current `active_mode` and pulses the same
`window_reset` a wraparound does.

**Caller contract.** One pulse, one meaning: each `window_reset` pulse
tells the AGU to advance its lookahead exactly one window. The AGU exposes
its next `NUM_TDM` addresses on `fetch_addr_i` continuously; NOP lanes
carry `addr = 0`.

## Write mode

Three stages run concurrently (the write-side twin of the read
pipelining):

1. **Fill.** Ports latch one group per cycle into the cells' primary
   latches (`p_gnt_o = fill_ok`), advancing straight through window
   boundaries whenever primaries are free (`fill_ptr_q`).
2. **Snapshot.** When the window is full and the previous burst has
   cleared —
   `snapshot = (fill_wrap || full_q) && shadows_free && (!resp || resp_wraps)`
   — a one-cycle `reset_window` pulse copies every primary into its cell's
   shadow engine atomically and frees the primaries; the next window's fill
   begins the very next cycle.
3. **Respond.** Port acks are **posted**: `p_rvalid_o` streams one group
   per cycle right behind the snapshot (`rd_ptr_q` doubles as the respond
   pointer). An ack means "the burst is in flight", not "the bank
   committed". A slow/conflicted burst back-pressures the NEXT window's
   snapshot (its fill parks as `full_q`) instead of stalling the ports beat
   by beat.

**Shadow lifetime.** A shadow engine requests until its grant and frees
**at the grant** — the bank fabric samples the payload on the edge after
the grant, so nothing downstream ever reads a freed shadow, and the
returning `rvalid` is not tracked at all. A grant-live preview
(`valid_o = busy && !gnt`) lets the parent snapshot on the exact edge the
last grant lands. Consequently the effective bus round trip never exceeds
the fill time and windows run back to back
(`window period = max(fill, flush) = fill` on conflict-free traffic).

**Ordering consequence.** Posted acks relocate conflict cost onto the
following window, and read-after-write ordering across *different* buffers
requires a fence — which the production phase structure provides at
thousands-of-cycles granularity.

## Mode changes

`window_mode_q` (and `pend_mode_q` for a parked write window) freeze the
geometry a window was started with; `active_mode` is re-sampled only at
window boundaries (wrap, boot, snapshot). The caller keeps `en_i`/traffic
aligned to window boundaries; within a window the latched geometry rules.

## Verification

Unit suites (see `tb/unit/`): `buffer` (154), `buffer_cell` (74),
`buffer_pipeline` (12) for read; `buffer_wr` (63), `buffer_cell_wr` (43),
`buffer_pipeline_write` (36) for write; `buffer_modes` (11) for the
active_mode alias and PORT_COUNT clamp geometry. System-level timing is pinned by
the `stim_bank_*` suites: conflict-free spans equal `ceil(n_data/lanes)`
exactly, and same-bank streams keep the target bank 100 % utilized
(bank-side occupancy exactly `n_data` on the adaptive build).
