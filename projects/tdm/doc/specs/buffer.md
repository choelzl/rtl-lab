# Buffer

Reference implementations:

- SystemC model: [`buffer.hpp`](../../rtl/systemc/buffer.hpp)
- SystemVerilog RTL work item: [`buffer.sv`](../../rtl/buffer.sv)

The buffer is a read-prefetch and width-adaptation block between port-facing OBI buses and a wide TDM-side OBI interface. It fetches a full `NUM_TDM` window, stores returned words in per-slot state, exposes valid data to the port side in active groups, and recycles the full window after all slots have been consumed.

This document is the target spec for correcting both the SystemC model and the SystemVerilog RTL. The implementation should use the canonical names below. Older names are listed only as legacy context for code that has not yet been migrated.

## Canonical Naming

Use the same logical names in SystemC and SystemVerilog. Language-specific prefixes (`a_`/`m_` in the current SystemC model, packed structs in SystemVerilog) may remain only as thin compatibility wrappers.

| Concept | Canonical name | SystemVerilog status | SystemC status | Target action |
| ------- | -------------- | --------------------- | --------------- | ------------- |
| Port count | `PORT_COUNT` | implemented | implemented | Use `PORT_COUNT` in both implementations. |
| OBI buses per port | `NUM_REQ` | implemented | implemented | Use `NUM_REQ` in both implementations. |
| TDM lanes / fetch-window slots | `NUM_TDM` | implemented | implemented | Use `NUM_TDM` in both implementations. |
| Buffer slots | `BUFFER_SIZE` / `NUM_TDM` | `BUFFER_SIZE` kept as compatibility parameter; internal slots use `NUM_TDM` | `BUFFER_SIZE` is an internal alias for `NUM_TDM` | Keep `BUFFER_SIZE == NUM_TDM` for this target. |
| Total port-side OBI buses | `NUM_IO` | implemented | implemented | Prefer `NUM_IO = PORT_COUNT * NUM_REQ`. |
| Active drain mode | `active_mode` | implemented | implemented | Use the 2-bit encoding below. |
| Fetch addresses | `fetch_addr_i` | implemented | implemented | Latch with `fetch_addr_valid_i`. |
| Fetch-address valid | `fetch_addr_valid_i` | implemented | implemented | Active-high latch strobe. |
| TDM request path | `tdm_req_o` | `tdm_req_o` | `m_*_o` | SystemC may keep scalar arrays, but docs and wrappers should call this TDM request. |
| TDM response path | `tdm_resp_i` | `tdm_resp_i` | `m_*_i` | SystemC may keep scalar arrays, but docs and wrappers should call this TDM response. |
| Port request path | `port_req_i` | `port_req_i` | `p_*_i` | SystemC may keep scalar arrays, but docs and wrappers should call this port request. |
| Port response path | `port_resp_o` | `port_resp_o` | `p_*_o` | SystemC may keep scalar arrays, but docs and wrappers should call this port response. |

For SystemVerilog, `fetch_addr_i` uses the address width from `obi_req_t.addr`: `logic [NUM_TDM-1:0][31:0]`. Do not introduce an `OBI_ADDR_WIDTH` macro unless `obi_pkg.sv` is changed first.

## Role

One buffer instance serves one RAGU driver group. The buffer is read-oriented: it prefetches data from the TDM side and returns that data to port-side OBI buses. Port-side writes are out of scope for this module and should be handled by a separate write-buffer component with the inverse collect-and-issue behavior.

The read-prefetch cycle is:

1. All slots start `MISSING`.
2. The buffer latches `fetch_addr_i` when `fetch_addr_valid_i` is asserted for this buffer.
3. The buffer issues one TDM read request per slot using OBI request fields.
4. TDM responses fill slots; filled slots become `VALID`.
5. Ports drain `VALID` slots in active chunks of 4, 8, or 16 beats.
6. Drained slots become `INVALID`.
7. Once the complete `NUM_TDM` window has been consumed, all slots return to `MISSING` and the next full-window fetch starts.

The port side is group-synchronized. All active OBI buses must assert `req` before any active beat receives `gnt`.

## Target Interfaces

### SystemVerilog

| Signal | Direction | Type | Meaning |
| ------ | --------- | ---- | ------- |
| `clk_i` | input | `logic` | Clock. |
| `rst_ni` | input | `logic` | Active-low asynchronous reset. |
| `port_req_i` | input | `obi_req_t [PORT_COUNT-1:0][NUM_REQ-1:0]` | Port-side read requests. `req` participates in the group handshake. Write fields are ignored by the read buffer. |
| `port_resp_o` | output | `obi_resp_t [PORT_COUNT-1:0][NUM_REQ-1:0]` | Port-side grant and read response data. |
| `tdm_req_o` | output | `obi_req_t [NUM_TDM-1:0]` | TDM-side read requests, one lane per TDM fetch beat. |
| `tdm_resp_i` | input | `obi_resp_t [NUM_TDM-1:0]` | TDM-side grants and read responses used to fill slots. |
| `active_mode` | input | `logic [1:0]` | Active drain width. `2'b11` aliases mode16. |
| `fetch_addr_i` | input | `logic [NUM_TDM-1:0][31:0]` | External `NUM_TDM`-wide address set for the next fetch window. |
| `fetch_addr_valid_i` | input | `logic` | Active-high strobe. When asserted at a legal boundary, the buffer latches `fetch_addr_i`. |

### SystemC

The corrected SystemC model should expose the same logical interface. It may continue to represent OBI as scalar arrays instead of packed structs, but the signal names should follow the canonical naming in new code or wrappers.

| Signal | Direction | Suggested SystemC type | Meaning |
| ------ | --------- | ---------------------- | ------- |
| `clk_i` | input | `sc_in<bool>` | Clock. |
| `rst_ni` | input | `sc_in<bool>` | Active-low asynchronous reset. |
| `active_mode` | input | `sc_in<sc_uint<2>>` or `sc_in<uint32_t>` | Same 2-bit mode encoding as RTL. |
| `fetch_addr_i` | input | `sc_in<uint64_t> [NUM_TDM]` | External address set; low 32 bits map to RTL `addr`. |
| `fetch_addr_valid_i` | input | `sc_in<bool>` | Active-high latch strobe for `fetch_addr_i`. |
| `port_req_i` / `port_resp_o` | mixed | scalar arrays or wrapper structs | Port-facing OBI. |
| `tdm_req_o` / `tdm_resp_i` | mixed | scalar arrays or wrapper structs | TDM-facing OBI. |

## Parameters

| Parameter | Default | Meaning |
| --------- | ------- | ------- |
| `PORT_COUNT` | 4 | Number of Port-facing ports. Implemented as `PORT_COUNT` in both targets. |
| `NUM_REQ` | 4 | Number of OBI buses per port. |
| `NUM_TDM` | 32 | Number of TDM lanes and slots in one fetch window. Implemented as `NUM_TDM` in both targets. |
| `BUFFER_SIZE` | 32 | Number of stored slots. For the first target, `BUFFER_SIZE == NUM_TDM`. |
| `NUM_IO` | `PORT_COUNT * NUM_REQ` | Total Port-side beats. SystemC keeps `TOTAL_PORT` only as a compatibility alias. |
| `BYTES_PER_ROW` | 16 in SystemC | Data word width in bytes. In SystemVerilog this comes from ``OBI_DATA_WIDTH / 8``. |

Remove the old SystemVerilog `MODE_4`, `MODE_8`, and `MODE_16` parameters. Runtime mode selection comes only from `active_mode`.

Both implementations require `NUM_TDM >= NUM_IO` and `NUM_TDM % NUM_IO == 0` for this first target, so the full window drains without an unreachable tail. Both implementations should at least be written for:

- `PORT_COUNT = 4`
- `NUM_REQ = 4`
- `NUM_TDM = BUFFER_SIZE = 32`
- `NUM_IO = 16`

## Active Modes

`active_mode` uses the same encoding in both implementations:

| `active_mode` | Name | Active ports | Active beats | Drain chunk |
| ------------- | ---- | ------------ | ------------ | ----------- |
| `2'b00` | mode4 | port 0 | 4 | slots `rd_ptr + 0 .. rd_ptr + 3` |
| `2'b01` | mode8 | ports 0-1 | 8 | slots `rd_ptr + 0 .. rd_ptr + 7` |
| `2'b10` | mode16 | ports 0-3 | 16 | slots `rd_ptr + 0 .. rd_ptr + 15` |
| `2'b11` | mode16 alias | ports 0-3 | 16 | same as `2'b10` |

Do not use the old port-strided mapping from `buffer.sv` as the target behavior. The canonical drain order is linear contiguous chunks.

## Slot State Machine

Each slot shall track one of three states:

| State | Meaning |
| ----- | ------- |
| `MISSING` | Slot has no fresh data and should be fetched. |
| `VALID` | Slot has received TDM read data and can be drained by the AGU side. |
| `INVALID` | Slot has already been consumed and waits for full-window recycle. |

On reset:

- every slot becomes `MISSING`,
- stored data is cleared or treated as invalid,
- per-slot TDM request-granted tracking is cleared,
- the drain pointer `rd_ptr_q` returns to zero,
- the buffer waits for or uses a valid fetch-address set, then enters fetch mode.

## Fetch Address Latching

An external unit generates one `NUM_TDM`-wide address set per buffer. The buffer latches that set when `fetch_addr_valid_i` is asserted at a legal boundary. A legal boundary is either reset/startup before issuing a fetch, or full-window recycle before issuing the next fetch.

Latched addresses are held in `fetch_addr_q[t]` and drive `tdm_req_o[t].addr` until lane `t` receives its response. This is required because a TDM lane may retry after conflicts or delayed grants.

If `fetch_addr_valid_i` is not asserted when the buffer needs a new address set, the buffer remains in `MISSING`/idle fetch-start state and does not issue new TDM requests.

## TDM Fetch Behavior

The TDM side is a `NUM_TDM`-wide read interface with one OBI lane per slot. The lane-to-slot mapping is linear: lane `t` fills slot `t`. Any crossbar or interconnect reordering must be handled before data reaches this buffer interface.

During fetch mode:

- `tdm_req_o[t].req` stays asserted for each slot whose request has not yet been granted.
- A per-slot `req_granted_q[t]` bit records that lane `t` has been accepted and prevents duplicate requests before its response returns.
- `tdm_req_o[t].addr` carries `fetch_addr_q[t]`.
- `tdm_req_o[t].we` is false.
- `tdm_req_o[t].be` is the full-word byte mask.
- When `tdm_resp_i[t].rvalid` is asserted, `tdm_resp_i[t].rdata` is stored in slot `t` and the slot becomes `VALID`.

The buffer does not need the entire full window to be valid before starting AGU drain. Drain eligibility is checked per current active drain window.

## AGU Drain Behavior

A group can drain when all of these are true:

- every slot in the current active drain window is `VALID`, including same-cycle TDM fills through forwarding/next-state valid logic,
- every active `port_req_i[p][r].req` is asserted,
- the selected drain indices are in range.

When the group drains:

- `port_resp_o[p][r].gnt` is asserted combinationally for active buses,
- `port_resp_o[p][r].rvalid` is asserted one cycle after grant,
- `port_resp_o[p][r].rdata` returns the corresponding slot data,
- drained slots become `INVALID`,
- `rd_ptr_q` advances by the number of active buses.

Inactive ports do not receive grants or valid responses.

## Drain Order

The canonical drain order is linear by active chunk size:

```text
active_beats = active_ports * NUM_REQ
index        = rd_ptr_q + active_beat_index
rd_ptr_next  = rd_ptr_q + active_beats
```

With default `NUM_REQ = 4`:

- mode4 drains slots `0..3`, then `4..7`, then `8..11`, and so on,
- mode8 drains slots `0..7`, then `8..15`, then `16..23`, and so on,
- mode16 drains slots `0..15`, then `16..31`.

The implementation never grants partial groups.

## Same-Cycle Fill And Drain

Same-cycle fill-and-drain forwarding is required. For example, in mode4 with `rd_ptr_q = 0`, slots `0..2` are already `VALID`, slot `3` is still `MISSING`, and all four active port requests are asserted. If `tdm_resp_i[3].rvalid` arrives in this cycle, the buffer treats slot `3` as valid for the drain decision and grants the AGU group in the same cycle.

Required precedence:

- Fill only: store `rdata` and mark the slot `VALID`.
- Drain only: return stored data and mark the slot `INVALID`.
- Fill and drain the same slot: forward incoming `rdata` to the port response path and leave the slot `INVALID` after the clock edge.
- Fill and drain different slots: apply both updates independently.

The current `buffer.sv` same-entry overwrite behavior is not allowed because it loses the new data without forwarding it.

## Recycle Behavior

After all slots in the `NUM_TDM` window have drained and become `INVALID`:

1. all slots return to `MISSING`,
2. request-granted tracking for all slots is cleared,
3. `rd_ptr_q` wraps to zero,
4. the buffer waits for/latches the next `fetch_addr_i` set,
5. the next full-window fetch begins.

## Mode Changes

`active_mode` is expected to change only at clean buffer boundaries: when the buffer is fully `MISSING` before a fetch starts, or when the fetched window is fully `VALID` before any drain has consumed slots. It must not change while the buffer contains a mixture of `VALID` and `INVALID` slots. The first RTL implementation does not need to assert or guard this rule; surrounding control logic is responsible for respecting it.

## Reads And Writes

This buffer is a read-prefetch buffer. Port-side write support should be implemented separately. A write buffer would collect 4, 8, or 16 AGU write beats and issue them to the TDM side as a grouped write operation.

## Future Ping-Pong Reuse

A future variant may avoid some TDM fetches by keeping one line from the previous fetch and reusing it when the next access overlaps. That ping-pong/cache-like behavior may require `NUM_TDM != BUFFER_SIZE` or additional valid/hit metadata. It is out of scope for the first implementation; the current drain contract remains contiguous 4-, 8-, or 16-beat chunks from the active window.

## Implementation Compliance

### SystemVerilog `buffer.sv`

Compliance checklist:

| Area | Status |
| ---- | --------------- |
| Parameters | Done. `MODE_4`, `MODE_8`, and `MODE_16` are removed; `BUFFER_SIZE == NUM_TDM`, `PORT_COUNT in [1,4]`, and `NUM_TDM % NUM_IO == 0` are checked for this target. |
| Ports | Done. `fetch_addr_i [NUM_TDM][31:0]` and `fetch_addr_valid_i` are present. |
| Slot state | Done. Uses explicit `slot_state_q` plus `slot_data_q`. |
| Fetch requests | Done. `tdm_req_o` is driven from `fetch_addr_q`, `req_granted_q`, and `slot_state_q`. |
| Address hold | Done. Address sets are latched at startup/recycle and held across TDM retries. |
| Drain order | Done. Drain indices are linear from `rd_ptr_q`. |
| Active mode | Done. `default` maps to mode16/max available ports. |
| Same-cycle forwarding | Done. Grant and response data can use same-cycle `tdm_rvalid/rdata`. |
| Recycle | Done. Full-window recycle resets states and can latch the next address set in the same cycle. |

### SystemC `buffer.hpp`

Compliance checklist:

| Area | Status |
| ---- | --------------- |
| Naming/parameters | Done. Uses `PORT_COUNT`, `NUM_TDM`, `NUM_IO`, and `active_mode`; `TOTAL_PORT` remains only as an alias. `NUM_TDM % NUM_IO == 0` is statically checked. |
| Active mode | Done. Uses the shared 2-bit mode encoding. |
| Address valid | Done. `fetch_addr_i` latches only when `fetch_addr_valid_i` is high. |
| Early drain | Done. Grant checks only the current active drain window. |
| Same-cycle forwarding | Done. The sequential model fills before checking drain. |
| Naming wrappers | Partially done. `fetch_addr_i` and `active_mode` are canonical; scalar `a_*`/`m_*` OBI arrays remain as the current SystemC representation. |

## Open Questions

No open questions remain for the first read-buffer target. Future work may reopen ping-pong reuse, write-buffer behavior, or `NUM_TDM != BUFFER_SIZE` support.
