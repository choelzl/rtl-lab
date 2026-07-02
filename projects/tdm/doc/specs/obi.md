# Simplified OBI link

A reduced **single-channel** subset of the Open Bus Interface (OBI) v1.6.0 — see [OBI-v1.6.0.pdf](../protocols/OBI-v1.6.0.pdf) — shared by all designs in this project (the [crossbar](crossbar.md) and the [TDM](tdm.md) interconnects).

> **Grouped use in the TDM datapath.** The TDM backend carries **`N` of these links in parallel** as one request group (one lane per TDM slot): `req`/`addr`/`wdata`/`gnt`/`rvalid`/`rdata` are per-lane, while `we`/`be` are broadcast to the whole group, and `addr == 0` marks an unused NOP lane that is granted without ever reaching a bank. There is no separate protocol — see [tdm.md](tdm.md) and the header of [tdm.hpp](../../rtl/systemc/tdm.hpp).

OBI is a point-to-point, request/grant **manager↔subordinate** protocol. We collapse it to a **single channel** carrying both request and response, using only these signals (plus the global `clk` / `reset_n`):

| Signal   | Dir         | Purpose                                                                            |
| -------- | ----------- | ---------------------------------------------------------------------------------- |
| `req`    | manager→sub | Request valid.                                                                     |
| `gnt`    | sub→manager | Request accepted — handshake completes on the rising `clk` where `req=1 && gnt=1`. |
| `addr`   | manager→sub | Word address.                                                                      |
| `we`     | manager→sub | **1 = write, 0 = read.**                                                           |
| `be`     | manager→sub | Byte enables (4 bits for a 32-bit word).                                           |
| `wdata`  | manager→sub | Write data (used when `we=1`).                                                     |
| `rvalid` | sub→manager | Response/completion valid this cycle (asserted for **both** reads and writes).     |
| `rdata`  | sub→manager | Read data (valid only when `we=0`).                                                |

## Handshake & rules

- A request is accepted on the rising `clk` where `req=1 && gnt=1`.
- The subordinate later raises `rvalid` to signal completion; `rdata` is valid for reads. **There is no `rready`** — the manager is always ready (equivalent to `rready` tied high). A manager treats `rvalid` as "this request is done".
- **In-order, pipelined (depth ≤ 2):** phases overlap — a manager may start the **address phase of its next request while the previous request's response phase is still completing**. Transactions stay strictly **in order**, and at most **two** are in flight on a path at once (one being responded, one being requested); there is no deeper queue and no out-of-order or multi-outstanding support. Responses therefore return in order over the established path — **no transaction IDs, queues, or deep tracking are needed** (the subordinate registers one response; the interconnect remembers one owner per subordinate).
- **Read vs. write is just the `we` bit on the same link**; a given run uses one mode (uniform R/W). 1 word = 32 bits, `be` = 4 bits.

## Dropped vs. full OBI

The separate A/R channel split, `rready`, `err`, transaction IDs (`aid`/`rid`), and all other optional signals.
