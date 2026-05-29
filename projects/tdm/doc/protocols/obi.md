# Simplified OBI link

A reduced **single-channel** subset of the Open Bus Interface (OBI) v1.6.0 — see [OBI-v1.6.0.pdf](OBI-v1.6.0.pdf) — shared by all designs in this project (the [crossbar](../diagrams/crossbar.md) and the [TDM](../diagrams/tdm.md) interconnects).

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
- **No outstanding / pipelined transactions:** a granted request **holds its connection open until the response closes it**; the next request on that path is issued only afterwards. When an interconnect connects a manager to a subordinate, the connection stays open from grant until the response is delivered, then frees. Responses therefore return over the already-established path — **no transaction IDs, queues, or outstanding-request tracking are needed**.
- **Read vs. write is just the `we` bit on the same link**; a given run uses one mode (uniform R/W). 1 word = 32 bits, `be` = 4 bits.

## Dropped vs. full OBI

The separate A/R channel split, `rready`, `err`, transaction IDs (`aid`/`rid`), and all other optional signals.
