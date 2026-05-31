# Extended OBI link (x-OBI, multi-word)

A multi-word extension of the [simplified single-channel OBI](obi.md). Where base OBI moves **one word** per channel, x-OBI moves a **group of `N` words** over a single channel in one transaction, with **per-word handshake** so a subset of the group can be issued at a time. It is the protocol used on the internal TDM datapath (AGU↔Buffer and Buffer↔OBI-Mux↔TDM).

It keeps base OBI's request/grant manager↔subordinate model and its in-order, shallow-pipelined behaviour; it only widens the data-carrying fields to `N` and makes the addressing/control fields **group-scoped**.

## Signals

Width `N` = words per group. Plus the global `clk` / `reset_n`.

| Signal   | Dir         | Width   | Purpose                                                                                                                              |
| -------- | ----------- | ------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| `req`    | manager→sub | `[N]`   | **Per-word** request valid. A manager may assert only a subset — e.g. re-request just the words a conflict left outstanding.         |
| `addr`   | manager→sub | scalar  | **Base** address of the group (a single reference point — see *Address reconstruction*).                                             |
| `we`     | manager→sub | scalar  | **1 = write, 0 = read**, uniform for the whole group.                                                                                |
| `be`     | manager→sub | scalar  | Byte enables, uniform for the whole group.                                                                                           |
| `wdata`  | manager→sub | `[N]`   | **Per-word** write data (used when `we = 1`).                                                                                        |
| `gnt`    | sub→manager | `[N]`   | **Per-word** grant. Word `w`'s request is accepted on the rising `clk` where `req[w] && gnt[w]`. A subset may be granted in a cycle. |
| `rvalid` | sub→manager | `[N]`   | **Per-word** response/completion valid (asserted for both reads and writes).                                                         |
| `rdata`  | sub→manager | `[N]`   | **Per-word** read data (valid only when `we = 0`).                                                                                   |

## Handshake & rules

- **Per-word handshake.** Word `w` is accepted on the rising `clk` where `req[w] && gnt[w]`. The words of a group may be granted **across several cycles** (*partial issue*): the manager keeps `req` asserted on the still-ungranted words and drops it on the granted ones. **`addr` (base), `we`, `be` stay constant for the whole group until all its words are granted**, and each word is identified by its **index** `w`, so a re-requested subset still maps to the same locations.
- **Responses** return later, per word, via `rvalid[w]` / `rdata[w]` (1-cycle subordinate latency in this project). As in base OBI there is no `rready` (the manager is always ready), responses stay in order per word, and pipelining is shallow.
- **Uniform group.** Because `we` and `be` are scalar, a group is **all-reads or all-writes** with uniform byte-enables — no mixed R/W within a group.
- **Group atomicity (manager convention in this project).** A manager treats a group as complete only when **all `N` words** have been granted/returned, and does not begin a *new* group until then (group-level back-pressure). Partial issue is purely the mechanism for finishing a group across multiple cycles under conflict.

## Address reconstruction

`addr` carries only the group's **base**. The mapping of each of the `N` words to an actual memory address is **deliberately not part of this protocol** — it is left to the consumer. In the TDM design the AGU sends each group's **access sequence directly to the TDM mapping function** (through the TDM config mux); the mapping reconstructs every word's address from the base plus that sequence. The sequence may be **arbitrary** (consecutive, strided, or otherwise). Consequently x-OBI transports **one base, not `N` addresses** — the per-word addresses never appear on the channel.

## Relation to base OBI

- **Added:** width `N` on the data-carrying fields (`req`, `gnt`, `rvalid`, `rdata`, `wdata`); per-word handshake enabling subset/partial issue.
- **Changed:** base OBI's `addr` becomes a single group **base**; `we` / `be` become **group-scoped scalars** (uniform group).
- **Not transported:** the `N` per-word addresses (reconstructed by the consumer, as above).
- **Dropped (same as base OBI):** the separate A/R channel split, `rready`, `err`, transaction IDs, and other optional signals — see [obi.md](obi.md).

## Widths on the TDM datapath

| Link                     | `N`                         |
| ------------------------ | --------------------------- |
| AGU ↔ Buffer             | `N_REQ` (default 4)         |
| Buffer ↔ OBI Mux ↔ TDM   | `N_AGU · N_REQ` (default 8) |

The Buffer is therefore a width converter between the two (e.g. 4 ↔ 8) — see [tdm.md](tdm.md).
