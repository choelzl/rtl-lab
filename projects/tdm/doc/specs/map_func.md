# TDM address-mapping function

This is the placement scheme used by the TDM interconnect's mapping module
([tdm.hpp](../../rtl/systemc/tdm.hpp)). For the surrounding datapath (buffers,
arbiter, mux, reused crossbar) see [tdm.md](tdm.md); this document specifies the
mapping function **in isolation**: what its inputs mean and exactly how it turns
one byte address into a `(bank, row)` location.

## Purpose

A kernel issues, every slot, a group of words it wants in parallel. If those
words were simply word-interleaved across the banks (`bank = word % N_BANK`),
regular access patterns (matrix rows/columns, strided loops) would repeatedly
collide on the same bank. The mapping function instead **scatters** the address
space across the banks with an XOR-skew that depends on the kernel's access
pattern, so the words a kernel needs together tend to land on **distinct** banks.
It is a pure function — `(address, parameters) → (bank_id, row_id)` — with no
state, so it is deterministic: a write and a later read of the same address
resolve to the same location.

## Interface

| Direction | Signal       | Meaning                                                        |
| --------- | ------------ | -------------------------------------------------------------- |
| in        | `addr`       | byte address of the word to place                              |
| in        | `num_banks`  | number of memory banks (power of two)                          |
| in        | `bank_width` | width/granularity of one bank word, in bytes (power of two)    |
| in        | `R`          | size of the **Row** axis of the access pattern (power of two)  |
| in        | `C`          | size of the **Col** axis of the access pattern (power of two)  |
| in        | `L`          | size of the **Loop** axis of the access pattern (power of two) |
| in        | `store_mode` | linearization order of the R/C/L axes (one of 16 mode indices, below) |
| out       | `bank_id`    | selected bank, a 5-bit value (0..31)                           |
| out       | `row_id`     | selected bank-local row                                        |

The six non-address inputs are **kernel-wide constants** (they describe the
access pattern of the whole kernel, not one access). In the TDM design,
`R`/`C`/`L`/`store_mode` come from each task's `#`-descriptor line, parsed by
[agu.hpp](../../tb/systemc/agu.hpp) and selected per time slot by
`top_tdm.hpp`'s `map_cfg_comb()` (an inline mux over each buffer's own config,
keyed by the same arbiter selection used to pick which buffer's address is on
the mapping function's input that cycle); `num_banks` and `bank_width` are
fixed at build time to `N_BANK` and `BYTES_PER_ROW`. See [tdm.md](tdm.md).

## Parameter meaning

The kernel walks a 3-dimensional iteration space whose axes are **Row (R)**,
**Col (C)** and **Loop (L)** — e.g. the two tile dimensions of a matrix block
plus an outer reduction/channel loop. Because every size is a power of two, each
axis occupies a fixed, contiguous group of address bits; the number of bits an
axis spans is the trailing-zero count of its size:

```
b   = log2(num_banks)     // bank-select bits
e   = log2(bank_width)    // sub-word offset bits (within one bank word)
tzR = log2(R)             // address bits spanned by the Row axis
tzC = log2(C)             // address bits spanned by the Col axis
tzL = log2(L)             // address bits spanned by the Loop axis
```

(`log2` here is `floor(log2)`, i.e. the position of the top set bit; the trailing
zeros of a power of two equal its `log2`.)

- **`bank_width`** fixes the lowest `e` address bits as the offset *inside* a
  bank word — they pick a byte/element within the word a bank returns, so they
  take no part in choosing the bank or the row.
- **`num_banks`** fixes how many bank-select bits `b` the scheme produces.
- **`R`, `C`, `L`** fix the *widths* of the access-pattern axes, hence where the
  field boundaries fall in the address.
- **`store_mode`** fixes the *order* in which the axes are linearized into the
  address, hence which axis sits where — see below.

## Storage mode and the field boundaries

The address (above the `e` offset bits) is divided into three fields by two bit
boundaries `k1 ≤ k2`. `store_mode` chooses how those boundaries are computed from
the axis widths; the table below is the authoritative definition (it is the
`get_k_raw` function). Both boundaries are clamped to at least `e` so a field
never reaches into the sub-word offset.

| `store_mode` (index)  | `k1`              | `k2`                |
| --------------------- | ----------------- | ------------------- |
| `Loop_Row_Col`   (0)  | `max(tzC, e)`     | `max(tzR + tzC, e)` |
| `Loop_Col_Row`   (1)  | `max(tzR, e)`     | `max(tzC + tzR, e)` |
| `Row_Col_Loop`   (2)  | `max(tzL, e)`     | `max(tzL + tzC, e)` |
| `Col_Row_Loop`   (3)  | `max(tzL, e)`     | `max(tzR + tzL, e)` |
| `Row_Loop_Col`   (4)  | `max(tzC, e)`     | `max(tzL + tzC, e)` |
| `Col_Loop_Row`   (5)  | `max(tzR, e)`     | `max(tzL + tzR, e)` |
| `Loop_2x2_H`     (6)  | `max(tzC + 1, e)` | `max(tzR + tzC, e)` |
| `Loop_2x2_V`     (7)  | `max(tzR + 1, e)` | `max(tzC + tzR, e)` |
| `Loop_4x4_H`     (8)  | `max(tzC + 2, e)` | `max(tzR + tzC, e)` |
| `Loop_4x4_V`     (9)  | `max(tzR + 2, e)` | `max(tzC + tzR, e)` |
| `Loop_Row`       (10) | `max(tzC, e)`     | `max(tzR + tzC, e)` |
| `Row_Loop`       (11) | `max(tzL, e)`     | `max(tzL + tzC, e)` |
| `Loop_Row_Space` (12) | `max(tzR, e)`     | `max(tzC + tzR, e)` |
| `Loop_2i`        (13) | `max(tzR + 1, e)` | `max(tzC + tzR, e)` |
| `Loop_3i`        (14) | — unsupported     | — unsupported       |
| `Loop_4i`        (15) | `max(tzR + 2, e)` | `max(tzC + tzR, e)` |

`store_mode` is carried in the trace as the **integer index** in the first
column (0..15). Modes that share a formula produce identical boundaries
(`Loop_Row` ≡ `Loop_Row_Col`, `Row_Loop` ≡ `Row_Col_Loop`,
`Loop_Row_Space` ≡ `Loop_Col_Row`, `Loop_2i` ≡ `Loop_2x2_V`,
`Loop_4i` ≡ `Loop_4x4_V`); the `2x2`/`4x4`/`2i`/`4i` variants widen the first
boundary by 1 or 2 bits, which shifts a sub-tile of consecutive elements into
the bank index (a coarser interleave). `Loop_3i` (14) has no `get_k` case and
raises a fatal error if a trace carries it.

The production `get_k` is a thin wrapper over this table: with the opt-in
build define `TDM_GETK_GUARD` it additionally repairs the degenerate split
that arises when the mode's leading dimension has no trailing zeros (`k1`
collapses onto `e`, the `con` field vanishes and windows can fold pairwise
onto half the banks) by borrowing up to two `str` bits into `con`. The guard
is pattern-blind and off by default — see the design report, Appendix A.8,
for the evaluation.

## The three address fields

With the boundaries `k1, k2` and the bank-bit count `b`, the address is split
into three `b`-bit slices (each masked to the low `b` bits):

```
con = (addr >> e ) & (2^(k1 - e) - 1)   & (2^b - 1)   // bits [e , k1)  "contiguous"
str = (addr >> k1) & (2^(k2 - k1) - 1)  & (2^b - 1)   // bits [k1, k2)  "strided-A"
l   = (addr >> k2)                      & (2^b - 1)   // bits [k2, ..)  "strided-B"
```

- **`con`** — the *contiguous* field: the part of the address that runs
  consecutively before the first axis boundary. (When `k1 == e` this field is
  empty / zero.)
- **`str`** — the *first strided* field: the axis between the two boundaries.
- **`l`** — the *second strided* field: everything above `k2` (folded into `b`
  bits).

Notation below: `con[i]`, `str[i]`, `l[i]` are bit `i` (0 = LSB) of each slice;
bits at index `≥ b` are zero.

## Bank id — the XOR matrix

`bank_id` is a fixed 5-bit value; each bit is the XOR of a chosen subset of the
field bits (this is the skew that breaks up regular patterns):

```
bank_id[0] = str[1] ^ con[2] ^ l[1] ^ l[2]
bank_id[1] = str[2] ^ con[1] ^ l[1]
bank_id[2] = str[0] ^ str[4] ^ con[0] ^ con[1] ^ l[4]
bank_id[3] = str[0] ^ con[4] ^ l[0]
bank_id[4] = str[1] ^ str[3] ^ con[0] ^ con[3] ^ l[3]
```

Because the matrix is 5 bits wide and indexes field bits up to position 4, it
targets a **32-bank** memory: `bank_id ∈ [0, 32)`, so the design needs
`N_BANK ≥ 32` (the TDM module raises a fatal error otherwise).

## Row id

The row is simply the address above the bank-selection region:

```
row_id = addr >> (e + b)
```

Two addresses that the XOR matrix sends to the same bank therefore keep
**different rows** as long as they differ above bit `e + b` — so a same-bank
collision is a genuine *bank conflict* (serialized by the downstream arbiter),
not an *aliasing* of two addresses onto one physical location. Within a single
row, however, the placement must be injective (the `b`+ low bits must map the
words of interest to distinct banks) for read-back to stay correct.

## Worked example

Sample parameters `num_banks=32, bank_width=4, C=4, R=4, L=8, store_mode=0`
(`Loop_Row_Col`): `b = 5`, `e = 2`, `tzR = tzC = 2`, `tzL = 3`, so
`k1 = max(2,2) = 2`, `k2 = max(4,2) = 4`. Then `con = 0` (empty, since
`k1 == e`), `str = (addr>>2) & 3`, `l = (addr>>4) & 31`.

Take `addr = 0x04` (`0b000100`): drop the low `e=2` bits → word 1; `con = 0`,
`str = 1` (`str[0]=1`), `l = 0`. The matrix gives `bank_id[2]=str[0]=1` and
`bank_id[3]=str[0]=1`, all other bits 0 → `bank_id = 4 + 8 = 12`; `row_id =
0x04 >> 7 = 0`.

Mapping the first 8 consecutive words this way scatters them across 8 distinct
banks:

| `addr` | `bank_id` | `row_id` |
| ------ | --------- | -------- |
| `0x00` | 0         | 0        |
| `0x04` | 12        | 0        |
| `0x08` | 17        | 0        |
| `0x0c` | 29        | 0        |
| `0x10` | 8         | 0        |
| `0x14` | 4         | 0        |
| `0x18` | 25        | 0        |
| `0x1c` | 21        | 0        |

Eight distinct banks ⇒ no conflict for this slot. A different `store_mode` or
axis sizing (or an adverse access pattern) makes two or more of a slot's words
share a bank — the bank conflict the experiment measures.

## Use in the TDM design

The reused word-interleaved crossbar primitive (`crossbar.hpp` — not the
three-level [crossbar backend](crossbar.md)) decodes a *byte address* into
bank/row by beat-interleave (`bank = (addr/BYTES_PER_ROW) % N_BANK`,
`row = (addr/BYTES_PER_ROW) / N_BANK` — one 16-byte beat per slot). To route a
mapped word there, the TDM module re-encodes the placement as the byte address
the crossbar will decode back to the same `(bank_id, row_id)`:

```
emitted_addr = (row_id * N_BANK + bank_id) * BYTES_PER_ROW
```

So the mapping function decides the bank placement, and the crossbar's per-bank
round-robin arbiter resolves any collisions. See [tdm.md](tdm.md) for the
end-to-end flow.

## Notes / assumptions

- **Power-of-two sizes** — `num_banks`, `bank_width`, `R`, `C`, `L` are assumed
  powers of two; the derived bit counts use `floor(log2)` / trailing zeros.
  Non-power-of-two dimensions still map (the trailing-zero counts just become
  lossy summaries), but they are the one task-geometry class that produces
  residual bank collisions in production traces — quantified, with every
  evaluated remedy, in the design report's Appendix A.8.
- **32 banks** — the XOR matrix is hard-wired to 5 output bits, so the scheme is
  a 32-bank placement (`N_BANK ≥ 32`).
- **`bank_width` unit** — taken here as **bytes per bank word** (so `e =
  log2(bank_width)`); confirm against the real kernels.
- **Injectivity** — the scheme is deterministic and is a bijection over an
  address range that spans enough rows; for a single-row range the words of
  interest must land on distinct banks to avoid aliasing.
