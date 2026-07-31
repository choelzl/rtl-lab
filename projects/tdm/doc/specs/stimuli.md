# Stimuli file format

Stimuli live under `tb/stimuli/<case>/`, one `ragu_*`/`wagu_*` file per driver
group (e.g. `ragu_a`, `wagu_d`, `ragu_e`), named with either a `.log` or a
`.csv` extension — `agu.hpp`/`lane_agu.hpp` try whichever one exists
(`csv_parse_util.hpp`'s `resolve_stim_path()`), so both spellings of a set can
coexist and callers never need to know which one is on disk. `agu.hpp`'s
per-line parser also ignores any fields past `addr`/`addr,data` (e.g. a
trailing `,# N` comment some trace exports carry), so lines with extra
annotation columns still parse correctly. There are two distinct file
formats in use, driven by two different AGU classes — they share the same
descriptor-then-address-lines shape, but differ in field count and meaning:

| Groups | Driver | Format |
| ------ | ------ | ------ |
| `RAGU_A`/`B`/`C`/`D`, `WAGU_A`/`B`/`D` | [`agu.hpp`](../../tb/systemc/agu.hpp) | grouped, TDM-geometry descriptor (below) |
| `RAGU_E`, `WAGU_E` | [`lane_agu.hpp`](../../tb/systemc/lane_agu.hpp) | independent sub-port descriptor (below) |

Both formats are plain text, one task descriptor line (`#...`) followed by a
variable number of address lines, repeated for as many tasks as the trace
has. Addresses are **byte addresses** in both formats — the drivers apply no
scaling. Production trace exports that are row-indexed (one count per 16-byte
row) must be converted (×16) in the files before use; feeding row indices as-is
folds every 16 consecutive addresses onto one row and shows up as a massive,
fabric-independent same-row conflict rate (this exact failure is documented in
the design report, §5.2). A `#` line's `cycle` field fences its address lines: the driver won't
start issuing them until its own `cycle_` counter reaches that value AND any
prior task has finished (see each driver's own fencing behavior for exactly
what "finished" means — it differs between grouped and per-sub-port
progress).

## RAGU_A/B/C/D, WAGU_A/B/D format (`agu.hpp`)

```
#cycle,num_port_active,R,C,L,storemode
0x00000200
0x00000210
...
#next_cycle,...
0x00001000
...
```

Descriptor fields:

| Field | Meaning |
| ----- | ------- |
| `cycle` | earliest start cycle for this task (fence) |
| `num_port_active` | number of active port *groups*; `ports_used = num_port_active * N_PER_GROUP` |
| `R`, `C`, `L` | access-pattern geometry passed to the TDM mapping function — omit all three for a short 3-field descriptor `#cycle,num_port_active,storemode` with no TDM mapping geometry (`has_crl = false`) |
| `storemode` | passed to the TDM mapping function (`tdm.hpp`) — see [map_func.md](map_func.md); last field of the full descriptor, third field of the short one |

Address lines follow each descriptor, one per active port, grouped
`num_port_active` groups of `N_PER_GROUP` lines at a time:

- **RAGU**: `addr` only (implicit read) — e.g. `0x00000200`.
- **WAGU**: `addr,data` (implicit write) — e.g. `0x000001f0,0x01`. `data` is a
  plain hex integer read via `strtoull`, so it silently truncates past 64
  bits — this format has no wide/multi-beat transfer notion (see
  `agu.hpp`'s `parse_addr_line()`).

An address line that's the bare word `addr` (no value) is a legacy header
placeholder and is skipped.

## RAGU_E/WAGU_E format (`lane_agu.hpp`)

DMA's access pattern and hardware are different enough (see
[tdm.md](tdm.md)'s buffer table — `buf_r4`/`buf_w3`) that it uses a
dedicated 7-field descriptor and its own driver rather than `agu.hpp`:

```
#cycle,rate,sub_port_id,store_mode,C,R,L
0x00022e40,32
0x00022e60,32
...
#next_cycle,...
...
```

Descriptor fields:

| Field | Meaning |
| ----- | ------- |
| `cycle` | earliest start cycle for this task (fence), same semantics as `agu.hpp` |
| `rate` | **unused** — verified to just be `cycle[k]-cycle[k-1]` from the raw interleaved trace, not consumed by the driver |
| `sub_port_id` | `0` or `1`. DMA's 4 physical OBI lanes are split into two independent, asynchronously-progressing sub-ports — sub-port 0 owns lanes `{0,1}`, sub-port 1 owns lanes `{2,3}`. Descriptors for both sub-ports are interleaved in the same file; each sub-port's own task queue is built by filtering on this field and consumed completely independently of the other (see `lane_agu.hpp`'s class-level comment for why this needs its own driver). |
| `store_mode`, `C`, `R`, `L` | present but **not meaningful for DMA** (same as `agu.hpp`'s `has_crl=false` fallback) — parsed only to validate field count, never threaded through to the TDM mapping function |

Unlike the grouped format above, each descriptor is followed by a
**variable** number of address lines (observed: 1, 2, 20, 42, 66... in real
traces) — every address line is its own independent transfer for that
sub-port, issued as fast as that sub-port's own lane(s) allow (not grouped
lockstep with other lanes).

Address line format:

- **RAGU (read)**: `addr,width`
- **WAGU (write)**: `addr,data,width`

`width` is in **bytes** (not bits) and must be in `(0, 2*BYTES_PER_ROW]`:

- `width <= BYTES_PER_ROW` (e.g. `16`): single beat on the sub-port's
  *primary* lane only; `be_o` enables just the low `width` bytes. The
  secondary lane sits idle (NOP) for the whole transfer.
- `width > BYTES_PER_ROW` (e.g. `24` or `32`): two beats — primary lane at
  `addr`, always a full beat, and secondary lane at `addr + kDmaWideOffset`
  covering the remaining `width - BYTES_PER_ROW` bytes, byte-enabled to just
  that many (full only when `width == 2*BYTES_PER_ROW` exactly, e.g. `32`).
  `data` is `width*2` hex chars (e.g. 48 hex chars for `width=24`, 64 for
  `width=32`), constructed via `sc_bv`'s string constructor rather than
  `strtoull`, so — unlike the grouped format above — it does *not* truncate
  wide (up to 256-bit) payloads. The hex string is split with the primary
  lane's (always-full) payload as the *last* `BYTES_PER_ROW*2` characters and
  the secondary lane's (possibly partial) payload as whatever precedes that.

Any `width` value outside `(0, 2*BYTES_PER_ROW]` is rejected with a fatal
error rather than silently mishandled.
