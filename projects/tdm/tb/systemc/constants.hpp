#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

// ---------------------------------------------------------------------------
// Bank
// ---------------------------------------------------------------------------
#ifndef N_BANK
#define N_BANK 32
#endif
#ifndef N_ROW
#define N_ROW 1024
#endif
#ifndef WORDS_PER_ROW
#define WORDS_PER_ROW 4
#endif
#ifndef WORD_BYTES
#define WORD_BYTES 4
#endif
#ifndef BYTES_PER_ROW
#define BYTES_PER_ROW (WORDS_PER_ROW * WORD_BYTES)
#endif
#ifndef BANK_BYTES
#define BANK_BYTES (N_ROW * BYTES_PER_ROW)
#endif

// ---------------------------------------------------------------------------
// DUT port side — per-unit driver group counts (not flat port counts)
// These name the number of RAGU/WAGU driver instances per group.
// Flat OBI port counts are derived inside top.hpp as N_RAGU_X * N_REQ.
// ---------------------------------------------------------------------------
#ifndef N_RAGU_A
#define N_RAGU_A 4
#endif
#ifndef N_RAGU_B
#define N_RAGU_B 2
#endif
#ifndef N_RAGU_C
#define N_RAGU_C 1
#endif
#ifndef N_RAGU_D
#define N_RAGU_D 1
#endif
#ifndef N_RAGU_E
#define N_RAGU_E 1
#endif

#ifndef N_WAGU_A
#define N_WAGU_A 4
#endif
#ifndef N_WAGU_B
#define N_WAGU_B 2
#endif
#ifndef N_WAGU_D
#define N_WAGU_D 1
#endif
#ifndef N_WAGU_E
#define N_WAGU_E 1
#endif

// ---------------------------------------------------------------------------
// DUT port side — aggregates (derived from per-unit counts)
// ---------------------------------------------------------------------------
#ifndef N_RPORT
#define N_RPORT (N_RAGU_A + N_RAGU_B + N_RAGU_C + N_RAGU_D + N_RAGU_E)
#endif
#ifndef N_WPORT
#define N_WPORT (N_WAGU_A + N_WAGU_B + N_WAGU_D + N_WAGU_E)
#endif
#ifndef N_REQ
#define N_REQ 4
#endif

// ---------------------------------------------------------------------------
// OBI data-path widths  (mirror the SV macros OBI_DATA_W / OBI_BE_W)
// ---------------------------------------------------------------------------
#ifndef OBI_DATA_W
#define OBI_DATA_W (BYTES_PER_ROW * 8)
#endif
#ifndef OBI_BE_W
#define OBI_BE_W BYTES_PER_ROW
#endif

// ---------------------------------------------------------------------------
// Simulation
// ---------------------------------------------------------------------------
#ifndef CLK_PERIOD_NS
#define CLK_PERIOD_NS 10
#endif

#endif // CONSTANTS_HPP
