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
// DUT port side
// ---------------------------------------------------------------------------
#ifndef N_RPORT
#define N_RPORT 9
#endif
#ifndef N_WPORT
#define N_WPORT 8
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
