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
#define WORDS_PER_ROW 1
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
// AGU / manager side
// ---------------------------------------------------------------------------
#ifndef N_AGU
#define N_AGU 8
#endif
#ifndef N_REQ
#define N_REQ 4
#endif
#ifndef N_MGR
#define N_MGR (N_AGU * N_REQ)
#endif

// ---------------------------------------------------------------------------
// L1 crossbar  (NUM_L1 switches, each L1_NIN-in × L1_NOUT-out)
// ---------------------------------------------------------------------------
#ifndef L1_NIN
#define L1_NIN 4
#endif
#ifndef L1_NOUT
#define L1_NOUT 4
#endif
#ifndef NUM_L1
#define NUM_L1 (N_MGR / L1_NIN)
#endif

// ---------------------------------------------------------------------------
// L2 crossbar  (NUM_L2 switches, each L2_NIN-in × L2_NOUT-out)
// ---------------------------------------------------------------------------
#ifndef L2_NIN
#define L2_NIN 8
#endif
#ifndef L2_NOUT
#define L2_NOUT 8
#endif
#ifndef NUM_L2
#define NUM_L2 (N_BANK / L2_NOUT)
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
