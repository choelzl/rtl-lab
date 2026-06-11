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
// AGU / manager side
// ---------------------------------------------------------------------------
#ifndef N_AGU
#define N_AGU 7
#endif
#ifndef N_WAGU
#define N_WAGU 6
#endif
#ifndef N_REQ
#define N_REQ 4
#endif
#ifndef N_MGR
#define N_MGR (N_AGU * N_REQ)
#endif

// ---------------------------------------------------------------------------
// Lx crossbar Number
// ---------------------------------------------------------------------------
#ifndef NUM_L1
#define NUM_L1 N_AGU
#endif
#ifndef NUM_L2
#define NUM_L2 4
#endif
// ---------------------------------------------------------------------------
// L1 crossbar  (NUM_L1 switches, each L1_NIN-in × L1_NOUT-out)
// ---------------------------------------------------------------------------
#ifndef L1_NIN
#define L1_NIN N_REQ
#endif
#ifndef L1_NOUT
#define L1_NOUT NUM_L2
#endif

// ---------------------------------------------------------------------------
// L2 crossbar  (NUM_L2 switches, each L2_NIN-in × L2_NOUT-out)
// ---------------------------------------------------------------------------

#ifndef L2_NIN
#define L2_NIN NUM_L1
#endif
#ifndef L2_NOUT
#define L2_NOUT N_BANK / NUM_L2
#endif

// ---------------------------------------------------------------------------
// Write crossbar (separate 2-layer path for N_WAGU write AGUs; not yet
// connected to banks — ports exist for future wiring only)
// ---------------------------------------------------------------------------
// WL1: NUM_WL1 switches, each WL1_NIN-in × WL1_NOUT-out
#ifndef NUM_WL1
#define NUM_WL1 N_WAGU
#endif
#ifndef NUM_WL2
#define NUM_WL2 4
#endif
#ifndef WL1_NIN
#define WL1_NIN N_REQ
#endif
#ifndef WL1_NOUT
#define WL1_NOUT NUM_WL2
#endif
// WL2: NUM_WL2 switches, each WL2_NIN-in × WL2_NOUT-out
#ifndef WL2_NIN
#define WL2_NIN NUM_WL1
#endif
#ifndef WL2_NOUT
#define WL2_NOUT N_BANK / NUM_WL2
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
