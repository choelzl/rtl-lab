#ifndef OBI_DATA_HPP
#define OBI_DATA_HPP
// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// obi_data<BYTES> — width-generic OBI data word backed by sc_bv<BYTES*8>,
// giving native sc_signal<T>/sc_trace support and integer conversions
// (to_uint64(), to_uint()) without a custom type. BYTES matches the
// byte-width convention used everywhere else in the design.
// -----------------------------------------------------------------------------

#include <systemc.h>

template <int BYTES> using obi_data = sc_bv<BYTES * 8>;

#endif
