#ifndef OBI_DATA_HPP
#define OBI_DATA_HPP
// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// obi_data<BYTES> — width-generic OBI data word backed by sc_bv<BYTES*8>.
//
// Using sc_bv gives native sc_signal<T> compatibility, sc_trace support,
// and integer conversion methods (to_uint64(), to_uint()) without a custom
// type.  BYTES is kept as the template parameter so call sites read naturally
// (e.g. obi_data<BYTES_PER_ROW>) and track the byte-width convention used
// everywhere else in the design.
// -----------------------------------------------------------------------------

#include <systemc.h>

template <int BYTES>
using obi_data = sc_bv<BYTES * 8>;

#endif
