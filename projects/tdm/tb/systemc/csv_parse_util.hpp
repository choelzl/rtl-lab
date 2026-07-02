// -----------------------------------------------------------------------------
// Tiny string/CSV parsing helpers shared by agu.hpp and lane_agu.hpp — both
// parse comma-separated trace lines, and trim() in particular was previously
// byte-identical copies in each. Kept generic and free-standing (no
// dependency on either AGU's own types) so both can share it without
// coupling their otherwise-independent parsers together.
// -----------------------------------------------------------------------------

#ifndef CSV_PARSE_UTIL_HPP
#define CSV_PARSE_UTIL_HPP

#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

inline std::string trim(const std::string &s) {
    const std::size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos)
        return std::string();
    const std::size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

// Splits on ',' and trims each token, keeping empty tokens (e.g. a trailing
// comma produces a trailing empty string) — used where the caller needs to
// distinguish "field present but empty" from "field absent".
inline std::vector<std::string> split_csv(const std::string &line) {
    std::vector<std::string> out;
    std::size_t              pos = 0;
    while (pos <= line.size()) {
        const std::size_t c = line.find(',', pos);
        out.push_back(trim(c == std::string::npos ? line.substr(pos) : line.substr(pos, c - pos)));
        if (c == std::string::npos)
            break;
        pos = c + 1;
    }
    return out;
}

inline uint64_t parse_hex_u64(const std::string &s) {
    return std::strtoull(s.c_str(), nullptr, 0);
}

// Strips an optional "0x"/"0X" prefix, returning pure hex digits.
inline std::string strip_0x(const std::string &s) {
    if (s.size() >= 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))
        return s.substr(2);
    return s;
}

#endif // CSV_PARSE_UTIL_HPP
