#!/bin/bash

# -----------------------------------------------------------------------------
# Author: Simone Machetti, Cedric Hoelzl
# Description: This script sets up environment variables for EDA tools used in the RTL lab.
# -----------------------------------------------------------------------------

# CODE_HOME: parent of this repo (derived from the script's own location).
# Pre-set it in your .envrc to override.
_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export CODE_HOME="${CODE_HOME:-$(dirname "$_SCRIPT_DIR")}"

# When sourced outside a direnv shell (e.g. by a coding agent or CI), bootstrap the
# .envrc so Nix-provided tool paths (SystemC, Verilator, …) are available.
if [ -z "${DIRENV_DIR:-}" ] && command -v direnv &>/dev/null && [ -f "$_SCRIPT_DIR/.envrc" ]; then
    eval "$(cd "$_SCRIPT_DIR" && direnv export bash 2>/dev/null)"
fi

unset _SCRIPT_DIR

# TOOLS_HOME: root of installed EDA tools.
# Pre-set it in your .envrc to override; falls back to /my_tools.
export TOOLS_HOME="${TOOLS_HOME:-/my_tools}"

# Verilator
export VERILATOR_HOME="${VERILATOR_HOME:-$TOOLS_HOME/verilator}"
export PATH=$VERILATOR_HOME/bin:$PATH

# Yosys
export YOSYS_HOME="${YOSYS_HOME:-$TOOLS_HOME/yosys}"
export PATH=$YOSYS_HOME/bin:$PATH

# Yosys Slang
export YOSYS_SLANG_HOME="${YOSYS_SLANG_HOME:-$TOOLS_HOME/yosys-slang}"
export PATH=$YOSYS_SLANG_HOME/bin:$PATH

# OpenSTA
export OPENSTA_HOME="${OPENSTA_HOME:-$TOOLS_HOME/opensta}"
export PATH=$OPENSTA_HOME/bin:$PATH

# OpenROAD
export OPENROAD_HOME="${OPENROAD_HOME:-$TOOLS_HOME/openroad}"
export PATH=$OPENROAD_HOME/bin:$PATH

# SystemC
export SYSTEMC_HOME="${SYSTEMC_HOME:-$TOOLS_HOME/systemc}"
export SYSTEMC_INCLUDE="${SYSTEMC_INCLUDE:-$SYSTEMC_HOME/include}"
if [ -z "$SYSTEMC_LIBDIR" ]; then
    if [ -d "$SYSTEMC_HOME/lib64" ]; then
        export SYSTEMC_LIBDIR=$SYSTEMC_HOME/lib64
    else
        export SYSTEMC_LIBDIR=$SYSTEMC_HOME/lib
    fi
fi
