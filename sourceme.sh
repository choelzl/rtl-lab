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
else 
    source $HOME/.bashrc
fi

unset _SCRIPT_DIR

