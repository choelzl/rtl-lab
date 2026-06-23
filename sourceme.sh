#!/bin/bash

# -----------------------------------------------------------------------------
# Author: Cedric Hölzl
# Description: This script sets up environment variables for EDA tools used in the RTL Lab.
# -----------------------------------------------------------------------------

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RTL_LAB_HOME="${RTL_LAB_HOME:-$_SCRIPT_DIR}"

if [ -z "${DIRENV_DIR:-}" ] && command -v direnv &>/dev/null && [ -f "$_SCRIPT_DIR/.envrc" ]; then
    eval "$(cd "$_SCRIPT_DIR" && direnv export bash 2>/dev/null)"
else
    source ~/.bashrc

    : "${SYSTEMC_INCLUDE:=$SYSTEMC_HOME/include}"
    if [ -d "$SYSTEMC_HOME/lib64" ]; then
        : "${SYSTEMC_LIBDIR:=$SYSTEMC_HOME/lib64}"
    else
        : "${SYSTEMC_LIBDIR:=$SYSTEMC_HOME/lib}"
    fi
    : "${ASAP7_HOME:=$PDK_HOME/OpenROAD-flow-scripts/flow/platforms/asap7}"
    export SYSTEMC_INCLUDE SYSTEMC_LIBDIR ASAP7_HOME
fi

unset _SCRIPT_DIR
