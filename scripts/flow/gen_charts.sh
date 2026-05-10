#!/usr/bin/env bash

# -----------------------------------------------------------------------------
# Author: Simone Machetti
# -----------------------------------------------------------------------------

set -uo pipefail

CHARTS_DIR="$CODE_HOME/ai-core/doc/charts"

echo "Generating area charts..."
(cd "$CHARTS_DIR/area"  || exit 1; python3 area_pe_level.py && python3 area_ai_core_level.py 8 && python3 area_ai_core_level.py 16)

echo "Generating frequency charts..."
(cd "$CHARTS_DIR/freq"  || exit 1; python3 freq_pe_level.py && python3 freq_ai_core_level.py)

echo "Generating power charts..."
(cd "$CHARTS_DIR/power" || exit 1; python3 pwr_pe_level.py  && python3 pwr_ai_core_level.py 8 && python3 pwr_ai_core_level.py 16)

echo ""
echo "Done."
