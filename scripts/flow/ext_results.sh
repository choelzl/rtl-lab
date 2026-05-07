#!/usr/bin/env bash

# -----------------------------------------------------------------------------
# Author: Simone Machetti
# -----------------------------------------------------------------------------

set -uo pipefail

cd "$CODE_HOME/ai-core" || exit 1

IMP_DIR="imp"
DOC_DIR="doc"
DATA_DIR="$DOC_DIR/data"
AREA_DIR="$DATA_DIR/area"
FREQ_DIR="$DATA_DIR/freq"
POWER_DIR="$DATA_DIR/power"
CLK_NS=1.35

DESIGNS=(
    "bas_4x8|Baseline 4x8"
    "bas_8x8|Baseline 8x8"
    "bas_4x8_sc|Baseline 4x8 SC"
    "win_4x8|Winograd 4x8"
    "win_4x8_sc|Winograd 4x8 SC"
    "sqr_4x8_sc|Square 4x8 SC"
    "alpha|Alpha"
    "alpha_sqr|Alpha Squared"
)

# -----------------------------------------------------------------------------
# Extraction helpers
# -----------------------------------------------------------------------------
extract_area() {
    local name="$1"
    local rpt="$IMP_DIR/${name}_syn/report/area.rpt"
    grep "Chip area for module" "$rpt" | awk '{printf "%.2f", $NF}'
}

extract_slack_ps() {
    local name="$1"
    local rpt="$IMP_DIR/${name}_post_syn_sta/report/critical_paths.rpt"
    grep -E "slack \((MET|VIOLATED)\)" "$rpt" | awk '{print $1}'
}

extract_power_w() {
    local name="$1"
    local rpt="$IMP_DIR/${name}_post_syn_dpa/report/power_summary.rpt"
    grep "^Total" "$rpt" | awk '{print $5}'
}

slack_to_freq_mhz() {
    local slack_ps="$1"
    awk -v clk="$CLK_NS" -v s="$slack_ps" 'BEGIN { printf "%.2f", 1000 / (clk - s/1000) }'
}

w_to_mw() {
    local watts="$1"
    awk -v w="$watts" 'BEGIN { printf "%.3f", w * 1000 }'
}

# -----------------------------------------------------------------------------
# Output writers
# -----------------------------------------------------------------------------
write_md() {
    local title="$1" h1="$2" h2="$3" out="$4"
    local -n _md_rows=$5

    local w1=${#h1} w2=${#h2}
    for row in "${_md_rows[@]}"; do
        local label="${row%%|*}" val="${row##*|}"
        (( ${#label} > w1 )) && w1=${#label}
        (( ${#val}   > w2 )) && w2=${#val}
    done

    local sep1; sep1=$(printf '%*s' $((w1+2)) '' | tr ' ' '-')
    local sep2; sep2=$(printf '%*s' $((w2+2)) '' | tr ' ' '-')

    {
        echo "# $title"
        echo ""
        printf "| %-${w1}s | %${w2}s |\n" "$h1" "$h2"
        echo "|${sep1}|${sep2}|"
        for row in "${_md_rows[@]}"; do
            local label="${row%%|*}" val="${row##*|}"
            printf "| %-${w1}s | %${w2}s |\n" "$label" "$val"
        done
    } > "$out"
    echo "  Wrote $out"
}

write_area_csv() {
    local -n _rows=$1
    local out="$AREA_DIR/area.csv"
    {
        echo "design,area_um2"
        for row in "${_rows[@]}"; do
            local label="${row%%|*}"
            local val="${row##*|}"
            echo "\"$label\",$val"
        done
    } > "$out"
    echo "  Wrote $out"
}

write_freq_csv() {
    local -n _rows=$1
    local out="$FREQ_DIR/freq.csv"
    {
        echo "design,freq_mhz"
        for row in "${_rows[@]}"; do
            local label="${row%%|*}"
            local val="${row##*|}"
            echo "\"$label\",$val"
        done
    } > "$out"
    echo "  Wrote $out"
}

write_power_csv() {
    local -n _rows=$1
    local out="$POWER_DIR/power.csv"
    {
        echo "design,power_mw"
        for row in "${_rows[@]}"; do
            local label="${row%%|*}"
            local val="${row##*|}"
            echo "\"$label\",$val"
        done
    } > "$out"
    echo "  Wrote $out"
}

# -----------------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------------
mkdir -p "$DOC_DIR/diagrams" "$DOC_DIR/charts" "$AREA_DIR" "$FREQ_DIR" "$POWER_DIR"

area_rows=()
freq_rows=()
power_rows=()

for entry in "${DESIGNS[@]}"; do
    name="${entry%%|*}"
    label="${entry##*|}"

    area=$(extract_area      "$name")
    slack=$(extract_slack_ps "$name")
    freq=$(slack_to_freq_mhz "$slack")
    power_w=$(extract_power_w "$name")
    power=$(w_to_mw "$power_w")

    area_rows+=("$label|$area")
    freq_rows+=("$label|$freq")
    power_rows+=("$label|$power")
done

echo ""
echo "Writing markdown tables..."
write_md "Area"            "Design" "Area (um²)" "$AREA_DIR/area.md"   area_rows
write_md "Frequency (max)" "Design" "Freq (MHz)" "$FREQ_DIR/freq.md"   freq_rows
write_md "Power"           "Design" "Power (mW)" "$POWER_DIR/power.md" power_rows

echo ""
echo "Writing CSV files..."
write_area_csv  area_rows
write_freq_csv  freq_rows
write_power_csv power_rows

echo ""
echo "Done."
