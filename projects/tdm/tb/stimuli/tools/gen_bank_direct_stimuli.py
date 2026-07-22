#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Converts a per-cycle, per-AGU bank-access trace (sim/tmp/intra_AGU_conflict_
# rates.csv: timestamp_req,agu_id,unique_banks,bank_list,total_requests,
# num_conflicts) into ragu_a/b/c.log stimuli in the existing agu.hpp grouped
# format (see doc/specs/stimuli.md).
#
# Each CSV row becomes one task (one group) in the matching AGU's own file:
# start_cycle = timestamp_req (fence, same semantics as any other trace),
# num_port_active rounded up to the driver's N_PER_GROUP=4 lane groups, and
# exactly total_requests address lines carrying the row's own bank_list.
#
# Bank targeting is direct, not hashed: address = bank_id * BYTES_PER_ROW,
# with all bits above the routing field left 0 (row=0). top_crossbar.hpp's
# default addr_hash() only mixes addr[8:6] with addr[11:9] (both 0 here) and
# leaves addr[5:4] alone (L1 select, untouched unless XBAR_HASH_L1 is built),
# so this address reaches the crossbar's L1/L2 select bits completely
# unmodified: bank_id lands on that exact logical bank with no DUT changes,
# no inverse-hash math, and no dependence on which hash build is active.
# -----------------------------------------------------------------------------

import argparse
import ast
import csv
import math
import pathlib

BYTES_PER_ROW = 16  # WORDS_PER_ROW(4) * WORD_BYTES(4), constants.hpp default
N_PER_GROUP = 4  # dut_t::NUM_REQ

# agu_id -> (output file stem, max flat lanes for that driver group)
AGU_MAP = {
    "0": ("ragu_a", 16),  # RAGU_A: 4 port instances * 4 lanes
    "1": ("ragu_b", 8),  # RAGU_B: 2 port instances * 4 lanes
    "2": ("ragu_c", 4),  # RAGU_C: 1 port instance * 4 lanes
}


def convert(csv_path: pathlib.Path, out_dir: pathlib.Path) -> None:
    tasks = {name: [] for name, _ in AGU_MAP.values()}

    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            agu_id = row["agu_id"].split(".")[0]
            if agu_id not in AGU_MAP:
                raise ValueError(f"unexpected agu_id {row['agu_id']!r}")
            name, max_ports = AGU_MAP[agu_id]

            total_requests = int(row["total_requests"])
            if total_requests <= 0:
                continue
            if total_requests > max_ports:
                raise ValueError(
                    f"{name}: total_requests={total_requests} exceeds {max_ports} lanes "
                    f"(cycle {row['timestamp_req']})"
                )

            ports_used = min(max_ports, math.ceil(total_requests / N_PER_GROUP) * N_PER_GROUP)
            num_port_active = ports_used // N_PER_GROUP

            bank_list = [int(float(b)) for b in ast.literal_eval(row["bank_list"])]
            if len(bank_list) != total_requests:
                raise ValueError(
                    f"{name}: bank_list length {len(bank_list)} != total_requests "
                    f"{total_requests} (cycle {row['timestamp_req']})"
                )
            for b in bank_list:
                if not (0 <= b < BYTES_PER_ROW * 2):  # sanity: NUM_BANK==32 default
                    raise ValueError(f"{name}: bank_id {b} out of range (cycle {row['timestamp_req']})")

            cycle = int(row["timestamp_req"])
            addrs = [b * BYTES_PER_ROW for b in bank_list]
            tasks[name].append((cycle, num_port_active, addrs))

    out_dir.mkdir(parents=True, exist_ok=True)
    for name, _ in AGU_MAP.values():
        lines = []
        for cycle, num_port_active, addrs in tasks[name]:
            lines.append(f"#{cycle},{num_port_active},0")
            lines.extend(f"0x{a:08x}" for a in addrs)
        out_path = out_dir / f"{name}.log"
        out_path.write_text("\n".join(lines) + ("\n" if lines else ""))
        print(f"wrote {out_path} ({len(tasks[name])} tasks)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", type=pathlib.Path, help="input intra_AGU_conflict_rates.csv")
    ap.add_argument("out_dir", type=pathlib.Path, help="output tb/stimuli/<case> directory")
    args = ap.parse_args()
    convert(args.csv, args.out_dir)


if __name__ == "__main__":
    main()
