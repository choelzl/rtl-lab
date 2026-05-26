# Prompt: Add a New PE Architecture

Use this prompt as a starting point when adding a new top-level PE to the project.
Replace `<TOP_LEVEL>` with the actual module name (e.g. `top_sqr_8x8`) and
`<LABEL>` with the human-readable label used in scripts and charts (e.g. `Square 8x8`).

---

## Prompt

I added a new top-level PE `<TOP_LEVEL>` and modified some existing RTL files.
Please carry out the following steps in order.

**1. Review added and modified RTL files**

Read every new or modified file under `projects/ai-core/rtl/` that is related to `<TOP_LEVEL>`.
For each file, check:
- Correct port connections (no missing commas, no mismatched widths).
- Localparameters and functions are consistent with the design intent.
- Header documentation matches the actual module behaviour (function, pipeline, parameters).

**2. Execute all `CLAUDE:` requests in the code**

Search all files under `projects/ai-core/rtl/` for comments starting with `CLAUDE:`.
Execute exactly what each comment requests, then remove the comment.
Typical requests include creating new modules, renaming existing ones, updating
documentation, or fixing port maps.
After acting on a request, verify that all references to renamed/created modules
are updated across the entire `projects/ai-core/rtl/` tree.

**3. Write the testbench**

Create `projects/ai-core/tb/tb_<TOP_LEVEL>.sv` following the conventions of the existing testbenches
(see `projects/ai-core/tb/tb_top_bas_8x8.sv` and `projects/ai-core/tb/tb_top_sqr_4x8_sc.sv` as primary references).
The testbench must:
- Implement a `run_and_check` task with a software reference model that matches
  the DUT's documented formula exactly.
- Run 1000 random tests followed by 5 corner cases (max-positive, min-negative,
  mixed-sign, zero).
- Support post-synthesis simulation via `` `ifdef POST_SYNTH `` with flat port maps.
- Call `$dumpfile` / `$dumpvars` and produce `activity.vcd` for dynamic power analysis.
- Call `$dumpoff` then `$fatal` on any mismatch.

**4. Run the complete flow**

Run each step sequentially; use CLK_PERIOD_NS=1.35 throughout.
Steps 3 and 4 below can be run in parallel (both depend only on synthesis output).

```bash
source sourceme.sh

make sim          TOP_LEVEL=<TOP_LEVEL> CLK_PERIOD_NS=1.35 OUT_DIR=<TOP_LEVEL>_sim          PARAMS="IS_PIPELINED=1"
make syn          TOP_LEVEL=<TOP_LEVEL>                    OUT_DIR=<TOP_LEVEL>_syn          PARAMS="IS_PIPELINED=1"
make post-syn-sim TOP_LEVEL=<TOP_LEVEL> CLK_PERIOD_NS=1.35 OUT_DIR=<TOP_LEVEL>_post_syn_sim NETLIST_DIR=<TOP_LEVEL>_syn PARAMS="IS_PIPELINED=1"
make post-syn-sta TOP_LEVEL=<TOP_LEVEL> CLK_PERIOD_NS=1.35 OUT_DIR=<TOP_LEVEL>_post_syn_sta NETLIST_DIR=<TOP_LEVEL>_syn
make post-syn-dpa TOP_LEVEL=<TOP_LEVEL> CLK_PERIOD_NS=1.35 OUT_DIR=<TOP_LEVEL>_post_syn_dpa NETLIST_DIR=<TOP_LEVEL>_syn VCD_DIR=<TOP_LEVEL>_post_syn_sim
```

All five steps must complete without errors before proceeding.
WIDTHEXPAND or UNDRIVEN warnings from the ASAP7 cell library are expected and benign.
Any other Verilator warning or a simulation failure must be investigated and fixed.

**5. Update the flow scripts**

In `projects/ai-core/scripts/flow/regres/run.py`, add a new entry for `<TOP_LEVEL>` to the
`ARCHES` list following the same format as the existing entries, placed in a logical
position relative to the other architectures.

In `projects/ai-core/scripts/flow/regres/ext.py`, add `("<LABEL>", "<SLUG>")` to the
`DESIGNS` list in the same relative position. Designs with no synthesis results are
skipped automatically.

**6. Extract results and regenerate charts**

Do NOT run `regres/run.py` (all other designs already have synthesis data).
Run only:

```bash
python3 projects/ai-core/scripts/flow/regres/ext.py
python3 projects/ai-core/scripts/flow/regres/gen.py
```

Confirm that `<LABEL>` appears in `projects/ai-core/doc/data/regres/results.xlsx` and that
the PE-level charts in `projects/ai-core/doc/charts/regres/` are regenerated successfully.

**7. Update documentation**

In `CLAUDE.md`:
- Add `<TOP_LEVEL>` to the PE Variants table with its algorithm, accumulator count,
  and array description.
- Update the Key RTL Modules list if any new source files were created.

In `README.md`:
- Add a row to the "Available TOP_LEVEL" table with module name, testbench name,
  and verified formula.
- Add a PE architecture section under "PE architectures" describing the formula,
  key design choices, and compression tree structure.
- Update the Squaring units and Partial product generators tables if new modules
  were added.
