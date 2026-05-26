---
name: add-arch
description: Integrate a new top-level architecture into an existing RTL project — review its RTL, write the testbench, run the full EDA flow, wire it into the project's automation, and update docs. Use when the user has added a new top-level module (and possibly modified RTL) and wants it verified, characterized, and documented within a project.
---

# Add a new architecture to a project

Integrate a newly added top-level module into an existing project under `projects/<project>/`. Carry out the steps below in order.

## 0. Resolve the arguments

From the skill invocation, determine:

- `<project>` — the target project under `projects/`. Default to `ai-core` if not given. Abort if `projects/<project>/` does not exist.
- `<top_level>` — the new top-level module name (e.g. `top_sqr_8x8`). Ask the user if not given.
- `<label>` — the human-readable label used in scripts/charts/docs (e.g. `Square 8x8`). Ask if not given and the project has automation that needs it.

Always source the environment first: `source sourceme.sh`.

All paths below are under `projects/<project>/`.

## 1. Review added and modified RTL

Read every new or modified file under `projects/<project>/rtl/` related to `<top_level>`. For each, check:

- Correct port connections (no missing commas, no mismatched widths).
- Localparameters and functions are consistent with the design intent.
- Header documentation matches the actual module behaviour (function, pipeline, parameters).

## 2. Execute `CLAUDE:` requests in the code

Search all files under `projects/<project>/rtl/` for comments starting with `CLAUDE:`. Execute exactly what each comment requests, then remove the comment. Typical requests: creating new modules, renaming existing ones, updating documentation, or fixing port maps. After acting on a request, verify that all references to renamed/created modules are updated across the whole `projects/<project>/rtl/` tree.

## 3. Write the testbench

Create `projects/<project>/tb/tb_<top_level>.sv` following the conventions of the project's existing testbenches (read two of the closest existing `tb/tb_*.sv` files as references before writing). The testbench must:

- Implement a reference-model check task whose software model matches the DUT's documented formula exactly.
- Run 1000 random tests followed by corner cases (max-positive, min-negative, mixed-sign, zero).
- Support post-synthesis simulation via `` `ifdef POST_SYNTH `` with flat port maps.
- Call `$dumpfile` / `$dumpvars` and produce `activity.vcd` for dynamic power analysis; call `$dumpoff` then `$fatal` on any mismatch.

If the project has no existing testbenches, follow the same pattern but infer the DUT interface from the top-level module's ports.

## 4. Run the complete flow

Determine the standard `CLK_PERIOD_NS` and `PARAMS` for this project by reading an existing entry in its automation scripts (see step 5) or an existing documented example. If none exist, default to `CLK_PERIOD_NS=1.35` and `PARAMS="IS_PIPELINED=1"`, and adjust `PARAMS` to the elaboration parameters this top-level actually accepts.

Run each step sequentially (post-syn-sta and post-syn-dpa depend only on synthesis / sim output):

```bash
source sourceme.sh

make sim          PROJECT=<project> TOP_LEVEL=<top_level> CLK_PERIOD_NS=<clk> OUT_DIR=<top_level>_sim          PARAMS="<params>"
make syn          PROJECT=<project> TOP_LEVEL=<top_level>                     OUT_DIR=<top_level>_syn          PARAMS="<params>"
make post-syn-sim PROJECT=<project> TOP_LEVEL=<top_level> CLK_PERIOD_NS=<clk> OUT_DIR=<top_level>_post_syn_sim NETLIST_DIR=<top_level>_syn PARAMS="<params>"
make post-syn-sta PROJECT=<project> TOP_LEVEL=<top_level> CLK_PERIOD_NS=<clk> OUT_DIR=<top_level>_post_syn_sta NETLIST_DIR=<top_level>_syn
make post-syn-dpa PROJECT=<project> TOP_LEVEL=<top_level> CLK_PERIOD_NS=<clk> OUT_DIR=<top_level>_post_syn_dpa NETLIST_DIR=<top_level>_syn VCD_DIR=<top_level>_post_syn_sim
```

All five steps must complete without errors before proceeding. WIDTHEXPAND or UNDRIVEN warnings from the ASAP7 cell library are expected and benign. Any other Verilator warning or a simulation failure must be investigated and fixed.

## 5. Update the project's automation (adaptive)

Inspect `projects/<project>/scripts/flow/`. Only act on what exists:

- If `regres/run.py` defines an `ARCHES` list, add an entry for `<top_level>` following the exact format and column layout of the existing entries, placed in a logical position relative to the others.
- If `regres/ext.py` defines a `DESIGNS` list, add `("<label>", "<slug>")` in the same relative position. Designs with no synthesis results are skipped automatically.
- If the project has different or no automation scripts, adapt to their actual structure, or skip this step and note that there was nothing to wire into.

## 6. Extract results and regenerate charts (if automation exists)

Only if step 5 found `ext.py` / `gen.py`. Do NOT run `regres/run.py` (other designs already have synthesis data). Run only:

```bash
python3 projects/<project>/scripts/flow/regres/ext.py
python3 projects/<project>/scripts/flow/regres/gen.py
```

Confirm `<label>` appears in `projects/<project>/doc/data/regres/results.xlsx` and that the charts in `projects/<project>/doc/charts/regres/` regenerate successfully.

## 7. Update documentation

In `projects/<project>/README.md`:

- Add a row to the "Top-level modules" table with module name, testbench name, and verified formula.
- Add an architecture section describing the formula, key design choices, and compression-tree (or datapath) structure.
- Update the module-reference tables if new RTL modules were created.

In the root `CLAUDE.md`: update only if a repo-level pointer genuinely needs it (usually not necessary — keep it minimal).

Do not commit anything unless the user asks.
