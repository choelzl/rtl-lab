# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository. See [README.md](README.md) for the shared EDA flow (commands, make parameters, pipeline). For a project's designs, top-levels, RTL parameters, and module reference, see that project's own README at `projects/<name>/README.md`.

# Base Rules 

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

# Project Rules

## Layout

This is a multi-project RTL sandbox.

- `scripts/` — project-agnostic EDA flow wrappers (`sim`, `sim-sc`, `syn`, `post-syn-{sta,sim,dpa}`).
- `projects/<name>/` — one RTL project; contains `rtl/`, `tb/`, `sim/`, `imp/`, `doc/`, and `scripts/flow/` (project-specific automation).
- Select a project with `make <target> PROJECT=<name>`; `PROJECT` is required (no default) and so is `TOP_LEVEL`. The available projects are listed in [README.md](README.md).

## Simulation modes

There are two independent simulation flows:

- `make sim` — the default SV flow. Verilator `--binary --timing`, with a self-contained SV testbench (`tb/tb_<top>.sv`) as the top module. This is what `syn` and the post-syn flows build on.
- `make sim-sc` — the SystemC flow. Verilator `--sc --exe`, with C++ `sc_main` as the top (`tb/systemc/tb_<top>.cpp`), able to host both Verilated SV modules and native SystemC modules together under the SystemC kernel. Simulation only — SystemC is never synthesized. Requires `$SYSTEMC_HOME`. Native SystemC design modules (the non-testbench half — e.g. accumulators, design tops) live under `rtl/systemc/`; the `sc_main` harness stays under `tb/systemc/`. Both directories are on the harness include path.

**Naming:** the `_sc` suffix on RTL/top-levels (e.g. `top_bas_4x8_sc`) is a *split-cell design variant*, **not** SystemC. Keep the SystemC harness under `tb/systemc/`, native SystemC design modules under `rtl/systemc/`, and never use an `_sc` filename suffix for any of them.

**SystemC template-parameter naming (any project):** `PARAMS`/`TB_DEFS` reach the harness as ALL-CAPS `-D` config macros (e.g. `N_BANK`, `WORD_BYTES`) — these are the user-facing knobs. SystemC module template parameters are ALL-CAPS too, but named **distinctly from every config macro** (counts use a `NUM_` prefix, e.g. `NUM_BANK`; other dimensions are spelled out, e.g. `BYTES_PER_WORD`). The harness passes the macros (or values derived from them) as positional template arguments. Because no template-parameter token ever equals a `-D` macro, a `PARAMS=...` override can never textually rewrite a template declaration. Do **not** name a template parameter the same as its knob macro.

## Partial / sparse clones

The repo supports partial clone + sparse-checkout (see the README "Cloning" section), so **a working copy may contain only a subset of `projects/*`** — sometimes none. Do not assume every project is present: never glob `projects/*` expecting it to be exhaustive, and check the project directory exists before operating on it. The `Projects:` list in the root README is the authoritative catalog of what exists. To work on a project that is not checked out, materialize it with `git sparse-checkout add projects/<name>`.

## Environment

Always source the environment script before running any command:

```bash
source sourceme.sh
```

This derives `RTL_LAB_HOME` (the repo root) from the script's own location, then sources your `~/.bashrc` — where you export the tool install roots: `EDA_HOME` (puts Verilator, Yosys, Yosys-Slang, OpenSTA, and SystemC on `PATH`) and `PDK_HOME`. It then derives `SYSTEMC_INCLUDE`/`SYSTEMC_LIB` (used only by `make sim-sc`) and `ASAP7_HOME` (from `PDK_HOME`). Paths inside the flow are resolved as `$RTL_LAB_HOME/projects/$PROJECT/...`.
