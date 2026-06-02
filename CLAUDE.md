# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository. See [README.md](README.md) for the shared EDA flow (commands, make parameters, pipeline). For a project's designs, top-levels, RTL parameters, and module reference, see that project's own README at `projects/<name>/README.md`.

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

This derives `RTL_LAB_HOME` (the repo root) from the script's own location, then sources your `~/.bashrc` — where you export the tool install roots: `EDA_HOME` (puts Verilator, Yosys, Yosys-Slang, OpenSTA, and SystemC on `PATH`) and `PDK_HOME`. It then derives `SYSTEMC_INCLUDE`/`SYSTEMC_LIBDIR` (used only by `make sim-sc`) and `ASAP7_HOME` (from `PDK_HOME`). Paths inside the flow are resolved as `$RTL_LAB_HOME/projects/$PROJECT/...`.
