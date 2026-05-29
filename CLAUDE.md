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

## Partial / sparse clones

The repo supports partial clone + sparse-checkout (see the README "Cloning" section), so **a working copy may contain only a subset of `projects/*`** — sometimes none. Do not assume every project is present: never glob `projects/*` expecting it to be exhaustive, and check the project directory exists before operating on it. The `Projects:` list in the root README is the authoritative catalog of what exists. To work on a project that is not checked out, materialize it with `git sparse-checkout add projects/<name>`.

## Environment

Always source the environment script before running any command:

```bash
source sourceme.sh
```

This sets `CODE_HOME` (the parent of this repo) and the paths for Verilator, Yosys, Yosys-Slang, OpenSTA, OpenROAD, and SystemC (`SYSTEMC_HOME`/`SYSTEMC_INCLUDE`/`SYSTEMC_LIBDIR`, used only by `make sim-sc`). Paths inside the flow are resolved as `$CODE_HOME/rtl-lab/projects/$PROJECT/...`.
