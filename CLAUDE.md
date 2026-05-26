# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository. See [README.md](README.md) for full documentation: commands, PE architectures, RTL module reference, and parameter tables.

## Layout

This is a multi-project RTL sandbox.

- `scripts/` — project-agnostic EDA flow wrappers (`sim`, `syn`, `post-syn-{sta,sim,dpa}`).
- `projects/<name>/` — one RTL project; contains `rtl/`, `tb/`, `sim/`, `imp/`, `doc/`, and `scripts/flow/` (project-specific automation).
- Default project is `ai-core` ([projects/ai-core/](projects/ai-core/)). Select a different project with `make <target> PROJECT=<name>`.

## Environment

Always source the environment script before running any command:

```bash
source sourceme.sh
```

This sets `CODE_HOME` (the parent of this repo) and the paths for Verilator, Yosys, Yosys-Slang, OpenSTA, and OpenROAD. Paths inside the flow are resolved as `$CODE_HOME/rtl-lab/projects/$PROJECT/...`.
