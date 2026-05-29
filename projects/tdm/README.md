# Tdm

<One-line description of the project.>

This project plugs into the repository-level EDA flow. See the [root README](../../README.md) for the `make` targets, their generic parameters, and the typical pipeline. This document covers the parts specific to `tdm`.

## Quick start

```bash
source ../../sourceme.sh   # or: source sourceme.sh from the repository root

make sim PROJECT=tdm TOP_LEVEL=<top_level> CLK_PERIOD_NS=1.0 OUT_DIR=<out_dir>
```

`tdm` is selected with `PROJECT=tdm` on every `make` command.

## Top-level modules

_None yet. Add RTL to `rtl/` and a testbench `tb/tb_<top_level>.sv` per top-level._

## RTL elaboration parameters

_None yet._

## Testbenches

_None yet._

## Experiments (automation scripts)

_None yet. Add experiment subfolders under `scripts/flow/`._
