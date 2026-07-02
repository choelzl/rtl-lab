# TDM vs. crossbar design report

`report.html` is a paper-style report describing the system, both interconnect
backends, the buffer design, the verification suites, and the measured timing.
It is fully regenerated from the snapshots in `data/` by:

```sh
python3 doc/report/gen_report.py            # → doc/report/report.html
```

The prose and figures live in `gen_report.py`; all tables (test counts, spans,
totals, parity checks) are recomputed from `data/` on every run. After a design
change, only the data snapshots need refreshing:

## Refreshing `data/`

All commands from `projects/tdm/`, with the environment sourced
(`source ../../sourceme.sh`).

### 1. Timing logs (three builds of the stim_bank suite)

Each build needs `-DSTIM_TIMING_REPORT`, which makes the suite print one
`[timing] backend,phase,ports,n_data,note,span` line per stimulus task.
Notes carry the pattern flavor: conflict/same-bank tasks without a suffix are
adversarial to the build's OWN routing; `_xbarhash`/`_tdmmap` suffixes mark
the cross-mapped flavor (same-bank under the sibling backend's routing) that
`gen_report.py` pairs across logs into the pattern-class tables:

```sh
CFLAGS='-std=c++17 -O2 -Wno-cpp -Irtl/systemc -Itb/systemc -I"$SYSTEMC_INCLUDE" -DSTIM_TIMING_REPORT'
LFLAGS='-L"$SYSTEMC_LIB" -Wl,-rpath,"$SYSTEMC_LIB" -lsystemc -pthread'

eval g++ $CFLAGS                    tb/unit/tb_stim_bank_xbar.cpp         -o /tmp/sx $LFLAGS
eval g++ $CFLAGS                    tb/unit/tb_stim_bank_tdm.cpp          -o /tmp/sr $LFLAGS
eval g++ $CFLAGS                    tb/unit/tb_stim_bank_tdm_adaptive.cpp -o /tmp/sa $LFLAGS

/tmp/sx > doc/report/data/timing_xbar.log         2>&1   # ~1 min each
/tmp/sr > doc/report/data/timing_tdm_rr.log       2>&1
/tmp/sa > doc/report/data/timing_tdm_adaptive.log 2>&1
```

(`edaf unit TOP=stim_bank_xbar ...` works too if the flow forwards the extra
defines; the manual builds above are the known-good path.)

### 2. Test sweep summary

Run the full unit sweep and write one line to `data/sweep.txt` in the form:

```
agu: 29 | arbiter: 16 | tdm: 16 | ... | top_tdm: 282
```

i.e. `suite: pass-count` pairs joined by `|`, one entry per `tb_*.cpp` suite.
The suite → focus-text mapping lives in `SUITE_FOCUS` inside `gen_report.py`;
add an entry there when a new suite is created.

### 3. Evaluation data (report §5)

The `eval_*.csv` snapshots come from the production-representative sweep
(final/0–19 through `tb/systemc/tb_top.cpp`, three builds, fenced + unfenced):

```sh
bash doc/report/tools/run_eval_sweep.sh          # ~10 min, writes data/eval_*.csv
python3 doc/report/gen_report.py
```

The stimuli under `tb/stimuli/final/{0..19}/` are proprietary and NOT in the
repository (see the `.gitignore` there) — obtain and place them first. The
sweep runs the evaluation configuration of report Appendix A.7: `SEL_LA_LEAD=16`
(hidden lookahead), `SEL_NO_MONITOR=1`, the RR build's 8-slot arbiter table,
and the crossbar built with `-DXBAR_HASH_L1` (the L1 hash repair — effect in
§5.2, diagnosis in Appendix A.8).
Harness knobs the sweep does not set — `SEL_NO_FENCE` (throughput-bound mode,
§5.3) and `SEL_PORT_INTERLEAVE` (cross-port lane dealing, A.8) — are applied
by the script per run or left at defaults; all knobs are documented in
Appendix A.7 and the top-of-file comments in `tb/systemc/agu.hpp`.

## Editing the report

- Prose: the `page = f"""..."""` template in `gen_report.py`.
- Figures: the `fig_*()` builders (plain SVG helpers) — wiring diagrams,
  buffer timelines, the results chart, and the per-set demand/pressure
  timeline (`fig_bank_timeline`, fed by `data/eval_timeline.csv`).
- Tables: computed — do not hand-edit; fix the data or the parser instead.
- The exact-span parity claim is asserted in-suite
  (`tb/unit/stim_bank_common.hpp`), so if the report says parity holds, the
  sweep in step 2 already proved it.
