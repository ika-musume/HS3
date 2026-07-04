# Out-of-Context FPGA Evaluation

This directory contains reusable command-line helpers for module-level FPGA
timing/resource experiments.

The current flow supports a Quartus registered-boundary OOC run:

```bash
python3 eval_ooc/tools/quartus_ooc.py prepare eval_ooc/inst_pipeline/config.json
python3 eval_ooc/tools/quartus_ooc.py run eval_ooc/inst_pipeline/config.json
python3 eval_ooc/tools/quartus_ooc.py sta eval_ooc/inst_pipeline/config.json
python3 eval_ooc/tools/quartus_ooc.py summarize eval_ooc/inst_pipeline/config.json
```

Or run the whole sequence:

```bash
python3 eval_ooc/tools/quartus_ooc.py all eval_ooc/inst_pipeline/config.json
```

Run without prepare:

```bash
python3 eval_ooc/tools/quartus_ooc.py all eval_ooc/inst_pipeline/config.json --no-prepare
```

Critical-path reporting defaults to the top 20 setup paths. Override it with:

```bash
python3 eval_ooc/tools/quartus_ooc.py sta eval_ooc/inst_pipeline/config.json --critical-paths 50
python3 eval_ooc/tools/quartus_ooc.py summarize eval_ooc/inst_pipeline/config.json --critical-paths 50
```

The generated run directory contains:

- `<wrapper>.sv`: auto-generated wrapper with registered DUT inputs/outputs
- `<revision>.sdc`: target clock and reset false-path constraints
- `<revision>.qsf` / `<revision>.qpf`: Quartus project files
- `run_quartus.sh`: batch compile script used inside Docker
- `run_sta.sh`: TimeQuest-only script used by the `sta` command
- `critical_paths.tcl`: TimeQuest script for detailed top-N setup paths
- `reports/critical_paths.rpt`: full-path critical timing report after `run`
  or `sta`
- `summary.json`: extracted timing/resource summary after `summarize`

The Quartus run uses the Docker image named in the config, currently
`raetro/quartus:17.0`. The repo root is mounted at `/work`, and the container
working directory is the generated run directory.

## Config Notes

- `sources` are relative to the config file unless absolute.
- `top_source` is the source file used for parsing the top module port list.
- `clock_enable.mode = "tie_high"` is the initial native 100 MHz mode. Later
  runs can add a separate clock-enable multicycle mode.
- `critical_paths` sets the default number of detailed setup paths requested
  from TimeQuest. The command-line `--critical-paths` option overrides it.
- DUT inputs are driven through wrapper registers and DUT outputs are captured
  through wrapper registers, so board IO delays are intentionally not modeled.
