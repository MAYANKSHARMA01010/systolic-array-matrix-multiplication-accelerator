"""
Systolic Array Core Automation Regression Runner (run_array_tests.py)

Description:
  Automates compilation, simulation, and report aggregation for the bare systolic
  array core across 10 distinct architectural parameter configurations (sweeping
  matrix dimension N, operand bit-width, and accumulator bit-width).

Configurations Tested:
  - Sizes: N = 1, 2, 3, 4, 8
  - Data widths: 8-bit, 16-bit
  - Accumulator widths: 16-bit, 32-bit, 48-bit

Workflow per Configuration:
  1. Compile: Invokes 'iverilog' (Icarus Verilog) with -g2012 SystemVerilog support
     and parameter overrides (-P).
  2. Simulate: Executes compiled binary with 'vvp' and records execution logs.
  3. Validate: Reads generated CSV report, verifies status == PASS, and confirms
     latency matches theoretical formulas:
         core_edges  == 3*N - 2
         drain_edges == 2*(N - 1)
  4. Aggregate: Consolidates results into 'sim/results/array_summary.csv'.
"""

import argparse
import csv
import shutil
import subprocess
import sys
from pathlib import Path

# Project directory paths
ROOT = Path(__file__).resolve().parents[1]        # Root repository directory
RESULTS = ROOT / 'sim' / 'results'                # Directory for logs, compiled vvp, and reports
WAVES = ROOT / 'sim' / 'waveforms'                # Directory for GTKWave VCD dumps

# Matrix parameter sweep configurations: (ARRAY_SIZE, DATA_WIDTH, ACC_WIDTH)
CONFIGS = (
    (2, 8, 32), (4, 8, 32), (8, 8, 32),      # Standard 8-bit configs with 32-bit accumulator
    (2, 16, 48), (4, 16, 48), (8, 16, 48),   # High-precision 16-bit configs with 48-bit accumulator
    (4, 8, 16), (4, 16, 32),                 # Minimal accumulator headroom configs (ACC_WIDTH = 2*DATA_WIDTH)
    (1, 8, 32), (3, 8, 32),                  # Degenerate 1x1 and odd 3x3 array configurations
)


def run_command(command, log, timeout):
    """
    Executes a shell command via subprocess, redirects stdout/stderr to a log file,
    and raises RuntimeError if the command exits with a non-zero return code.
    """
    completed = subprocess.run(command, cwd=ROOT, text=True, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, timeout=timeout, check=False)
    log.write_text(completed.stdout)
    if completed.returncode:
        raise RuntimeError(f'Command failed ({completed.returncode}); see {log.relative_to(ROOT)}\n'
                           f'{completed.stdout[-4000:]}')
    return completed.stdout


def main():
    """Main CLI entry point for array regression."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--waveforms', action='store_true', help='Save per-configuration VCD files')
    parser.add_argument('--random-cases', type=int, default=100,
                        help='Number of random matrix test cases to execute per configuration')
    args = parser.parse_args()

    if args.random_cases < 0:
        parser.error('--random-cases must be nonnegative')

    # Ensure required EDA simulator tools (iverilog and vvp) are installed on PATH
    missing = [name for name in ('iverilog', 'vvp') if shutil.which(name) is None]
    if missing:
        print('Cannot run HDL simulations: missing ' + ', '.join(missing) + ' on PATH.', file=sys.stderr)
        return 2

    RESULTS.mkdir(parents=True, exist_ok=True)
    WAVES.mkdir(parents=True, exist_ok=True)

    summary = RESULTS / 'array_summary.csv'
    summary.unlink(missing_ok=True)  # Remove stale summary if present
    rows = []

    # Iterate through all architectural configurations
    for size, data_width, acc_width in CONFIGS:
        name = f'array_{size}x{size}_d{data_width}_a{acc_width}'
        executable = RESULTS / f'{name}.vvp'
        report = RESULTS / f'{name}.csv'
        report.unlink(missing_ok=True)

        # --------------------------------------------------------------------
        # Step 1: Compile Verilog Source with Icarus Verilog
        # --------------------------------------------------------------------
        command = [
            'iverilog', '-g2012', '-Wall', '-s', 'tb_systolic_array_core',
            '-P', f'tb_systolic_array_core.ARRAY_SIZE={size}',
            '-P', f'tb_systolic_array_core.DATA_WIDTH={data_width}',
            '-P', f'tb_systolic_array_core.ACC_WIDTH={acc_width}',
            '-P', f'tb_systolic_array_core.RANDOM_CASES={args.random_cases}',
            '-o', str(executable),
            'rtl/processing_element.v',
            'rtl/systolic_array.v',
            'testbench/tb_systolic_array_core.v'
        ]
        run_command(command, RESULTS / f'{name}_compile.log', 60)

        # --------------------------------------------------------------------
        # Step 2: Run Simulation with vvp Runtime Engine
        # --------------------------------------------------------------------
        command = ['vvp', str(executable), f'+REPORT={report}']
        if args.waveforms:
            command.append(f'+VCD={WAVES / (name + ".vcd")}')
        output = run_command(command, RESULTS / f'{name}.log', 120)

        # Confirm testbench finished with pass message
        if 'ALL ARRAY CORE TESTS PASSED' not in output:
            raise RuntimeError(f'{name}: simulation did not report success')

        # --------------------------------------------------------------------
        # Step 3: Validate Generated Simulation Report
        # --------------------------------------------------------------------
        with report.open(newline='') as handle:
            records = list(csv.DictReader(handle))
        if len(records) != 1 or records[0]['status'] != 'PASS':
            raise RuntimeError(f'{name}: invalid simulation report')

        row = records[0]
        # Strictly verify theoretical latency equations:
        # core_edges == 3*N - 2, drain_edges == 2*(N - 1)
        if int(row['core_edges']) != 3*size-2 or int(row['drain_edges']) != 2*(size-1):
            raise RuntimeError(f'{name}: measured latency does not match schedule')

        rows.append(row)
        print(f'PASS {name}: {row["matrices"]} matrices, {row["checked_edges"]} checked edges; '
              f'core={row["core_edges"]}, drain={row["drain_edges"]}')

    # ------------------------------------------------------------------------
    # Step 4: Write Aggregated CSV Summary
    # ------------------------------------------------------------------------
    with summary.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    print(f'All {len(rows)} configurations passed. Measured results: {summary.relative_to(ROOT)}')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, subprocess.TimeoutExpired, OSError) as exc:
        print(f'Array regression failed: {exc}', file=sys.stderr)
        sys.exit(1)
