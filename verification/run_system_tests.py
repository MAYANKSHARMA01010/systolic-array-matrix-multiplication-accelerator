"""
Systolic Array Top-Level System Regression & Performance Benchmark Runner

Script: run_system_tests.py
Description:
  Automates end-to-end integration verification, co-simulation, and performance
  evaluation of the systolic array accelerator against the sequential baseline.

Execution Stages:
  1. Golden Model Self-Test:
     Executes 'test_golden_model.py' unit tests to verify software reference correctness.
  2. Multi-Configuration Architectural Sweep:
     Iterates through 9 parameter configurations across matrix sizes (1, 2, 3, 4, 8)
     and bit-widths (8-bit/32-bit, 16-bit/48-bit, 8-bit/16-bit, etc.).
  3. Golden Vector Generation:
     For each configuration, synthesizes directed and random test matrices using
     golden_model.py and generates a.hex, b.hex, c.hex for Verilog $readmemh loading.
  4. Dual-DUT Hardware Simulation:
     Compiles all RTL and testbench files with Icarus Verilog and executes with vvp.
     Cross-verifies systolic array output vs sequential baseline vs golden vectors.
  5. Performance Metrics Extraction:
     Calculates speedups (latency and throughput), MAC operations per second,
     and initiation intervals, saving an aggregated CSV report to 'sim/results/system_summary.csv'.
"""

from __future__ import annotations

import argparse
import csv
import shutil
import subprocess
import sys
from pathlib import Path

# -----------------------------------------------------------------------------
# Directory Paths Setup
# -----------------------------------------------------------------------------
ROOT = Path(__file__).resolve().parents[1]        # Root repository directory
VERIFICATION = ROOT / "verification"              # Python verification scripts directory
RESULTS = ROOT / "sim" / "results"                # Simulation outputs and test vector dumps
WAVES = ROOT / "sim" / "waveforms"                # GTKWave VCD dump files
RTL = ROOT / "rtl"                                # Synthesizable Verilog RTL source files
TESTBENCH = ROOT / "testbench"                    # Verilog testbench directory

# Add verification directory to Python module search path
sys.path.insert(0, str(VERIFICATION))
import golden_model as gm

# -----------------------------------------------------------------------------
# Parameter Configurations to Sweep: (ARRAY_SIZE, DATA_WIDTH, ACC_WIDTH)
# -----------------------------------------------------------------------------
CONFIGS = (
    (1, 8, 32),   # Boundary 1x1 scalar baseline
    (2, 8, 32),   # 2x2 small matrix
    (3, 8, 32),   # 3x3 odd-dimension matrix
    (4, 8, 32),   # 4x4 standard project demonstration configuration
    (8, 8, 32),   # 8x8 larger scaled mesh
    (2, 16, 32),  # 16-bit operand with 32-bit accumulator (minimal headroom)
    (4, 16, 48),  # 16-bit high-precision operand with 48-bit accumulator
    (4, 8, 16),   # 8-bit operand with minimal 16-bit accumulator (overflow wrapping test)
    (8, 16, 48),  # 8x8 high-precision large scale mesh
)

# List of all synthesizable RTL Verilog files required for system compilation
RTL_SOURCES = (
    RTL / "processing_element.v",
    RTL / "systolic_array.v",
    RTL / "matrix_controller.v",
    RTL / "systolic_array_top.v",
    RTL / "sequential_matmul_baseline.v",
)
TB_SOURCE = TESTBENCH / "tb_systolic_array.v"     # System integration testbench


def run_command(command, log: Path, timeout: int, cwd: Path = ROOT) -> str:
    """
    Helper function to run a command line subprocess, capture stdout/stderr,
    write output to a log file, and raise an informative error if it fails.
    """
    completed = subprocess.run(
        command,
        cwd=cwd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        check=False,
    )
    log.write_text(completed.stdout)
    if completed.returncode:
        raise RuntimeError(
            f"Command failed ({completed.returncode}); see {log.relative_to(ROOT)}\n"
            f"{completed.stdout[-4000:]}"
        )
    return completed.stdout


def golden_unit_tests(log: Path) -> None:
    """Runs Python unit tests in test_golden_model.py before launching HDL simulations."""
    output = run_command(
        [sys.executable, "-m", "unittest", "test_golden_model", "-v"],
        log,
        60,
        cwd=VERIFICATION,
    )
    if "OK" not in output:
        raise RuntimeError("Golden-model unit tests did not report OK")


def build_vectors(size: int, data_width: int, acc_width: int, random_cases: int,
                  seed: int, directory: Path) -> list[gm.Vector]:
    """
    Generates test vectors using golden_model.py, performs self-check,
    and writes out a.hex, b.hex, c.hex in the specified directory.
    """
    vectors = gm.generate_vectors(size, data_width, acc_width, random_cases, seed)
    for name, A, B, C in vectors:
        recomputed = gm.matmul(A, B, data_width, acc_width)
        if recomputed != C:
            raise RuntimeError(f"Golden model self-check failed for vector {name}")
    gm.write_rtl_vectors(
        directory,
        [A for _, A, _, _ in vectors],
        [B for _, _, B, _ in vectors],
        [C for _, _, _, C in vectors],
        data_width,
        acc_width,
    )
    return vectors


def validate_report(row: dict[str, str], size: int, data_width: int, acc_width: int,
                    golden_cases: int, random_cases: int) -> dict[str, float]:
    """
    Validates that simulation measurements in the CSV report match exact theoretical formulas:
    - Systolic Latency   == 3*N - 1 cycles
    - Sequential Latency == N^3 cycles
    - Systolic II        == 3*N cycles
    - Sequential II      == N^3 + 1 cycles
    """
    expected = {
        "array_size": size,
        "data_width": data_width,
        "acc_width": acc_width,
        "systolic_latency_cycles": 3 * size - 1,
        "sequential_latency_cycles": size ** 3,
        "systolic_ii_cycles": 3 * size,
        "sequential_ii_cycles": size ** 3 + 1,
        "total_cases": 8 + golden_cases + random_cases + 3,
    }
    for field, value in expected.items():
        if int(row[field]) != value:
            raise RuntimeError(f"{field}: expected {value}, measured {row[field]}")
    if row["status"] != "PASS":
        raise RuntimeError("testbench reported FAIL")

    systolic_latency   = int(row["systolic_latency_cycles"])
    sequential_latency = int(row["sequential_latency_cycles"])
    systolic_ii        = int(row["systolic_ii_cycles"])
    sequential_ii      = int(row["sequential_ii_cycles"])
    return {
        "latency_speedup": sequential_latency / systolic_latency,
        "throughput_speedup": sequential_ii / systolic_ii,
    }


def main() -> int:
    """Main CLI entry point for full system regression."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--golden-random", type=int, default=12,
                        help="Number of random vectors generated by golden_model (plus directed cases)")
    parser.add_argument("--random-cases", type=int, default=4,
                        help="Number of extra random cases generated internally inside the Verilog testbench")
    parser.add_argument("--seed", type=int, default=42, help="Golden-model random seed")
    parser.add_argument("--clock-mhz", type=float, default=100.0,
                        help="Assumed clock frequency in MHz for throughput/latency reporting (default: 100 MHz)")
    parser.add_argument("--waveforms", action="store_true",
                        help="Save GTKWave VCD dump for each configuration")
    parser.add_argument("--skip-unit-tests", action="store_true",
                        help="Skip running Python golden-model unit tests")
    args = parser.parse_args()

    if args.golden_random < 0 or args.random_cases < 0:
        parser.error("case counts must be nonnegative")
    if args.clock_mhz <= 0:
        parser.error("--clock-mhz must be positive")

    # Check simulator tools availability
    missing = [name for name in ("iverilog", "vvp") if shutil.which(name) is None]
    if missing:
        print("Cannot run HDL simulations: missing " + ", ".join(missing) + " on PATH.",
              file=sys.stderr)
        return 2

    # Verify all source files exist
    for source in (*RTL_SOURCES, TB_SOURCE):
        if not source.exists():
            print(f"Missing source file: {source}", file=sys.stderr)
            return 2

    RESULTS.mkdir(parents=True, exist_ok=True)
    WAVES.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------------
    # Step 1: Run Python Golden Model Unit Tests
    # ------------------------------------------------------------------------
    if not args.skip_unit_tests:
        golden_unit_tests(RESULTS / "golden_unit_tests.log")
        print("PASS golden model unit tests")

    summary_path = RESULTS / "system_summary.csv"
    summary_path.unlink(missing_ok=True)
    summary_rows: list[dict[str, object]] = []

    # ------------------------------------------------------------------------
    # Step 2: Iterate Over Architectural Configurations
    # ------------------------------------------------------------------------
    for size, data_width, acc_width in CONFIGS:
        name = f"system_{size}x{size}_d{data_width}_a{acc_width}"
        exe = RESULTS / f"{name}.vvp"
        report = RESULTS / f"{name}.csv"
        vector_dir = RESULTS / "vectors" / name
        report.unlink(missing_ok=True)

        # Build test vectors in hex format
        vectors = build_vectors(size, data_width, acc_width, args.golden_random,
                                args.seed, vector_dir)
        golden_cases = len(vectors)

        # Compile with Icarus Verilog
        compile_command = [
            "iverilog", "-g2012", "-Wall", "-s", "tb_systolic_array",
            "-P", f"tb_systolic_array.ARRAY_SIZE={size}",
            "-P", f"tb_systolic_array.DATA_WIDTH={data_width}",
            "-P", f"tb_systolic_array.ACC_WIDTH={acc_width}",
            "-P", f"tb_systolic_array.RANDOM_CASES={args.random_cases}",
            "-P", f"tb_systolic_array.GOLDEN_CASES={golden_cases}",
            "-o", str(exe),
            *[str(source) for source in RTL_SOURCES],
            str(TB_SOURCE),
        ]
        run_command(compile_command, RESULTS / f"{name}_compile.log", 120)

        # Simulate with vvp
        simulation_command = [
            "vvp", str(exe),
            f"+VECTORS_DIR={vector_dir}",
            f"+REPORT={report}",
        ]
        if args.waveforms:
            simulation_command.append(f"+VCD={WAVES / (name + '.vcd')}")
        output = run_command(simulation_command, RESULTS / f"{name}.log", 300)
        if "ALL SYSTEM INTEGRATION TESTS PASSED" not in output:
            raise RuntimeError(f"{name}: testbench did not report success")

        # Read and validate report
        with report.open(newline="") as handle:
            records = list(csv.DictReader(handle))
        if len(records) != 1:
            raise RuntimeError(f"{name}: expected exactly one report row")
        row = records[0]
        speedups = validate_report(row, size, data_width, acc_width,
                                   golden_cases, args.random_cases)

        # Compute physical timing and throughput metrics based on clock frequency
        period_ns = 1000.0 / args.clock_mhz
        systolic_latency   = int(row["systolic_latency_cycles"])
        sequential_latency = int(row["sequential_latency_cycles"])
        systolic_ii        = int(row["systolic_ii_cycles"])
        sequential_ii      = int(row["sequential_ii_cycles"])
        clock_hz           = args.clock_mhz * 1e6
        systolic_matrices_per_second   = clock_hz / systolic_ii
        sequential_matrices_per_second = clock_hz / sequential_ii
        macs_per_matrix    = size ** 3

        summary_rows.append({
            "array_size": size,
            "data_width": data_width,
            "acc_width": acc_width,
            "golden_cases": golden_cases,
            "random_cases": args.random_cases,
            "total_cases": int(row["total_cases"]),
            "clock_mhz": args.clock_mhz,
            "systolic_latency_cycles": systolic_latency,
            "sequential_latency_cycles": sequential_latency,
            "systolic_ii_cycles": systolic_ii,
            "sequential_ii_cycles": sequential_ii,
            "systolic_latency_ns": systolic_latency * period_ns,
            "sequential_latency_ns": sequential_latency * period_ns,
            "systolic_matrices_per_second": systolic_matrices_per_second,
            "sequential_matrices_per_second": sequential_matrices_per_second,
            "systolic_macs_per_second": systolic_matrices_per_second * macs_per_matrix,
            "sequential_macs_per_second": sequential_matrices_per_second * macs_per_matrix,
            "latency_speedup": speedups["latency_speedup"],
            "throughput_speedup": speedups["throughput_speedup"],
            "status": "PASS",
        })
        print(
            f"PASS {name}: {golden_cases} golden vectors, {int(row['total_cases'])} cases; "
            f"latency {systolic_latency} vs {sequential_latency} cycles "
            f"({speedups['latency_speedup']:.2f}x), "
            f"throughput {systolic_ii} vs {sequential_ii} cycles "
            f"({speedups['throughput_speedup']:.2f}x)"
        )

    # ------------------------------------------------------------------------
    # Step 3: Write Full Summary CSV
    # ------------------------------------------------------------------------
    with summary_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(summary_rows[0]))
        writer.writeheader()
        writer.writerows(summary_rows)

    # Print summary table to terminal
    print()
    print(f"{'N':>2} {'DW':>3} {'ACC':>4} {'lat(cyc)':>9} {'II(cyc)':>8} "
          f"{'lat(ns)':>9} {'MMAC/s':>9} {'lat x':>7} {'thr x':>7}")
    for row in summary_rows:
        print(
            f"{row['array_size']:>2} {row['data_width']:>3} {row['acc_width']:>4} "
            f"{row['systolic_latency_cycles']:>9} {row['systolic_ii_cycles']:>8} "
            f"{row['systolic_latency_ns']:>9.1f} "
            f"{row['systolic_macs_per_second'] / 1e6:>9.2f} "
            f"{row['latency_speedup']:>7.2f} {row['throughput_speedup']:>7.2f}"
        )
    print(f"\nAll {len(summary_rows)} configurations passed. Summary: "
          f"{summary_path.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (RuntimeError, subprocess.TimeoutExpired, OSError, ValueError) as exc:
        print(f"System regression failed: {exc}", file=sys.stderr)
        sys.exit(1)
