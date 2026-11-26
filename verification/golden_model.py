"""
Systolic Array Matrix Multiplication - Golden Model & Verification Tool

This module provides:
1. Bit-Accurate Golden Reference Model:
   Implements software matrix multiplication (matmul) with exact two's complement
   hardware integer wrapping behavior (wrap_signed) matching Verilog RTL.
2. Bus Serialization & Formatting (pack/unpack):
   Converts between Python 2D nested lists and flat bit-vectors/hex strings
   matching the flattened hardware buses (a_flat, b_flat, result).
3. Test Vector Generation:
   Produces comprehensive test suites combining deterministic corner cases
   (identity, zeros, signed extremes, checkerboards, triangular matrices)
   and pseudo-random matrices.
4. RTL Compatibility Export:
   Exports test matrices in $readmemh hex formats (a.hex, b.hex, c.hex) for direct
   consumption by Verilog testbenches.
"""

from __future__ import annotations

import argparse
import random
import sys
from pathlib import Path
from typing import Iterable, Sequence

# -----------------------------------------------------------------------------
# Default Architectural Parameters
# -----------------------------------------------------------------------------
DEFAULT_SIZE = 4            # Default matrix dimension (4x4)
DEFAULT_DATA_WIDTH = 8      # Default input operand bit-width (8-bit signed)
DEFAULT_ACC_WIDTH = 32      # Default accumulator bit-width (32-bit signed)
DEFAULT_VECTORS_DIR = (     # Default directory where generated test vectors are stored
    Path(__file__).resolve().parents[1] / "sim" / "results" / "golden_vectors"
)

# Type Aliases for Clean Type Annotations
Matrix = list[list[int]]                            # 2D grid of integer values
Vector = tuple[str, Matrix, Matrix, Matrix]         # (test_name, Matrix A, Matrix B, Matrix C)


def wrap_signed(value: int, width: int) -> int:
    """
    Simulates hardware two's complement integer wrapping to a fixed bit-width.

    In Python, integers have arbitrary precision (unlimited bits). However, in
    hardware registers (Verilog), an N-bit register wraps around modulo 2^N.
    This function mimics exact hardware register behavior:
    1. Bitwise AND with ((1 << width) - 1) discards all bits beyond 'width'.
    2. If the sign bit (bit width - 1) is 1, converts unsigned value to negative
       signed integer by subtracting 2^width.

    Example for width=8:
      wrap_signed(127, 8)  -> +127 (0x7F)
      wrap_signed(128, 8)  -> -128 (0x80 in two's complement)
      wrap_signed(255, 8)  -> -1   (0xFF)
      wrap_signed(256, 8)  -> 0    (0x00 overflow wrap)
    """
    if width < 1:
        raise ValueError("width must be at least 1")
    mask = (1 << width) - 1              # e.g., 0xFF (255) for 8 bits
    value &= mask                        # Truncate to lower 'width' bits
    sign_bit = 1 << (width - 1)          # e.g., 0x80 (128) for 8 bits
    # If MSB (sign bit) is set, convert from unsigned representation to signed negative
    return value - (1 << width) if value & sign_bit else value


def _validate_params(size: int, data_width: int, acc_width: int) -> None:
    """
    Ensures architectural parameters satisfy hardware design constraints.
    - Matrix size must be at least 1x1.
    - Data width must be at least 1 bit.
    - Accumulator width must be at least 2*data_width (to hold at least one product).
    """
    if size < 1:
        raise ValueError("size must be at least 1")
    if data_width < 1:
        raise ValueError("data_width must be at least 1")
    if acc_width < 2 * data_width:
        raise ValueError("acc_width must be at least 2*data_width")


def _validate_square(matrix: Sequence[Sequence[int]], name: str) -> int:
    """
    Validates that the input matrix is non-empty and perfectly square (N x N).
    Returns the dimension N.
    """
    if not matrix:
        raise ValueError(f"{name} must not be empty")
    size = len(matrix)
    for row in matrix:
        if len(row) != size:
            raise ValueError(f"{name} must be square (got {size}x{len(row)})")
    return size


def matmul(
    A: Sequence[Sequence[int]],
    B: Sequence[Sequence[int]],
    data_width: int = DEFAULT_DATA_WIDTH,
    acc_width: int = DEFAULT_ACC_WIDTH,
) -> Matrix:
    """
    Bit-accurate matrix multiplication: C = A * B.

    Performs the standard 3-nested-loop matrix multiplication:
        C[r][c] = sum_{k=0}^{N-1} (A[r][k] * B[k][c])
    At each step:
    - Input operands A[r][k] and B[k][c] are truncated to 'data_width' bits.
    - Product is added to running accumulator.
    - Running accumulator is wrapped to 'acc_width' bits on every addition,
      precisely matching the hardware PE accumulator behavior.
    """
    _validate_params(len(A), data_width, acc_width)
    size = _validate_square(A, "A")
    if _validate_square(B, "B") != size:
        raise ValueError("A and B must have the same dimensions")

    # Allocate result matrix filled with 0s
    result: Matrix = [[0] * size for _ in range(size)]

    # Compute dot products for each cell (r, c)
    for r in range(size):
        for c in range(size):
            acc = 0
            for k in range(size):
                # Clamp/wrap inputs to signed data_width
                a = wrap_signed(A[r][k], data_width)
                b = wrap_signed(B[k][c], data_width)
                # Accumulate product and wrap to accumulator bit-width
                acc = wrap_signed(acc + a * b, acc_width)
            result[r][c] = acc
    return result


def pack_matrix(matrix: Sequence[Sequence[int]], data_width: int) -> int:
    """
    Flattens a 2D matrix into a single large integer representing the packed 1D
    hardware bus in row-major order.

    Layout in bit-vector:
      Linear index = (row * size) + col
      Bit offset   = index * data_width
      Bits [offset + data_width - 1 : offset] = matrix[row][col] & mask
    """
    size = _validate_square(matrix, "matrix")
    mask = (1 << data_width) - 1
    packed = 0
    for r in range(size):
        for c in range(size):
            # Place element bits at appropriate bit offset
            packed |= (matrix[r][c] & mask) << (data_width * (r * size + c))
    return packed


def unpack_matrix(packed: int, size: int, width: int) -> Matrix:
    """
    Reverses 'pack_matrix': extracts each element from a large packed integer
    bus back into a 2D Python list of signed values.
    """
    _validate_params(size, width, 2 * width)
    mask = (1 << width) - 1
    matrix: Matrix = [[0] * size for _ in range(size)]
    for index in range(size * size):
        # Extract 'width' bits at the given element index
        field = (packed >> (width * index)) & mask
        # Convert unsigned bitfield to signed integer
        matrix[index // size][index % size] = wrap_signed(field, width)
    return matrix


def matrix_to_hex(matrix: Sequence[Sequence[int]], width: int) -> str:
    """
    Packs a matrix and formats it as a zero-padded lowercase hexadecimal string.
    Useful for comparing against simulator hex dumps.
    """
    size = _validate_square(matrix, "matrix")
    digits = (size * size * width + 3) // 4  # Total hex characters needed (4 bits per hex digit)
    return f"{pack_matrix(matrix, width):0{digits}x}"


def verify_result_hex(
    result_hex: str,
    A: Sequence[Sequence[int]],
    B: Sequence[Sequence[int]],
    data_width: int = DEFAULT_DATA_WIDTH,
    acc_width: int = DEFAULT_ACC_WIDTH,
    verbose: bool = True,
) -> bool:
    """
    Verifies a raw hex string dumped by an RTL simulation against the expected
    matrix multiplication result computed by the golden model.

    Returns True if hardware output matches golden model exactly; False otherwise.
    """
    size = _validate_square(A, "A")
    if _validate_square(B, "B") != size:
        raise ValueError("A and B must have the same dimensions")
    _validate_params(size, data_width, acc_width)

    # Clean up formatting: strip whitespace, '0x' prefixes, and underscores
    cleaned = result_hex.strip().lower().replace("_", "").replace(" ", "")
    if cleaned.startswith("0x"):
        cleaned = cleaned[2:]
    if not cleaned or any(ch not in "0123456789abcdef" for ch in cleaned):
        raise ValueError("result_hex must contain only hexadecimal digits")

    value = int(cleaned, 16)
    expected = matmul(A, B, data_width=data_width, acc_width=acc_width)

    mask = (1 << acc_width) - 1
    ok = True
    for r in range(size):
        for c in range(size):
            # Extract cell [r][c] from packed hardware hex
            field = (value >> (acc_width * (r * size + c))) & mask
            got = wrap_signed(field, acc_width)
            if got != expected[r][c]:
                ok = False
                if verbose:
                    print(f"  MISMATCH C[{r}][{c}]: expected {expected[r][c]}, got {got}")
    return ok


def _format_matrix(matrix: Sequence[Sequence[int]]) -> list[str]:
    """Helper: formats each row of a matrix into space-separated string."""
    return ["  ".join(str(value) for value in row) for row in matrix]


def write_vector(
    path: str | Path,
    A: Sequence[Sequence[int]],
    B: Sequence[Sequence[int]],
    C: Sequence[Sequence[int]] | None = None,
    data_width: int = DEFAULT_DATA_WIDTH,
    acc_width: int = DEFAULT_ACC_WIDTH,
    name: str | None = None,
) -> None:
    """
    Writes a human-readable test vector file (.txt) with comments, metadata,
    and formatted matrices A, B, and expected C.
    """
    size = _validate_square(A, "A")
    if _validate_square(B, "B") != size:
        raise ValueError("A and B must have the same dimensions")
    if C is None:
        C = matmul(A, B, data_width=data_width, acc_width=acc_width)
    if _validate_square(C, "C") != size:
        raise ValueError("A, B and C must have the same dimensions")

    lines = [
        "# Parameterized systolic-array test vector",
        f"# ARRAY_SIZE={size}",
        f"# DATA_WIDTH={data_width}",
        f"# ACC_WIDTH={acc_width}",
    ]
    if name:
        lines.append(f"# NAME={name}")
    lines.append("A:")
    lines.extend(_format_matrix(A))
    lines.append("B:")
    lines.extend(_format_matrix(B))
    lines.append("Expected C:")
    lines.extend(_format_matrix(C))
    Path(path).write_text("\n".join(lines) + "\n")


def read_vector(path: str | Path) -> tuple[Matrix, Matrix, Matrix, dict[str, str]]:
    """
    Parses a human-readable test vector file (.txt) created by 'write_vector'
    and returns matrices A, B, C, and metadata dictionary.
    """
    meta: dict[str, str] = {}
    sections: dict[str, Matrix] = {"A": [], "B": [], "Expected C": []}
    current: str | None = None

    for raw_line in Path(path).read_text().splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith("#"):
            body = line[1:].strip()
            if "=" in body:
                key, _, value = body.partition("=")
                meta[key.strip()] = value.strip()
            continue
        if line in ("A:", "B:", "Expected C:"):
            current = line[:-1]
            continue
        if current is None:
            raise ValueError(f"{path}: value before any matrix header")
        row = [int(value) for value in line.split()]
        sections[current].append(row)

    A, B, C = sections["A"], sections["B"], sections["Expected C"]
    size = _validate_square(A, "A")
    if _validate_square(B, "B") != size:
        raise ValueError(f"{path}: A and B dimensions differ")
    if _validate_square(C, "Expected C") != size:
        raise ValueError(f"{path}: expected C has the wrong dimension")
    return A, B, C, meta


def _hex_digits(width: int) -> int:
    """Calculates number of hexadecimal digits needed to represent 'width' bits."""
    return (width + 3) // 4


def write_readmemh(
    path: str | Path,
    matrices: Iterable[Sequence[Sequence[int]]],
    width: int,
) -> None:
    """
    Writes a list of matrices to a flat text file in hexadecimal format compatible
    with Verilog's $readmemh system task (one hex value per line).
    """
    digits = _hex_digits(width)
    mask = (1 << width) - 1
    lines: list[str] = []
    for matrix in matrices:
        size = _validate_square(matrix, "matrix")
        for r in range(size):
            for c in range(size):
                # Write each element masked to 'width' as zero-padded hex
                lines.append(f"{matrix[r][c] & mask:0{digits}x}")
    Path(path).write_text("\n".join(lines) + "\n")


def write_rtl_vectors(
    directory: str | Path,
    A_cases: Sequence[Sequence[Sequence[int]]],
    B_cases: Sequence[Sequence[Sequence[int]]],
    C_cases: Sequence[Sequence[Sequence[int]]],
    data_width: int = DEFAULT_DATA_WIDTH,
    acc_width: int = DEFAULT_ACC_WIDTH,
) -> tuple[Path, Path, Path]:
    """
    Generates the a.hex, b.hex, and c.hex test vector files in 'directory'
    used by Verilog testbenches with +VECTORS_DIR=<dir>.
    """
    if not (len(A_cases) == len(B_cases) == len(C_cases)):
        raise ValueError("A/B/C case lists must have the same length")
    out_dir = Path(directory)
    out_dir.mkdir(parents=True, exist_ok=True)
    a_path, b_path, c_path = out_dir / "a.hex", out_dir / "b.hex", out_dir / "c.hex"
    write_readmemh(a_path, A_cases, data_width)
    write_readmemh(b_path, B_cases, data_width)
    write_readmemh(c_path, C_cases, acc_width)
    return a_path, b_path, c_path


def directed_cases(size: int, data_width: int, acc_width: int) -> list[tuple[str, Matrix, Matrix]]:
    """
    Generates a suite of deterministic, mathematically significant corner-case
    matrix pairs (A, B) for rigorous verification.

    Covers:
    - Zero matrices (null property: A * 0 = 0)
    - Identity matrices (neutral element: A * I = A)
    - Ones matrix
    - Maximum positive signed extremes (checking positive saturation/wrapping)
    - Minimum negative signed extremes (checking negative saturation/wrapping)
    - Checkerboard matrices (alternating +1 and -1)
    - Diagonal and triangular matrices (upper/lower triangular)
    - Ramp and transposed pairs
    """
    _validate_params(size, data_width, acc_width)
    n = size
    lo = -(1 << (data_width - 1))       # Minimum signed value (e.g. -128)
    hi = (1 << (data_width - 1)) - 1    # Maximum signed value (e.g. +127)

    def build(row_fn, col_fn) -> Matrix:
        return [[row_fn(r, c) for c in range(n)] for r in range(n)]

    zero        = build(lambda r, c: 0, None)
    identity    = build(lambda r, c: 1 if r == c else 0, None)
    ramp        = build(lambda r, c: r * n + c + 1, None)
    neg_ramp    = build(lambda r, c: -(r * n + c + 1), None)
    ones        = build(lambda r, c: 1, None)
    all_hi      = build(lambda r, c: hi, None)
    all_lo      = build(lambda r, c: lo, None)
    diagonal    = build(lambda r, c: (r + 1) if r == c else 0, None)
    lower       = build(lambda r, c: (r + 1) if c <= r else 0, None)
    upper       = build(lambda r, c: (c + 1) if r <= c else 0, None)
    alternating = build(lambda r, c: hi if (r + c) % 2 == 0 else lo, None)
    alt_inv     = build(lambda r, c: lo if (r + c) % 2 == 0 else hi, None)
    checker     = build(lambda r, c: 1 if (r + c) % 2 == 0 else -1, None)
    mixed       = build(lambda r, c: (r * n + c + 1) if (r + c) % 2 == 0 else -(r * n + c + 1), None)
    rank_one_a  = build(lambda r, c: (r + 1) if c == 0 else 0, None)
    rank_one_b  = build(lambda r, c: (c + 1) if r == 0 else 0, None)
    transpose   = [[ramp[c][r] for c in range(n)] for r in range(n)]

    pairs: list[tuple[str, Matrix, Matrix]] = [
        ("zero", zero, zero),
        ("identity", identity, identity),
        ("identity_left", identity, ramp),
        ("identity_right", ramp, identity),
        ("ones", ones, ones),
        ("max_positive", all_hi, all_hi),
        ("min_negative", all_lo, all_lo),
        ("max_times_min", all_hi, all_lo),
        ("alternating_extremes", alternating, alt_inv),
        ("diagonal", diagonal, diagonal),
        ("lower_upper", lower, upper),
        ("ramp", ramp, ramp),
        ("negative_ramp", neg_ramp, neg_ramp),
        ("ramp_times_negative", ramp, neg_ramp),
        ("rank_one", rank_one_a, rank_one_b),
        ("checkerboard", checker, checker),
        ("transpose_pair", ramp, transpose),
        ("mixed_signs", mixed, mixed),
        ("identity_accumulator", build(lambda r, c: (2 if r == c else 0), None), ramp),
    ]

    return [(name, A, B) for name, A, B in pairs if len(A) == n and len(B) == n]


def generate_vectors(
    size: int = DEFAULT_SIZE,
    data_width: int = DEFAULT_DATA_WIDTH,
    acc_width: int = DEFAULT_ACC_WIDTH,
    count: int = 20,
    seed: int = 42,
) -> list[Vector]:
    """
    Generates a full test vector set: all directed corner cases plus 'count'
    reproducible randomized test cases.
    """
    _validate_params(size, data_width, acc_width)
    if count < 0:
        raise ValueError("count must be nonnegative")

    vectors: list[Vector] = []
    # Add all directed cases
    for name, A, B in directed_cases(size, data_width, acc_width):
        vectors.append((name, A, B, matmul(A, B, data_width, acc_width)))

    # Add random test cases
    rng = random.Random(seed)
    lo = -(1 << (data_width - 1))
    hi = (1 << (data_width - 1)) - 1
    for index in range(count):
        A = [[rng.randint(lo, hi) for _ in range(size)] for _ in range(size)]
        B = [[rng.randint(lo, hi) for _ in range(size)] for _ in range(size)]
        vectors.append((f"random_{index:03d}", A, B, matmul(A, B, data_width, acc_width)))
    return vectors


def generate_test_vectors(
    size: int = DEFAULT_SIZE,
    data_width: int = DEFAULT_DATA_WIDTH,
    acc_width: int = DEFAULT_ACC_WIDTH,
    count: int = 20,
    seed: int = 42,
    out_dir: str | Path = DEFAULT_VECTORS_DIR,
) -> list[Vector]:
    """
    Generates test vectors and saves each one as an individual file (test_000.txt, ...)
    and writes an 'index.txt' manifest listing all generated test cases.
    """
    vectors = generate_vectors(size, data_width, acc_width, count, seed)
    out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    for index, (name, A, B, C) in enumerate(vectors):
        path = out_path / f"test_{index:03d}.txt"
        write_vector(path, A, B, C, data_width, acc_width, name=name)

    # Write manifest index file
    manifest = out_path / "index.txt"
    manifest.write_text(
        "\n".join(
            f"test_{index:03d}.txt {name}" for index, (name, _, _, _) in enumerate(vectors)
        )
        + "\n"
    )
    return vectors


def _positive_int(text: str) -> int:
    """Helper validator for argparse CLI positive integers."""
    value = int(text)
    if value < 1:
        raise argparse.ArgumentTypeError("value must be at least 1")
    return value


def main(argv: Sequence[str] | None = None) -> int:
    """Command-line interface entry point."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--size", type=_positive_int, default=DEFAULT_SIZE,
                        help="square matrix dimension N (default: 4)")
    parser.add_argument("--data-width", type=_positive_int, default=DEFAULT_DATA_WIDTH,
                        help="signed operand width (default: 8)")
    parser.add_argument("--acc-width", type=_positive_int, default=DEFAULT_ACC_WIDTH,
                        help="signed accumulator width (default: 32)")
    parser.add_argument("--count", type=int, default=20,
                        help="number of random cases in addition to directed cases")
    parser.add_argument("--seed", type=int, default=42, help="random seed")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_VECTORS_DIR,
                        help="directory for generated .txt vectors")
    parser.add_argument("--result-hex", help="verify an RTL result hex dump")
    parser.add_argument("--vector", type=Path,
                        help="vector file that supplies A/B for --result-hex")
    args = parser.parse_args(argv)

    if args.count < 0:
        parser.error("--count must be nonnegative")
    try:
        _validate_params(args.size, args.data_width, args.acc_width)
    except ValueError as exc:
        parser.error(str(exc))

    # Mode 1: Verifying an RTL simulation hex dump
    if args.result_hex is not None:
        if args.vector is None:
            parser.error("--result-hex requires --vector")
        A, B, _, _ = read_vector(args.vector)
        ok = verify_result_hex(args.result_hex, A, B, args.data_width, args.acc_width)
        print("RTL result matches the golden model" if ok else "RTL result MISMATCH")
        return 0 if ok else 1

    # Mode 2: Generating test vector files and running golden self-check
    print("=" * 68)
    print(
        f"Systolic Array Golden Model  (N={args.size}, DATA_WIDTH={args.data_width}, "
        f"ACC_WIDTH={args.acc_width})"
    )
    print("=" * 68)
    vectors = generate_test_vectors(
        args.size, args.data_width, args.acc_width, args.count, args.seed, args.out_dir
    )

    failures = 0
    for index, (name, A, B, C) in enumerate(vectors):
        recomputed = matmul(A, B, args.data_width, args.acc_width)
        ok = recomputed == C
        if not ok:
            failures += 1
            print(f"  test_{index:03d} ({name}): FAIL")
        else:
            print(f"  test_{index:03d} ({name}): PASS")
    print(f"\nGenerated {len(vectors)} vectors in {args.out_dir}")
    print(f"Golden self-check: {len(vectors) - failures} PASSED, {failures} FAILED")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
