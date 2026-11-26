"""
Unit Tests for the Golden Model (golden_model.py)

This test suite verifies that the Python golden model itself is 100% mathematically
correct, robust against edge cases, and faithfully simulates hardware bit-widths
before we use it to verify the Verilog RTL.

Key Test Categories:
1. Mathematical Equivalence: Cross-checks against a naive unrolled reference matrix multiply.
2. Two's Complement Overflow Wrapping: Validates that integer overflow exactly replicates hardware.
3. Bit Packaging & Serialization: Tests 'pack_matrix' and 'unpack_matrix' round-trip consistency.
4. File I/O: Tests human-readable .txt vectors and Verilog $readmemh hex outputs.
5. Error Handling: Verifies rejection of illegal dimensions and bit-widths.
"""

from __future__ import annotations

import random
import sys
import tempfile
import unittest
from pathlib import Path

# Add verification directory to Python import search path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import golden_model as gm


def naive_matmul(A, B, data_width, acc_width):
    """
    Independent naive reference implementation of matrix multiplication.
    Used as an unoptimized secondary check to validate gm.matmul.
    """
    n = len(A)
    C = [[0] * n for _ in range(n)]
    for r in range(n):
        for c in range(n):
            acc = 0
            for k in range(n):
                # Multiply signed wrapped operands and accumulate
                acc += gm.wrap_signed(A[r][k], data_width) * gm.wrap_signed(B[k][c], data_width)
            # Wrap final accumulated sum to accumulator bit-width
            C[r][c] = gm.wrap_signed(acc, acc_width)
    return C


class GoldenModelTests(unittest.TestCase):
    """Unit test cases for validating golden_model.py functions."""

    def test_matmul_matches_naive_across_sizes(self):
        """
        Tests that gm.matmul matches naive_matmul across multiple matrix dimensions
        (N = 1, 2, 3, 4, 5) and multiple bit-width combinations (8/32, 16/48, 8/16).
        """
        rng = random.Random(7)
        for size in (1, 2, 3, 4, 5):
            for data_width, acc_width in ((8, 32), (16, 48), (8, 16)):
                lo = -(1 << (data_width - 1))
                hi = (1 << (data_width - 1)) - 1
                for _ in range(5):
                    # Generate random square matrices A and B within signed range [lo, hi]
                    A = [[rng.randint(lo, hi) for _ in range(size)] for _ in range(size)]
                    B = [[rng.randint(lo, hi) for _ in range(size)] for _ in range(size)]
                    self.assertEqual(
                        gm.matmul(A, B, data_width, acc_width),
                        naive_matmul(A, B, data_width, acc_width),
                        msg=f"size={size} dw={data_width} aw={acc_width}",
                    )

    def test_accumulator_wraparound_matches_rtl(self):
        """
        Verifies accumulator overflow wrapping in 16-bit mode:
        Multiplying 127 * 127 = 16129. Over 4 terms: 16129 * 4 = 64516.
        In 16-bit signed arithmetic, 64516 wraps around: 64516 - 65536 = -1020.
        Confirms golden model wraps identically to Verilog hardware.
        """
        A = [[127] * 4 for _ in range(4)]
        B = [[127] * 4 for _ in range(4)]
        C = gm.matmul(A, B, data_width=8, acc_width=16)
        self.assertEqual(C[0][0], 64516 - 65536)
        self.assertEqual(C, [[-1020] * 4 for _ in range(4)])

    def test_minimum_accumulator_width_is_supported(self):
        """
        Verifies behavior when ACC_WIDTH is at the minimum allowed limit (2 * DATA_WIDTH).
        For 8-bit operands, ACC_WIDTH = 16 bits.
        """
        A = [[127, 0], [0, 127]]
        B = [[127, 0], [0, 127]]
        C = gm.matmul(A, B, data_width=8, acc_width=16)
        self.assertEqual(C, [[gm.wrap_signed(16129, 16), 0], [0, gm.wrap_signed(16129, 16)]])

    def test_inputs_are_wrapped_to_data_width(self):
        """
        Verifies that out-of-range integer inputs are correctly wrapped to data_width
        before multiplication (e.g. 300 wraps to 44 in 8-bit, -129 wraps to +127).
        """
        self.assertEqual(gm.matmul([[300]], [[2]], data_width=8, acc_width=32), [[88]])
        self.assertEqual(gm.matmul([[-129]], [[1]], data_width=8, acc_width=32), [[127]])

    def test_signed_extremes_across_widths(self):
        """
        Tests combinations of maximum positive and minimum negative signed values
        across various bit-widths (4/8, 8/16, 8/32, 16/48).
        """
        for data_width, acc_width in ((4, 8), (8, 16), (8, 32), (16, 48)):
            lo = -(1 << (data_width - 1))
            hi = (1 << (data_width - 1)) - 1
            A = [[lo, hi], [hi, lo]]
            B = [[lo, lo], [hi, hi]]
            self.assertEqual(
                gm.matmul(A, B, data_width, acc_width),
                naive_matmul(A, B, data_width, acc_width),
            )

    def test_pack_unpack_roundtrip(self):
        """
        Tests serialization round-trip: matrix -> pack_matrix -> unpack_matrix -> matrix.
        Confirms lossless conversion between Python 2D lists and 1D hardware bit-buses.
        """
        rng = random.Random(11)
        for size in (1, 2, 3, 4, 8):
            for width in (8, 16):
                lo = -(1 << (width - 1))
                hi = (1 << (width - 1)) - 1
                matrix = [[rng.randint(lo, hi) for _ in range(size)] for _ in range(size)]
                packed = gm.pack_matrix(matrix, width)
                # Unpack and verify perfect equality with original matrix
                self.assertEqual(gm.unpack_matrix(packed, size, width), matrix)
                # Verify hex string length matches required nibble count
                self.assertEqual(len(gm.matrix_to_hex(matrix, width)), (size * size * width + 3) // 4)

    def test_pack_bit_layout(self):
        """
        Tests exact bit-field placement of elements in packed bus:
        Matrix [[1, 2], [3, 4]] with 8-bit width should pack into:
        Byte 0: 0x01, Byte 1: 0x02, Byte 2: 0x03, Byte 3: 0x04 -> 0x04030201 in hex.
        """
        matrix = [[1, 2], [3, 4]]
        packed = gm.pack_matrix(matrix, 8)
        self.assertEqual(packed, 1 | (2 << 8) | (3 << 16) | (4 << 24))
        self.assertEqual(gm.matrix_to_hex(matrix, 8), "04030201")

    def test_verify_result_hex(self):
        """
        Tests verify_result_hex helper:
        - True on correct hex string.
        - False on corrupted hex string.
        - True on uppercase / 0x-prefixed hex string.
        """
        A = [[1, 2], [3, 4]]
        B = [[5, 6], [7, 8]]
        C = gm.matmul(A, B, 8, 32)
        packed = gm.pack_matrix(C, 32)
        hex_result = f"{packed:0{(2 * 2 * 32 + 3) // 4}x}"
        self.assertTrue(gm.verify_result_hex(hex_result, A, B, 8, 32, verbose=False))
        # Corrupt 1 bit and verify rejection
        corrupted = f"{packed ^ 1:0{(2 * 2 * 32 + 3) // 4}x}"
        self.assertFalse(gm.verify_result_hex(corrupted, A, B, 8, 32, verbose=False))
        # Verify case insensitivity and 0x prefix tolerance
        self.assertTrue(gm.verify_result_hex("0x" + hex_result.upper(), A, B, 8, 32, verbose=False))

    def test_vector_file_roundtrip(self):
        """
        Tests write_vector and read_vector with temporary files on disk:
        Confirms metadata headers and matrix formatting parse correctly.
        """
        A = [[-1, 2], [3, -4]]
        B = [[5, -6], [-7, 8]]
        C = gm.matmul(A, B, 8, 32)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "case.txt"
            gm.write_vector(path, A, B, C, 8, 32, name="roundtrip")
            A2, B2, C2, meta = gm.read_vector(path)
        self.assertEqual((A2, B2, C2), (A, B, C))
        self.assertEqual(meta["ARRAY_SIZE"], "2")
        self.assertEqual(meta["DATA_WIDTH"], "8")
        self.assertEqual(meta["ACC_WIDTH"], "32")
        self.assertEqual(meta["NAME"], "roundtrip")

    def test_readmemh_files_are_flat_and_wrapped(self):
        """
        Tests export to Verilog $readmemh hex files (write_rtl_vectors).
        Confirms file format has 1 hex line per matrix element in row-major order.
        """
        A = [[-1, 2], [3, -4]]
        B = [[5, -6], [-7, 8]]
        C = gm.matmul(A, B, 8, 32)
        with tempfile.TemporaryDirectory() as tmp:
            a_path, b_path, c_path = gm.write_rtl_vectors(tmp, [A], [B], [C], 8, 32)
            a_lines = a_path.read_text().split()
            b_lines = b_path.read_text().split()
            c_lines = c_path.read_text().split()
        # In 8-bit hex: -1 -> "ff", 2 -> "02", 3 -> "03", -4 -> "fc"
        self.assertEqual(a_lines, ["ff", "02", "03", "fc"])
        self.assertEqual(b_lines, ["05", "fa", "f9", "08"])
        self.assertEqual(len(c_lines), 4)
        self.assertEqual(
            [gm.wrap_signed(int(line, 16), 32) for line in c_lines],
            [C[0][0], C[0][1], C[1][0], C[1][1]],
        )

    def test_rejects_invalid_dimensions(self):
        """
        Tests that invalid inputs trigger ValueError exceptions:
        - Non-square matrices (1x2)
        - Insufficient accumulator width (acc_width < 2*data_width)
        - Ragged matrices
        - Zero data_width
        """
        with self.assertRaises(ValueError):
            gm.matmul([[1, 2]], [[1], [2]], 8, 32)  # Non-square
        with self.assertRaises(ValueError):
            gm.matmul([[1, 2], [3, 4]], [[1, 2], [3, 4]], 8, 8)  # acc_width too small
        with self.assertRaises(ValueError):
            gm.matmul([[1, 2], [3]], [[1, 2], [3, 4]], 8, 32)  # Ragged matrix
        with self.assertRaises(ValueError):
            gm.matmul([[1]], [[1]], 0, 32)  # Zero data_width

    def test_directed_cases_are_square_and_correct(self):
        """
        Tests that all deterministic corner-case generator functions in directed_cases()
        produce valid square matrices and match naive_matmul.
        """
        for size in (1, 2, 3, 4, 5):
            for data_width, acc_width in ((8, 32), (16, 48)):
                cases = gm.directed_cases(size, data_width, acc_width)
                self.assertGreaterEqual(len(cases), 15)
                for name, A, B in cases:
                    self.assertEqual(len(A), size, msg=name)
                    self.assertTrue(all(len(row) == size for row in A), msg=name)
                    self.assertEqual(len(B), size, msg=name)
                    self.assertEqual(
                        gm.matmul(A, B, data_width, acc_width),
                        naive_matmul(A, B, data_width, acc_width),
                        msg=name,
                    )


if __name__ == "__main__":
    unittest.main(verbosity=2)
