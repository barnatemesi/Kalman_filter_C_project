"""
Continuous-time to discrete-time state-space model conversion.

Converts continuous-time matrices (A_c, B_c, C_c, D_c) to their
discrete-time equivalents (A_d, B_d, C_d, D_d) using zero-order hold (ZOH).

Discretization formulas:
    A_d = expm(A_c * Ts)
    B_d = inv(A_c) * (A_d - I) * B_c   (via matrix exponential block method)
    C_d = C_c
    D_d = D_c
"""

import argparse
import numpy as np
from scipy.linalg import expm


def continuous_to_discrete(A_c, B_c, C_c, D_c, Ts):
    """Convert continuous-time state-space model to discrete-time using ZOH.

    Uses the matrix exponential block method which handles singular A_c:
        expm([[A_c, B_c], [0, 0]] * Ts) = [[A_d, B_d], [0, I]]

    Args:
        A_c: State matrix (n x n).
        B_c: Input matrix (n x m).
        C_c: Output matrix (p x n).
        D_c: Feedthrough matrix (p x m).
        Ts:  Sampling period in seconds.

    Returns:
        Tuple of (A_d, B_d, C_d, D_d) discrete-time matrices.
    """
    A_c = np.atleast_2d(np.array(A_c, dtype=float))
    B_c = np.atleast_2d(np.array(B_c, dtype=float))
    C_c = np.atleast_2d(np.array(C_c, dtype=float))
    D_c = np.atleast_2d(np.array(D_c, dtype=float))

    n = A_c.shape[0]
    m = B_c.shape[1]

    # Build the block matrix [[A_c, B_c], [0, 0]] and exponentiate
    block = np.zeros((n + m, n + m))
    block[:n, :n] = A_c
    block[:n, n:] = B_c
    result = expm(block * Ts)

    A_d = result[:n, :n]
    B_d = result[:n, n:]
    C_d = C_c.copy()
    D_d = D_c.copy()

    return A_d, B_d, C_d, D_d


def print_matrix(name, mat):
    """Pretty-print a matrix with its name."""
    print(f"\n{name} =")
    rows, cols = mat.shape
    for i in range(rows):
        row_str = "  [ " + "  ".join(f"{mat[i, j]: .6f}" for j in range(cols)) + " ]"
        print(row_str)


def main():
    parser = argparse.ArgumentParser(
        description="Convert continuous-time state-space model to discrete-time (ZOH)."
    )
    parser.add_argument(
        "--Ts", type=float, required=True,
        help="Sampling period in seconds (e.g. 0.01 for 100 Hz)"
    )
    parser.add_argument(
        "--A", type=float, nargs="+", required=True,
        help="A matrix elements in row-major order (n*n values)"
    )
    parser.add_argument(
        "--B", type=float, nargs="+", required=True,
        help="B matrix elements in row-major order (n*m values)"
    )
    parser.add_argument(
        "--C", type=float, nargs="+", required=True,
        help="C matrix elements in row-major order (p*n values)"
    )
    parser.add_argument(
        "--D", type=float, nargs="+", required=True,
        help="D matrix elements in row-major order (p*m values)"
    )
    parser.add_argument(
        "--n", type=int, required=True,
        help="Number of states (rows of A)"
    )
    parser.add_argument(
        "--m", type=int, required=True,
        help="Number of inputs (columns of B)"
    )
    parser.add_argument(
        "--p", type=int, required=True,
        help="Number of outputs (rows of C)"
    )
    args = parser.parse_args()

    A_c = np.array(args.A).reshape(args.n, args.n)
    B_c = np.array(args.B).reshape(args.n, args.m)
    C_c = np.array(args.C).reshape(args.p, args.n)
    D_c = np.array(args.D).reshape(args.p, args.m)

    print(f"Sampling period: Ts = {args.Ts} s  (fs = {1.0 / args.Ts:.2f} Hz)")
    print("\n--- Continuous-time model ---")
    print_matrix("A_c", A_c)
    print_matrix("B_c", B_c)
    print_matrix("C_c", C_c)
    print_matrix("D_c", D_c)

    A_d, B_d, C_d, D_d = continuous_to_discrete(A_c, B_c, C_c, D_c, args.Ts)

    print("\n--- Discrete-time model (ZOH) ---")
    print_matrix("A_d", A_d)
    print_matrix("B_d", B_d)
    print_matrix("C_d", C_d)
    print_matrix("D_d", D_d)


if __name__ == "__main__":
    main()
