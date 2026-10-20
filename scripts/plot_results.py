"""
plot_results.py – Generate visual figures and statistics from experiment CSV data.

Reads recorded trials from results/experiment_results.csv and produces:
  1. results/displacement_vs_delay.png (Measured vs Expected displacement & error)
  2. results/trajectory_xy.png (Initial vs Final XY coordinates)
  3. results/time_vs_delay.png (Elapsed wall-clock time vs commanded duration)

Run with:
    python scripts/plot_results.py
    python scripts/plot_results.py --csv results/experiment_results.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path


def load_results(csv_path: Path | str) -> list[dict]:
    """Load and parse CSV trial records."""
    path = Path(csv_path)
    if not path.exists():
        return []

    records = []
    with open(path, mode="r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                records.append({
                    "timestamp": row.get("timestamp", ""),
                    "tag": row.get("tag", ""),
                    "delay_ms": float(row.get("delay_ms", 0.0)),
                    "trial_num": int(row.get("trial_num", 1)),
                    "command": row.get("command", "FORWARD"),
                    "commanded_speed": float(row.get("commanded_speed", 0.05)),
                    "commanded_duration": float(row.get("commanded_duration", 2.0)),
                    "expected_displacement": float(row.get("expected_displacement", 0.10)),
                    "start_x": float(row.get("start_x", 0.0)),
                    "start_y": float(row.get("start_y", 0.0)),
                    "start_heading": float(row.get("start_heading", 0.0)),
                    "end_x": float(row.get("end_x", 0.0)),
                    "end_y": float(row.get("end_y", 0.0)),
                    "end_heading": float(row.get("end_heading", 0.0)),
                    "euclidean_displacement": float(row.get("euclidean_displacement", 0.0)),
                    "signed_x_displacement": float(row.get("signed_x_displacement", 0.0)),
                    "signed_y_displacement": float(row.get("signed_y_displacement", 0.0)),
                    "displacement_error": float(row.get("displacement_error", 0.0)),
                    "x_displacement_error": float(row.get("x_displacement_error", 0.0)),
                    "elapsed_time": float(row.get("elapsed_time", 0.0)),
                    "final_state": row.get("final_state", ""),
                    "is_stopped": row.get("is_stopped", "True").lower() == "true",
                    "success": row.get("success", "True").lower() == "true",
                    "error_message": row.get("error_message", ""),
                })
            except (ValueError, TypeError) as exc:
                continue
    return records


def print_summary_table(records: list[dict]) -> None:
    """Print statistical summary table across tested latency delays."""
    successful = [r for r in records if r["success"]]
    if not successful:
        print("[Notice] No successful trials found in dataset.")
        return

    grouped: dict[float, list[dict]] = defaultdict(list)
    for r in successful:
        grouped[r["delay_ms"]].append(r)

    print("\n" + "=" * 80)
    print("EXPERIMENTAL BENCHMARK SUMMARY TABLE")
    print("=" * 80)
    header = f"{'Delay [ms]':<12} | {'Trials':<8} | {'Mean Disp [m]':<15} | {'Mean Error [m]':<16} | {'Mean Time [s]':<14} | {'Expected [m]':<12}"
    print(header)
    print("-" * len(header))

    for delay in sorted(grouped.keys()):
        trials = grouped[delay]
        n = len(trials)
        disps = [t["euclidean_displacement"] for t in trials]
        errors = [t["displacement_error"] for t in trials]
        times = [t["elapsed_time"] for t in trials]
        expected = trials[0]["expected_displacement"]

        mean_disp = sum(disps) / n
        mean_err = sum(errors) / n
        mean_time = sum(times) / n

        print(f"{delay:<12.1f} | {n:<8} | {mean_disp:<15.4f} | {mean_err:<+16.4f} | {mean_time:<14.2f} | {expected:<12.4f}")
    print("=" * 80)
    print("Note: Preliminary baseline measurements. Real-world trajectory depends on physics time-step.")


def generate_plots(records: list[dict], output_dir: Path | str) -> list[str]:
    """Generate Matplotlib plots from experiment records and save PNG files."""
    try:
        import matplotlib
        matplotlib.use("Agg")  # Non-interactive headless backend
        import matplotlib.pyplot as plt
    except ImportError:
        print("[Error] Matplotlib is required to generate plots. Install with: pip install matplotlib", file=sys.stderr)
        return []

    successful = [r for r in records if r["success"]]
    if not successful:
        print("[Notice] No successful records to plot. Skipping plot generation.")
        return []

    out_path = Path(output_dir)
    out_path.mkdir(parents=True, exist_ok=True)
    generated: list[str] = []

    # Group by delay_ms
    grouped: dict[float, list[dict]] = defaultdict(list)
    for r in successful:
        grouped[r["delay_ms"]].append(r)

    delays = sorted(grouped.keys())
    mean_disps = []
    std_disps = []
    mean_errors = []
    mean_times = []
    expected_disps = []

    for d in delays:
        trials = grouped[d]
        n = len(trials)
        d_vals = [t["euclidean_displacement"] for t in trials]
        e_vals = [t["displacement_error"] for t in trials]
        t_vals = [t["elapsed_time"] for t in trials]

        m_d = sum(d_vals) / n
        m_e = sum(e_vals) / n
        m_t = sum(t_vals) / n
        std_d = math.sqrt(sum((x - m_d) ** 2 for x in d_vals) / n) if n > 1 else 0.0

        mean_disps.append(m_d)
        std_disps.append(std_d)
        mean_errors.append(m_e)
        mean_times.append(m_t)
        expected_disps.append(trials[0]["expected_displacement"])

    # -----------------------------------------------------------------------
    # Figure 1: Displacement & Error vs Delay
    # -----------------------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    fig.suptitle(
        "Robot Displacement & Error vs Added Network Latency\n(Preliminary Baseline Measurements)",
        fontsize=12,
        fontweight="bold",
    )

    # Plot 1: Measured vs Expected Displacement
    ax1.errorbar(delays, mean_disps, yerr=std_disps, fmt="o-", color="#1f77b4", capsize=5, label="Measured Mean ± Std")
    ax1.plot(delays, expected_disps, "--", color="#d62728", label="Kinematic Expected (v × t)")
    ax1.set_xlabel("Added Network Latency [ms]", fontsize=10)
    ax1.set_ylabel("Displacement [m]", fontsize=10)
    ax1.set_title("Total Euclidean Displacement", fontsize=11)
    ax1.grid(True, linestyle=":", alpha=0.6)
    ax1.legend(loc="best")

    # Plot 2: Displacement Error
    ax2.bar([str(int(d)) for d in delays], mean_errors, color="#2ca02c", alpha=0.8, edgecolor="black")
    ax2.axhline(0.0, color="black", linestyle="-", linewidth=0.8)
    ax2.set_xlabel("Added Network Latency [ms]", fontsize=10)
    ax2.set_ylabel("Displacement Error [m]", fontsize=10)
    ax2.set_title("Displacement Error (Measured - Expected)", fontsize=11)
    ax2.grid(True, linestyle=":", alpha=0.6)

    fig.tight_layout()
    f1_path = out_path / "displacement_vs_delay.png"
    fig.savefig(f1_path, dpi=200)
    plt.close(fig)
    generated.append(str(f1_path))

    # -----------------------------------------------------------------------
    # Figure 2: Trajectory XY Start & End Poses
    # -----------------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(8, 6))
    colors = plt.cm.viridis([i / max(1, len(delays) - 1) for i in range(len(delays))])

    for i, d in enumerate(delays):
        trials = grouped[d]
        sx = [t["start_x"] for t in trials]
        sy = [t["start_y"] for t in trials]
        ex = [t["end_x"] for t in trials]
        ey = [t["end_y"] for t in trials]

        ax.scatter(sx, sy, marker="o", color=colors[i], label=f"Start ({int(d)} ms)", alpha=0.7)
        ax.scatter(ex, ey, marker="^", color=colors[i], label=f"End ({int(d)} ms)", s=60)
        for j in range(len(sx)):
            ax.annotate("", xy=(ex[j], ey[j]), xytext=(sx[j], sy[j]),
                        arrowprops=dict(arrowstyle="->", color=colors[i], lw=1.2, alpha=0.6))

    ax.set_xlabel("X Position [m]", fontsize=10)
    ax.set_ylabel("Y Position [m]", fontsize=10)
    ax.set_title("Robot Trajectory Vectors in CoppeliaSim Coordinates\n(Preliminary Baseline Measurements)", fontsize=11, fontweight="bold")
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.axis("equal")
    ax.legend(bbox_to_anchor=(1.04, 1), loc="upper left", fontsize=8)

    fig.tight_layout()
    f2_path = out_path / "trajectory_xy.png"
    fig.savefig(f2_path, dpi=200)
    plt.close(fig)
    generated.append(str(f2_path))

    # -----------------------------------------------------------------------
    # Figure 3: Elapsed Wall Time vs Delay
    # -----------------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(8, 5))
    cmd_duration = successful[0]["commanded_duration"]

    ax.plot(delays, mean_times, "s-", color="#9467bd", label="Measured Elapsed Wall Time")
    ax.axhline(cmd_duration, color="#d62728", linestyle="--", label=f"Commanded Duration ({cmd_duration:.1f} s)")

    ax.set_xlabel("Added Network Latency [ms]", fontsize=10)
    ax.set_ylabel("Elapsed Time [s]", fontsize=10)
    ax.set_title("Elapsed Trial Execution Time vs Injected Latency\n(Preliminary Baseline Measurements)", fontsize=11, fontweight="bold")
    ax.grid(True, linestyle=":", alpha=0.6)
    ax.legend(loc="best")

    fig.tight_layout()
    f3_path = out_path / "time_vs_delay.png"
    fig.savefig(f3_path, dpi=200)
    plt.close(fig)
    generated.append(str(f3_path))

    return generated


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate benchmark plots from experiment CSV data.")
    parser.add_argument("--csv", default="results/experiment_results.csv", help="Input CSV path (default: results/experiment_results.csv)")
    parser.add_argument("--outdir", default="results", help="Directory to save generated PNG plots (default: results)")

    args = parser.parse_args()

    records = load_results(args.csv)
    if not records:
        print(f"[Notice] No experiment results found at '{args.csv}'.")
        print("Run baseline experiment first: python scripts/run_experiments.py")
        return

    print_summary_table(records)
    plots = generate_plots(records, args.outdir)
    if plots:
        print("\nGenerated Plot Files:")
        for p in plots:
            print(f"  [PNG] {p}")


if __name__ == "__main__":
    main()
