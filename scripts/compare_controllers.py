"""Overlay trials and summarize outcomes from one controlled comparison suite."""
import argparse
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    reports = sorted(args.run.glob("trial-*/report.json"))
    if not reports:
        parser.error("No trial reports found")
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    summary = []
    for path in reports:
        report = json.loads(path.read_text())
        name = report["case_name"]
        data = np.genfromtxt(path.parent / "telemetry.csv", delimiter=",", names=True)
        for ax, key, unit in zip(axes.flat, ["x", "vx", "pitch", "slip_l"],
                                 ["Axle position (m)", "Speed (m/s)", "COM pitch (rad)", "Left slip speed (m/s)"]):
            ax.plot(data["t"], data[key], label=name)
            ax.set(xlabel="Time (s)", ylabel=unit)
            ax.grid(alpha=.3)
        summary.append({"case": name, "controller": report["controller"]["name"],
                        **{k: report[k] for k in ["stable_stop", "termination", "signed_stop_distance_m",
                           "max_excursion_after_brake_m", "stop_time_after_brake_s",
                           "max_supported_slip_after_brake_mps", "max_abs_pitch_deg"]}})
    axes[0,0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(args.run / "comparison.png", dpi=150)
    with (args.run / "comparison.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summary[0]))
        writer.writeheader()
        writer.writerows(summary)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
