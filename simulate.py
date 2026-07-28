"""
simulate.py

Runs the same leader maneuver through two platoons -- one using ACC, one
using CACC -- and compares them: velocity profiles, spacing error over
time, and a direct string-stability comparison.

Usage:
    python simulate.py                     # default settings
    python simulate.py --headway 0.3       # shorter headway (more aggressive)
    python simulate.py --vehicles 10       # longer platoon
"""

import argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from controllers import SpacingPolicy, ControllerGains
from platoon import run_platoon_simulation, step_pulse_maneuver
from string_stability import analyze, print_report


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--vehicles", type=int, default=8)
    p.add_argument("--headway", type=float, default=0.4, help="time headway in seconds")
    p.add_argument("--duration", type=float, default=40.0)
    p.add_argument("--output", type=str, default="platoon_comparison.png")
    return p.parse_args()


def main():
    args = parse_args()

    policy = SpacingPolicy(standstill_distance=2.0, time_headway=args.headway)
    gains = ControllerGains(kp=0.45, kd=0.9, kff=1.0)
    leader_fn = step_pulse_maneuver(magnitude=1.0, start=5.0, duration=2.0)

    results = {}
    reports = {}
    for mode in ["ACC", "CACC"]:
        result = run_platoon_simulation(
            mode=mode,
            leader_accel_fn=leader_fn,
            num_vehicles=args.vehicles,
            duration=args.duration,
            policy=policy,
            gains=gains,
        )
        results[mode] = result
        reports[mode] = analyze(result)
        print_report(f"{mode}  (time headway = {args.headway}s)", reports[mode])

    _plot_comparison(results, reports, args)


def _plot_comparison(results, reports, args):
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    colors = plt.cm.viridis([i / args.vehicles for i in range(args.vehicles)])

    for col, mode in enumerate(["ACC", "CACC"]):
        result = results[mode]

        ax_v = axes[0, col]
        for i in range(args.vehicles):
            label = "Leader" if i == 0 else f"Vehicle {i}"
            ax_v.plot(result.time, result.velocities[:, i], color=colors[i], label=label, linewidth=1.3)
        ax_v.set_title(f"{mode}: Velocity of every vehicle in the platoon")
        ax_v.set_xlabel("Time (s)")
        ax_v.set_ylabel("Velocity (m/s)")
        if col == 1:
            ax_v.legend(fontsize=7, loc="upper right", ncol=2)

        ax_e = axes[1, col]
        for i in range(1, args.vehicles):
            ax_e.plot(result.time, result.spacing_errors[:, i], color=colors[i], linewidth=1.3)
        ax_e.set_title(f"{mode}: Spacing error of every follower")
        ax_e.set_xlabel("Time (s)")
        ax_e.set_ylabel("Spacing error (m)")
        ax_e.axhline(0, color="black", linewidth=0.6)

    fig.suptitle(
        f"Platoon comparison — {args.vehicles} vehicles, {args.headway}s time headway\n"
        f"ACC peak-error amplification: {reports['ACC'].amplification_ratio:.2f}x   |   "
        f"CACC peak-error amplification: {reports['CACC'].amplification_ratio:.2f}x",
        fontsize=12,
    )
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(args.output, dpi=130)
    print(f"Saved comparison plot to {args.output}")

    _plot_string_stability_bars(reports, args)


def _plot_string_stability_bars(reports, args):
    fig, ax = plt.subplots(figsize=(7, 4.5))
    vehicle_indices = list(range(1, args.vehicles))
    width = 0.35

    ax.bar([i - width / 2 for i in vehicle_indices],
           reports["ACC"].peak_error_per_vehicle[1:], width=width, label="ACC", color="#d9534f")
    ax.bar([i + width / 2 for i in vehicle_indices],
           reports["CACC"].peak_error_per_vehicle[1:], width=width, label="CACC", color="#2e7d32")

    ax.set_xlabel("Vehicle position in platoon (1 = right behind leader)")
    ax.set_ylabel("Peak spacing error (m)")
    ax.set_title("String stability: peak spacing error by platoon position")
    ax.set_xticks(vehicle_indices)
    ax.legend()
    fig.tight_layout()
    fig.savefig("string_stability_comparison.png", dpi=130)
    print("Saved string stability bar chart to string_stability_comparison.png")


if __name__ == "__main__":
    main()
