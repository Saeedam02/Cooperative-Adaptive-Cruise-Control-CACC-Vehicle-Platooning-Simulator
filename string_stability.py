"""
string_stability.py

String stability is the central concept this whole project exists to
demonstrate: does a disturbance introduced at the front of a platoon
(the leader braking or accelerating) shrink as it propagates backward
through the following vehicles, or does it grow?

A platoon is "string stable" if the peak spacing error does NOT increase
from one vehicle to the next, for every vehicle in the chain. A platoon
that's only locally stable (each vehicle individually settles down) can
still be string UNSTABLE overall -- that's exactly the failure mode this
module measures.
"""

from dataclasses import dataclass
from typing import List

import numpy as np

from platoon import SimulationResult


@dataclass
class StringStabilityReport:
    peak_error_per_vehicle: np.ndarray   # index 0 is the leader (always 0)
    amplification_ratio: float           # peak error at the last vehicle / peak error at vehicle 1
    is_string_stable: bool               # True if peak error never increases down the chain
    worst_case_step_ratio: float         # largest single vehicle-to-vehicle error increase


def analyze(result: SimulationResult) -> StringStabilityReport:
    peak_per_vehicle = np.max(np.abs(result.spacing_errors), axis=0)

    # compare vehicle 1 (first follower) to the last vehicle in the platoon
    first_follower_peak = peak_per_vehicle[1]
    last_vehicle_peak = peak_per_vehicle[-1]
    amplification_ratio = (last_vehicle_peak / first_follower_peak
                            if first_follower_peak > 1e-9 else float("nan"))

    # check every consecutive pair (vehicle 1 onward) for growth
    followers_peaks = peak_per_vehicle[1:]
    step_ratios = followers_peaks[1:] / np.maximum(followers_peaks[:-1], 1e-9)
    worst_case_step_ratio = float(np.max(step_ratios)) if len(step_ratios) else float("nan")
    is_string_stable = bool(np.all(step_ratios <= 1.0 + 1e-6))

    return StringStabilityReport(
        peak_error_per_vehicle=peak_per_vehicle,
        amplification_ratio=float(amplification_ratio),
        is_string_stable=is_string_stable,
        worst_case_step_ratio=worst_case_step_ratio,
    )


def print_report(label: str, report: StringStabilityReport) -> None:
    print(f"--- {label} ---")
    print("Peak spacing error per vehicle (m):",
          np.round(report.peak_error_per_vehicle, 3).tolist())
    print(f"Amplification (last vehicle / first follower): {report.amplification_ratio:.2f}x")
    print(f"String stable (error never grows down the chain): {report.is_string_stable}")
    print(f"Worst single-hop growth ratio: {report.worst_case_step_ratio:.2f}x")
    print()
