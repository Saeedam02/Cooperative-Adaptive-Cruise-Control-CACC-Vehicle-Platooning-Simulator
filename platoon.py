"""
platoon.py

Builds and simulates a platoon of N vehicles: one leader (whose trajectory
is prescribed, not controlled) followed by N-1 vehicles each running
either an ACC or a CACC spacing controller.

The platoon is initialized already at its steady-state desired spacing, so
any spacing error that appears during the simulation is *caused by* the
leader's maneuver propagating backward through the platoon -- not by a
mismatched starting condition. That distinction matters: it's what makes
the resulting plots an honest measurement of string stability rather than
an artifact of how the simulation was set up.
"""

from dataclasses import dataclass
from typing import Callable, List

import numpy as np

from vehicle import Vehicle
from controllers import SpacingPolicy, ControllerGains, spacing_error, acc_control, cacc_control


@dataclass
class SimulationResult:
    time: np.ndarray
    positions: np.ndarray          # shape (steps, N)
    velocities: np.ndarray         # shape (steps, N)
    accelerations: np.ndarray      # shape (steps, N)
    spacing_errors: np.ndarray     # shape (steps, N); column 0 is always 0 (no predecessor)


def run_platoon_simulation(
    mode: str,                                  # "ACC" or "CACC"
    leader_accel_fn: Callable[[float], float],  # t -> commanded leader acceleration
    num_vehicles: int = 8,
    initial_speed: float = 20.0,                # m/s (~72 km/h)
    duration: float = 40.0,
    dt: float = 0.05,
    policy: SpacingPolicy = None,
    gains: ControllerGains = None,
) -> SimulationResult:
    if mode not in ("ACC", "CACC"):
        raise ValueError("mode must be 'ACC' or 'CACC'")

    policy = policy or SpacingPolicy()
    gains = gains or ControllerGains()

    # Initialize every vehicle exactly at the CTH-desired spacing for the
    # initial speed, so the platoon starts with zero spacing error.
    vehicles: List[Vehicle] = []
    position = 0.0
    for i in range(num_vehicles):
        vehicles.append(Vehicle(position=position, velocity=initial_speed))
        position -= vehicles[-1].length + policy.standstill_distance + policy.time_headway * initial_speed

    steps = int(duration / dt)
    time = np.arange(steps) * dt
    positions = np.zeros((steps, num_vehicles))
    velocities = np.zeros((steps, num_vehicles))
    accelerations = np.zeros((steps, num_vehicles))
    errors = np.zeros((steps, num_vehicles))

    for k in range(steps):
        t = time[k]
        commanded = [leader_accel_fn(t)]

        for i in range(1, num_vehicles):
            predecessor = vehicles[i - 1]
            own = vehicles[i]

            e = spacing_error(policy, own.position, own.velocity,
                               predecessor.position, predecessor.length)
            relative_velocity = predecessor.velocity - own.velocity

            if mode == "ACC":
                u = acc_control(gains, e, relative_velocity)
            else:
                u = cacc_control(gains, e, relative_velocity, predecessor.acceleration)

            commanded.append(u)
            errors[k, i] = e

        # record state BEFORE integrating, then advance every vehicle
        for i, vehicle in enumerate(vehicles):
            positions[k, i] = vehicle.position
            velocities[k, i] = vehicle.velocity
            accelerations[k, i] = vehicle.acceleration

        for vehicle, u in zip(vehicles, commanded):
            vehicle.step(u, dt)

    return SimulationResult(time, positions, velocities, accelerations, errors)


def step_pulse_maneuver(magnitude: float = 1.0, start: float = 5.0, duration: float = 2.0):
    """A leader acceleration profile: 0, then a brief accel pulse, then back to 0.
    Models something like a brief speed-up (e.g. merging traffic clearing)."""
    def fn(t: float) -> float:
        return magnitude if start <= t < start + duration else 0.0
    return fn
