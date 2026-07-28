"""
vehicle.py

A longitudinal vehicle model used throughout the platoon simulation.

Each vehicle has three states: position, velocity, and *actual*
acceleration. The actual acceleration doesn't jump instantly to whatever
the controller commands -- it lags behind through a first-order transfer
function, which is a standard, well-established way to represent engine/
brake actuator dynamics in ACC/CACC literature:

    tau * da/dt = u - a

where `u` is the commanded acceleration (the controller's output) and
`tau` is the actuator time constant. This lag is exactly what makes
string stability a real engineering problem instead of a triviality --
without it, spacing errors wouldn't need a full control law to manage.
"""

from dataclasses import dataclass


@dataclass
class Vehicle:
    position: float
    velocity: float
    acceleration: float = 0.0
    length: float = 4.5       # meters, used for bumper-to-bumper spacing
    tau: float = 0.5          # actuator time constant (seconds)

    def step(self, commanded_accel: float, dt: float) -> None:
        """Advance the vehicle's state by one simulation timestep using
        simple forward-Euler integration (accurate enough at dt <= 0.05s
        for this kind of longitudinal simulation)."""
        self.acceleration += dt * ((commanded_accel - self.acceleration) / self.tau)
        self.velocity += dt * self.acceleration
        self.position += dt * self.velocity
