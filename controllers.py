"""
controllers.py

Two spacing controllers for a following vehicle in a platoon, both built
on a Constant Time Headway (CTH) spacing policy -- the desired gap to the
vehicle ahead grows with your own speed:

    desired_gap = standstill_distance + time_headway * own_velocity

This is the same policy real adaptive cruise control systems use (it's
why your car's ACC leaves a bigger gap on the highway than in traffic).

Spacing error is defined as:
    e = actual_gap - desired_gap
  where actual_gap = predecessor_position - own_position - predecessor_length

A positive e means you're following at more than the desired distance
(too far back); negative means too close.

--- ACC (Adaptive Cruise Control) ---
Radar/camera-only: the vehicle can only sense the position and velocity of
the vehicle directly ahead. It has no way to know what that vehicle's
*acceleration* is until the effect shows up in its velocity a moment
later -- there's an inherent sensing delay baked into the physics.

--- CACC (Cooperative Adaptive Cruise Control) ---
Same feedback terms as ACC, PLUS a feedforward term using the predecessor's
*current* acceleration, transmitted directly over V2V (vehicle-to-vehicle)
communication rather than inferred from sensor lag. This is the actual
engineering reason CACC exists: it removes a step of reaction delay from
the control loop, which is what lets real-world CACC systems safely run
much shorter, more fuel-efficient headways than ACC without the platoon
becoming string unstable.
"""

from dataclasses import dataclass


@dataclass
class SpacingPolicy:
    standstill_distance: float = 2.0   # meters, gap at v = 0
    time_headway: float = 0.4          # seconds


def spacing_error(policy: SpacingPolicy, own_position, own_velocity,
                   predecessor_position, predecessor_length) -> float:
    actual_gap = predecessor_position - own_position - predecessor_length
    desired_gap = policy.standstill_distance + policy.time_headway * own_velocity
    return actual_gap - desired_gap


@dataclass
class ControllerGains:
    kp: float = 0.45   # proportional gain on spacing error
    kd: float = 0.9    # derivative gain on relative velocity
    kff: float = 1.0   # feedforward gain on predecessor acceleration (CACC only)


def acc_control(gains: ControllerGains, error: float, relative_velocity: float) -> float:
    """
    relative_velocity = predecessor_velocity - own_velocity
    Returns the commanded acceleration.
    """
    return gains.kp * error + gains.kd * relative_velocity


def cacc_control(gains: ControllerGains, error: float, relative_velocity: float,
                  predecessor_acceleration: float) -> float:
    """
    Same as ACC, plus a feedforward term using the predecessor's acceleration
    (assumed available via V2V communication, with no delay in this model --
    a real system would need to account for communication latency and
    packet loss, which is a natural extension of this project).
    """
    return (gains.kp * error
            + gains.kd * relative_velocity
            + gains.kff * predecessor_acceleration)
