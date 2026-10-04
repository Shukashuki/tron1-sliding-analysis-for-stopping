"""Controller reference and measurements, independent of the Isaac GUI."""
from dataclasses import dataclass, asdict
import math
import numpy as np


@dataclass(frozen=True)
class Settings:
    initial_speed: float = .5
    static_friction: float = .8
    dynamic_friction: float = .6
    torque_conversion: float = 12.
    torque_limit: float = 12.
    no_load_speed: float = 100.
    shaft_resistance: float = 0.
    brake_at: float = 1.
    deceleration: float = .8
    duration: float = 8.

    def __post_init__(self):
        bounds = {"initial_speed": (-3, 3), "static_friction": (0, 2),
                  "dynamic_friction": (0, 2), "torque_conversion": (0, 40),
                  "torque_limit": (.1, 40), "no_load_speed": (1, 200),
                  "shaft_resistance": (0, 2), "brake_at": (0, 20),
                  "deceleration": (.05, 5), "duration": (2, 60)}
        for name, (low, high) in bounds.items():
            value = getattr(self, name)
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f"{name}: expected [{low}, {high}]")
        if self.dynamic_friction > self.static_friction:
            raise ValueError("Dynamic friction must not exceed static friction")
        if self.brake_at >= self.duration:
            raise ValueError("Brake time must precede trial end")


def reference(t, speed, brake_at, deceleration):
    """Continuous position/speed; acceleration changes at brake and ramp end."""
    elapsed = max(0., t - brake_at)
    ramp = min(elapsed, abs(speed) / deceleration)
    sign = np.sign(speed)
    x = speed * min(t, brake_at) + speed * ramp - .5 * sign * deceleration * ramp**2
    v = speed - sign * deceleration * ramp
    a = -sign * deceleration if 0 <= t - brake_at < abs(speed) / deceleration else 0.
    return float(x), float(v), float(a)


def motor_torque(command, omega, cfg):
    """Nm/unit gain, four-quadrant DC envelope, then smooth shaft drag.

    command is dimensionless. Gain is NOT a physical gearbox model.
    Braking at positive speed admits negative torque down to -stall torque.
    """
    command, omega = np.asarray(command), np.asarray(omega)
    requested = np.clip(command, -1., 1.) * cfg.torque_conversion
    lower = np.clip(cfg.torque_limit * (-1 - omega / cfg.no_load_speed), -cfg.torque_limit, 0.)
    upper = np.clip(cfg.torque_limit * (1 - omega / cfg.no_load_speed), 0., cfg.torque_limit)
    motor = np.clip(requested, lower, upper)
    net = np.clip(motor - cfg.shaft_resistance * np.tanh(omega / .5), -cfg.torque_limit, cfg.torque_limit)
    return net


def slip_metrics(axle_velocity, angular_velocity, radius):
    """World-X bottom-point velocity for upright wheels on horizontal ground.

    Uses full rigid-body angular velocity (including chassis rotation), not
    just the relative joint encoder speed. Airborne values are kinematic only.
    """
    axle_velocity, angular_velocity = np.asarray(axle_velocity), np.asarray(angular_velocity)
    rolling_speed = radius * angular_velocity[..., 1]
    slip_speed = axle_velocity[..., 0] - rolling_speed
    denom = np.maximum(np.maximum(np.abs(axle_velocity[..., 0]), np.abs(rolling_speed)), .05)
    return rolling_speed, slip_speed, -slip_speed / denom


def summarize(rows, cfg, reason):
    braking = [r for r in rows if r["braking"]]
    first = braking[0] if braking else None
    dwell, stop, stable_since = 0., None, None
    for r in braking:
        stable = (abs(r["vx"]) < .05 and abs(r["pitch"]) < math.radians(10)
                  and abs(r["roll"]) < math.radians(10)
                  and abs(r["rolling_l"]) < .05 and abs(r["rolling_r"]) < .05
                  and min(r["normal_l"], r["normal_r"]) > 1.)
        if stable:
            if stable_since is None:
                stable_since = r["t"]
            dwell = r["t"] - stable_since
        else:
            dwell, stable_since = 0., None
        if dwell >= .5 - 1e-9 and stop is None:
            stop = r
    stopped = stop is not None and dwell >= .5 - 1e-9 and reason == "completed"
    return {"settings": asdict(cfg), "termination": reason, "stable_stop": stopped,
            "brake_x_m": first["x"] if first else None,
            "stop_time_after_brake_s": stop["t"] - first["t"] if stopped else None,
            "signed_stop_distance_m": stop["x"] - first["x"] if stopped else None,
            "final_displacement_after_brake_m": braking[-1]["x"]-first["x"] if first else None,
            "max_excursion_after_brake_m": max(abs(r["x"]-first["x"]) for r in braking) if first else None,
            "distance_definition": "signed_stop_distance is displacement at first confirmed stop; max_excursion is greatest absolute travel from brake position, including overshoot; final_displacement uses trial endpoint",
            "max_abs_slip_speed_mps": max((max(abs(r["slip_l"]), abs(r["slip_r"])) for r in rows), default=0),
            "max_supported_slip_after_brake_mps": max((abs(r['slip_'+side]) for r in braking for side in ('l','r') if r['normal_'+side] > 1.), default=0),
            "max_abs_pitch_deg": max((abs(math.degrees(r["pitch"])) for r in rows), default=0),
            "samples": len(rows), "physics_hz": 200,
            "stop_criterion": "0.5 s: axle and wheel surface speeds <0.05 m/s, COM pitch and base roll <10 deg, both wheel normal proxies >1 N; full trial completes",
            "slip_definition": "v_axle_x - radius * world_angular_velocity_y; upright-wheel approximation",
            "normal_force_evidence": "net world-Z link contact force proxy, not ground-only contact force",
            "controller": "fixed-leg implicit PD + COM-state LQR with braking reference; no learned policy",
            "root_pose_prescribed_during_trial": False}
