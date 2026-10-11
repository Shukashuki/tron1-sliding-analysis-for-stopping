"""Simulator-independent wheel controllers. All torques are wheel-shaft Nm."""
from dataclasses import dataclass
from typing import Protocol
import numpy as np
from balance_lqr import design_lqr


@dataclass(frozen=True)
class Observation:
    x: float
    velocity: float
    pitch: float
    pitch_rate: float


@dataclass(frozen=True)
class Reference:
    x: float
    velocity: float
    acceleration: float


class WheelController(Protocol):
    def reset(self) -> None: ...
    def step(self, observation: Observation, reference: Reference) -> np.ndarray: ...
    def metadata(self) -> dict: ...


CONTROLLERS = ("lqr_tracking", "lqr_no_position")


class LQRController:
    def __init__(self, parameters, dt, name="lqr_tracking"):
        if name not in CONTROLLERS:
            raise ValueError(f"Unknown controller: {name}")
        self.p, self.dt, self.name = parameters, dt, name
        self.design = design_lqr(parameters, dt, [20., 10., 500., 20.], .1)
        self.gain = self.design.K.copy()
        if name == "lqr_no_position":
            # Deliberate ablation, NOT a newly optimized velocity-only LQR.
            self.gain[0, 0] = 0.

    def reset(self):
        pass  # Stateless; this lifecycle hook also supports future dynamic controllers.

    def step(self, observation, reference):
        p = self.p
        lean = (p.coupling + p.effective_mass*p.radius)/(p.coupling*p.gravity)*reference.acceleration
        measured = np.array([observation.x, observation.velocity, observation.pitch, observation.pitch_rate])
        target = np.array([reference.x, reference.velocity, lean, 0.])
        total = -float((self.gain @ (measured-target)).item()) + p.effective_mass*p.radius*reference.acceleration
        return np.full(2, total / 2.)  # [left, right], before shared actuator model

    def metadata(self):
        return {"name": self.name, "gain": self.gain.tolist(), "dt": self.dt,
                "design_Q": [20., 10., 500., 20.], "design_R": .1,
                "position_gain_disabled": self.name == "lqr_no_position",
                "output": "requested left/right wheel torque in Nm before shared actuator mapping"}


def create_controller(name, parameters, dt) -> WheelController:
    return LQRController(parameters, dt, name)


def torque_to_command(requested):
    """Fixed 12 Nm/unit nominal mapping preserves the original baseline exactly.

    Configurable torque_conversion remains downstream as actuator uncertainty.
    """
    requested = np.asarray(requested, dtype=float)
    if requested.shape != (2,) or not np.all(np.isfinite(requested)):
        raise ValueError("Controller must return two finite wheel torques")
    return np.clip(requested / 12., -1., 1.)
