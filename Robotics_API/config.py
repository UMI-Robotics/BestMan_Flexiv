"""Validated Rizon 4 defaults. Controller limits/nominal gains remain authoritative."""

import json
import math
from dataclasses import dataclass
from pathlib import Path


def vector(values, size, name, *, lower=None, upper=None, strict=False, allow_inf=False):
    try:
        result = [float(v) for v in values]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain {size} numbers") from exc
    if len(result) != size or any(math.isnan(v) or (not allow_inf and not math.isfinite(v)) for v in result):
        raise ValueError(f"{name} must contain {size} {'non-NaN' if allow_inf else 'finite'} numbers")
    if lower is not None and any(v <= lower if strict else v < lower for v in result):
        raise ValueError(f"{name} contains a value below its allowed minimum")
    if upper is not None and any(v > upper for v in result):
        raise ValueError(f"{name} contains a value above its allowed maximum")
    return result


def positive(value, name):
    return vector([value], 1, name, lower=0, strict=True)[0]


@dataclass(frozen=True)
class RobotConfig:
    model: str = "Rizon4"
    frequency: float = 20.0
    joint_velocity_scale: float = 0.1
    joint_max_acc: float = 0.5
    max_linear_vel: float = 0.05
    max_angular_vel: float = 0.3
    max_linear_acc: float = 0.2
    max_angular_acc: float = 0.5
    force_axis_max_linear_velocity: tuple = (0.02, 0.02, 0.02)
    max_contact_wrench: tuple = (10.0, 10.0, 10.0, 3.0, 3.0, 3.0)
    # Offline FK/IK only. Replace with the actual active tool transform (RDK wxyz).
    flange_to_tcp: tuple = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)

    def __post_init__(self):
        if self.model != "Rizon4":
            raise ValueError("Only Rizon4 is configured")
        if not 1 <= positive(self.frequency, "frequency") <= 100:
            raise ValueError("frequency must be in [1, 100]")
        if positive(self.joint_velocity_scale, "joint_velocity_scale") > 1:
            raise ValueError("joint_velocity_scale must be <= 1")
        for name in (
            "joint_max_acc",
            "max_linear_vel",
            "max_angular_vel",
            "max_linear_acc",
            "max_angular_acc",
        ):
            positive(getattr(self, name), name)
        for name, size, limits in (
            ("force_axis_max_linear_velocity", 3, dict(lower=0.005, upper=2.0)),
            ("max_contact_wrench", 6, dict(lower=0)),
            ("flange_to_tcp", 7, {}),
        ):
            object.__setattr__(self, name, tuple(vector(getattr(self, name), size, name, **limits)))
        from .Pose import Pose

        Pose.from_rdk(self.flange_to_tcp)

    @classmethod
    def load(cls, path=None):
        if path is None:
            return cls()
        with Path(path).open(encoding="utf-8") as stream:
            return cls(**json.load(stream))
