"""Pose with explicit quaternion conventions: public xyzw, RDK boundary wxyz."""

import numpy as np
from scipy.spatial.transform import Rotation


class Pose:
    """Position in metres; orientation as xyzw quaternion, xyz Euler radians or 3x3 matrix."""

    def __init__(self, position, orientation):
        p = np.asarray(position, dtype=float)
        o = np.asarray(orientation, dtype=float)
        if p.shape != (3,) or not np.isfinite(p).all() or not np.isfinite(o).all():
            raise ValueError("Pose components must be finite; position must have length 3")
        if o.shape == (4,):
            if np.linalg.norm(o) < 1e-12:
                raise ValueError("Quaternion must be nonzero")
            r = Rotation.from_quat(o)
        elif o.shape == (3,):
            r = Rotation.from_euler("xyz", o)
        elif o.shape == (3, 3):
            if not np.allclose(o.T @ o, np.eye(3), atol=1e-6) or not np.isclose(
                np.linalg.det(o), 1.0, atol=1e-6
            ):
                raise ValueError("Rotation matrix must be orthonormal with determinant +1")
            r = Rotation.from_matrix(o)
        else:
            raise ValueError("orientation must be xyzw quaternion, xyz Euler radians or 3x3 rotation matrix")
        self.position = p.tolist()
        self.x, self.y, self.z = self.position
        self.orientation = r.as_quat().tolist()

    @classmethod
    def from_rdk(cls, pose):
        values = np.asarray(pose, dtype=float)
        if values.shape != (7,):
            raise ValueError("RDK pose must be [x,y,z,qw,qx,qy,qz]")
        return cls(values[:3], values[[4, 5, 6, 3]])

    def to_rdk(self):
        # Revalidate in case a caller has modified the public attributes.
        checked = Pose(self.position, self.orientation)
        x, y, z, w = checked.orientation
        return checked.position + [w, x, y, z]

    def get_position(self):
        return self.position.copy()

    def get_orientation(self, type="quaternion"):
        if type == "quaternion":
            return self.orientation.copy()
        if type == "euler":
            return Rotation.from_quat(self.orientation).as_euler("xyz")
        if type == "rotation_matrix":
            return Rotation.from_quat(self.orientation).as_matrix()
        raise ValueError("type must be quaternion/euler/rotation_matrix")

    def as_matrix(self):
        matrix = np.eye(4)
        matrix[:3, :3] = self.get_orientation("rotation_matrix")
        matrix[:3, 3] = self.position
        return matrix

    def print(self, pose_description="", type="quaternion"):
        print(
            f"{pose_description} position: {self.position}, orientation ({type}): {self.get_orientation(type)}"
        )
