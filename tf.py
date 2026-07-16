import numpy as np
from scipy.spatial.transform import RigidTransform, Rotation
from math import pi

def pose_to_tf(pose):
    return RigidTransform.from_components(
        pose[:3],
        #  Rotation.from_euler('XYZ', pose[3:])
        Rotation.from_rotvec(pose[3:])
    )

g_fc = pose_to_tf([0.0, 0.0433, 0.015, -pi/6, 0.0, 0.0])
