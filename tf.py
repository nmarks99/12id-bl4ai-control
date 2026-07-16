import numpy as np
from scipy.spatial.transform import RigidTransform, Rotation
from math import pi

# Example TCP pose in the "home" position:
# [-0.128, -0.256, 0.171, 0.0, 0.0, 0.0] # meters, radians

# Camera pose in flange frame measured as approximately:
# [0.0, 0.0433, 0.015, -pi/6, 0.0, 0.0]
#  _tf = lambda pose: RigidTransform.from_components(pose[:3], Rotation.from_euler('xyz', pose[3:]))
def pose_to_tf(pose):
    return RigidTransform.from_components(
        pose[:3],
        #  Rotation.from_euler('XYZ', pose[3:])
        Rotation.from_rotvec(pose[3:])
    )
#  g_wf = _tf([-0.128, -0.256, 0.171+0.150, 0.0, 0.0, 0.0])
g_fc = pose_to_tf([0.0, 0.0433, 0.015, -pi/6, 0.0, 0.0])

#  g_wc = g_wf * g_fc
# TODO: get g_ca from opencv code
#  g_ca = RigidTransform.identity()
#  g_wa = g_wc * g_ca

#  if __name__ == "__main__":
    #  print(f"Camera in the world frame:\n{g_wc}")
    #  print(f"\nAprilTag in the world frame:\n{g_wc}")
