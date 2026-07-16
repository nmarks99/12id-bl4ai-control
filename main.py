from robotiq_tools import WristCamera
import numpy as np
import tf
from scipy.spatial.transform import RigidTransform, Rotation

def get_tcp_pose():
    return [-0.128, -0.256, 0.171, 0.0, 0.0, 0.0]

def main():
    cam = WristCamera(device_index=1)

    # Locate our sample by tag ID
    id = 22
    tag = cam.locate_tag(id)
    if (not tag):
        print(f"Tag {id} not found")
        return

    g_ca = RigidTransform.from_matrix(tag["tf"])
    g_wf = tf.pose_to_tf(get_tcp_pose())
    g_wc = g_wf * tf.g_fc

    g_wa = g_wc * g_ca

    #  print(f"Tag in camera frame:\n{g_ca}")
    print(f"\nTag in world frame:\n{g_wa}")

    tran, rot = g_wa.as_components()

    x,y,z = tran
    roll, pitch, yaw = rot.as_euler("xyz", degrees=True);
    print(f"(x,y,z) = ({x},{y},{z})")
    print(f"(roll,pitch,yaw) = ({roll},{pitch},{yaw})")

if __name__ == "__main__":
    main()
