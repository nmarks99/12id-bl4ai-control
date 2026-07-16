from robotiq_tools import WristCamera
import numpy as np
import tf
from scipy.spatial.transform import RigidTransform, Rotation
import time

def get_tcp_pose():
    pose = [-134.878, -293.816, 290.852, -1.17952, -171.452, 53.8269]
    pose[0] /= 1000.0
    pose[1] /= 1000.0
    pose[2] /= 1000.0
    pose[3] *= np.pi/180.0
    pose[4] *= np.pi/180.0
    pose[5] *= np.pi/180.0
    print(pose)
    return pose

def main():
    cam = WristCamera()
    time.sleep(1)

    # Locate our sample by tag ID
    id = 10
    #  try a few times
    for i in range(5):
        tag = cam.locate_tag(id)
        print(f"[Attempt {i}] ", end="")
        if (not tag):
            print(f"Tag {id} not found")
        else:
            print(f"Tag {id} found")
            break;
    else:
        if not tag:
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
