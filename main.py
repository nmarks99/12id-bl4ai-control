from robotiq_tools import WristCamera
import numpy as np
import tf
from scipy.spatial.transform import RigidTransform, Rotation
import time
from epics import caget

def get_tcp_offset():
    x = caget("bcur:Control:TCPOffset_X")
    y = caget("bcur:Control:TCPOffset_Y")
    z = caget("bcur:Control:TCPOffset_Z")
    rx = caget("bcur:Control:TCPOffset_Rx")
    ry = caget("bcur:Control:TCPOffset_Ry")
    rz = caget("bcur:Control:TCPOffset_Rz")
    offset = [x/1000.0, y/1000.0, z/1000.0, rx, ry, rz]
    print(f"offset = {offset}")
    return offset


def get_tcp_pose():
    pose = caget("bcur:Receive:ActualTCPPose")
    pose[0] /= 1000.0
    pose[1] /= 1000.0
    pose[2] /= 1000.0
    print(f"robot pose = {pose}")
    return pose

def main():
    cam = WristCamera(device_index=0)
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


    # tag in camera frame
    g_ca = RigidTransform.from_matrix(tag["tf"])

    # TCP in world frame
    g_wt = tf.pose_to_tf(get_tcp_pose())

    # TCP in flange frame
    g_ft = tf.pose_to_tf(get_tcp_offset())

    # flange in world frame
    g_wf = g_wt * g_ft.inv()

    # camera in world frame
    g_wc = g_wf * tf.g_fc

    # And finally, tag in world frame
    g_wa = g_wc * g_ca

    tran, rot = g_wa.as_components()

    print("\nTag pose in robot base frame:")
    x,y,z = tran
    roll, pitch, yaw = rot.as_euler("xyz");
    print(f"(x,y,z) = ({x},{y},{z})")
    print(f"(roll,pitch,yaw) = ({roll},{pitch},{yaw})")

if __name__ == "__main__":
    main()
