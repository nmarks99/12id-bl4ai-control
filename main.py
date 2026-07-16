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
    return offset


def get_tcp_pose():
    pose = caget("bcur:Receive:ActualTCPPose")
    pose[0] /= 1000.0
    pose[1] /= 1000.0
    pose[2] /= 1000.0
    return pose

def to_robot_frame(g_ca):
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

    return g_wa


def main():
    cam = WristCamera(device_index=0)

    # Locate our sample by tag ID
    id = 10
    #  try a few times in case we don't find it on the first go
    for i in range(5):
        tag = cam.locate_tag(id)
        if (not tag):
            print(f"Tag {id} not found. Trying again[{i+1}]")
        else:
            break;
    else:
        if not tag:
            return


    # tag in camera frame
    g_ca = RigidTransform.from_matrix(tag["tf"])

    # Get the tag in robot's base frame
    g_wa = to_robot_frame(g_ca)

    tran, rot = g_wa.as_components()
    x,y,z = tran * 1000.0
    roll, pitch, yaw = rot.as_euler("xyz");
    print("\nTag pose in robot base frame:")
    print(f"         (x, y, z) = ({x:+10.4f}, {y:+10.4f}, {z:+10.4f}) mm")
    print(f"(roll, pitch, yaw) = ({roll:+10.4f}, {pitch:+10.4f}, {yaw:+10.4f}) rad")

if __name__ == "__main__":
    main()
