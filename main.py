from robotiq_tools import WristCamera
import numpy as np
from scipy.spatial.transform import RigidTransform, Rotation
import time
from epics import caget

# IOC prefix
PREFIX = "bcur:"

def pose_to_tf(pose):
    """
    Creates a scipy RigidTransform from a 3D pose (6-vector)
    """
    return RigidTransform.from_components(
        pose[:3],
        Rotation.from_rotvec(pose[3:])
    )

# Transform from robot flange to camera
g_fc = pose_to_tf([0.0, 0.0433, 0.015, -np.pi/6, 0.0, 0.0])

def get_tcp_pose():
    """
    Gets the current TCP pose from EPICS. Must convert mm->m
    """
    pose = caget(f"{PREFIX}Receive:ActualTCPPose")
    pose[0] /= 1000.0
    pose[1] /= 1000.0
    pose[2] /= 1000.0
    return pose

def get_tcp_offset():
    """
    Gets the TCP offset from EPICS. Must convert mm->m
    """
    x = caget(f"{PREFIX}Control:TCPOffset_X")
    y = caget(f"{PREFIX}Control:TCPOffset_Y")
    z = caget(f"{PREFIX}Control:TCPOffset_Z")
    rx = caget(f"{PREFIX}Control:TCPOffset_Rx")
    ry = caget(f"{PREFIX}Control:TCPOffset_Ry")
    rz = caget(f"{PREFIX}Control:TCPOffset_Rz")
    offset = [x/1000.0, y/1000.0, z/1000.0, rx, ry, rz]
    return offset


def camera_to_robot_frame(g_ca):
    """
    Finds the transformation between the robot's base frame and
    the tag given the transformation between the camera and the tag
    """
    # TCP in world frame
    g_wt = pose_to_tf(get_tcp_pose())

    # TCP in flange frame
    g_ft = pose_to_tf(get_tcp_offset())

    # flange in world frame
    g_wf = g_wt * g_ft.inv()

    # camera in world frame
    g_wc = g_wf * g_fc

    # And finally, tag in world frame
    g_wa = g_wc * g_ca

    return g_wa


def main():

    # Connect the wrist camera. Look at /dev/video*
    # to find what device_index should be
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
    g_wa = camera_to_robot_frame(g_ca)

    tran, rot = g_wa.as_components()
    x,y,z = tran * 1000.0
    roll, pitch, yaw = rot.as_euler("xyz");
    print("\nTag pose in robot base frame:")
    print(f"         (x, y, z) = ({x:+10.4f}, {y:+10.4f}, {z:+10.4f}) mm")
    print(f"(roll, pitch, yaw) = ({roll:+10.4f}, {pitch:+10.4f}, {yaw:+10.4f}) rad")

if __name__ == "__main__":
    main()
