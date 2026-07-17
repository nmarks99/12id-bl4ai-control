"""
Tests AprilTag detection with Robotiq wrist camera and finding
the transformation between the robot's base frame and AprilTag.
"""

from tag_camera import TagCamera
import numpy as np
from scipy.spatial.transform import RigidTransform, Rotation
import time
from ur_robot import UR

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
# is this correct?
g_fc = pose_to_tf([0.0, 0.0433, 0.015, -np.pi/6, 0.0, 0.0])

def get_tag_in_robot_frame(tf_cam_to_tag, robot):
    """ Gets the transform from robot base frame to AprilTag

    Finds the transformation between the robot's base frame and
    the tag given the transformation between the camera and the tag

    Args:
        tf_cam_to_tag: 4x4 transformation matrix from camera to tag frame
        robot: UR ophyd device instance

    Returns:
        Transformation between robot base frame and tag as a RigidTransform
    """
    # TCP in world frame
    tcp_pose = robot.pose.readback.get()
    tcp_pose[:3] /= 1000.0 # convert mm->m
    g_wt = pose_to_tf(tcp_pose)

    # TCP in flange frame, note convert mm->m
    tcp_offset = [
        robot.tcp_offset.x.get() / 1000.0,
        robot.tcp_offset.y.get() / 1000.0,
        robot.tcp_offset.z.get() / 1000.0,
        robot.tcp_offset.rx.get(),
        robot.tcp_offset.ry.get(),
        robot.tcp_offset.rz.get()
    ]
    g_ft = pose_to_tf(tcp_offset)

    # flange in world frame
    g_wf = g_wt * g_ft.inv()

    # camera in world frame
    g_wc = g_wf * g_fc

    # And finally, tag in world frame
    g_ca = RigidTransform.from_matrix(tf_cam_to_tag)
    g_wa = g_wc * g_ca

    return g_wa


def main():

    # Connect to robot using ophyd device
    robot = UR(PREFIX, name="robot")
    robot.wait_for_connection()

    # Connect the wrist camera. Look at /dev/video*
    # to find what device_index should be
    cam = TagCamera(device_index=0)

    # Locate our sample by tag ID
    id = 10

    #  try a few times in case we don't find it on the first go
    for i in range(5):
        tag = cam.locate(id)
        if (not tag):
            print(f"Tag {id} not found. Trying again[{i+1}]")
        else:
            break;
    else:
        if not tag:
            return

    # Get the tag in robot's base frame
    # tag["tf"] is a 4x4 transformation matrix between camera origin and AprilTag center
    g_wa = get_tag_in_robot_frame(tag["tf"], robot)

    tran, rot = g_wa.as_components()
    x,y,z = tran * 1000.0
    roll, pitch, yaw = rot.as_euler("xyz");
    print("\nTag pose in robot base frame:")
    print(f"         (x, y, z) = ({x:+10.4f}, {y:+10.4f}, {z:+10.4f}) mm")
    print(f"(roll, pitch, yaw) = ({roll:+10.4f}, {pitch:+10.4f}, {yaw:+10.4f}) rad")

if __name__ == "__main__":
    main()
