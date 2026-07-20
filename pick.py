from tag_camera import TagCamera
import numpy as np
from scipy.spatial.transform import RigidTransform, Rotation
import time
from ur_robot import UR
from pathlib import Path
import pyrobotiqgripper as rq

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
# Taken from Byeongdu's code. I think it is correct?
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

    # hacky fix to computed Z position in tag frame
    Tz = RigidTransform.from_translation([0.0, 0.0, -0.015])
    g_wa = Tz * g_wa

    return g_wa


wpj = {
    "HOME" : [-90.0, -75.0, -100.0, -90.0, 90.0, 0.0],
    "VIEW" : [-87.0, -70.0, -120.0, -49.0, 88.0, 0.0]
}


def main():

    # Connect to robot using ophyd device
    robot = UR(PREFIX, name="robot")
    robot.wait_for_connection()

    # TODO: Need to be in dialout group
    #  gripper = rq.RobotiqGripper("/dev/ttyACM0")
    #  if (not gripper.isActivated()):
        #  gripper.activate()
    #  gripper.open()

    # Connect the wrist camera. Look at /dev/video*
    # to find what device_index should be
    cam = TagCamera(device_index=0)

    # Move to a safe "home" position first
    robot.joints.set(wpj["HOME"]).wait()

    # Move to position where the camera can see the tag
    robot.joints.set(wpj["VIEW"]).wait()

    # Locate tag by tag ID
    id = 10
    #  try a few times in case we don't find it on the first attempt
    MAX_RETRY = 10
    for i in range(MAX_RETRY):
        tag = cam.locate(id)
        if (not tag):
            print(f"Tag {id} not found. Trying again[{i}]")
            time.sleep(0.1)
        else:
            break;
    else:
        if not tag:
            print(f"Tag {id} not found after {MAX_RETRY} attempts")
            return

    # Get the tag in robot's base frame
    # tag["tf"] is a 4x4 transformation matrix between camera origin and AprilTag center
    g_wa = get_tag_in_robot_frame(tag["tf"], robot)
    tran, rot = g_wa.as_components()
    x,y,z = tran * 1000.0
    rx, ry, rz = rot.as_rotvec();
    print("\nTag pose in robot base frame:")
    print(f"   (x, y, z) = ({x:+10.4f}, {y:+10.4f}, {z:+10.4f}) mm")
    print(f"(rx, ry, rz) = ({rx:+10.4f}, {ry:+10.4f}, {rz:+10.4f}) rad")
    print("-----------------------------------------------------------------------\n")
    time.sleep(0.2)

    # Static transform from tag frame to pick point
    g_ap_t = np.array([0.050, 0.0, 0.0])
    g_ap_R = Rotation.from_euler("z", -90.0, degrees=True);
    g_ap = RigidTransform.from_components(g_ap_t, g_ap_R)
    g_wp = g_wa * g_ap
    tran, rot = g_wp.as_components()
    x, y, z = tran * 1000.0
    rx, ry, rz = rot.as_rotvec();

    # Move to pick point standoff
    robot.pose.set([x,y,z+20,rx,ry,rz]).wait()

    # Pick up the sample
    robot.pose.set([x,y,z-10,rx,ry,rz]).wait()
    # TODO: use gripper
    time.sleep(1.0)
    robot.pose.set([x,y,z+50,rx,ry,rz]).wait()

    # Go home
    robot.joints.set(wpj["HOME"]).wait()

if __name__ == "__main__":
    main()
