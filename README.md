# 12-ID BL4AI Control

## Notes on Robotiq wrist camera and gripper
This code assumes the wrist camera + gripper is connected directly to a computer where you will run
this code via USB. The camera will show up as a standard USB camera (/dev/video*).
When the camera is connected to the robot, there is almost no control made available remotely, which
is a problem because I find the autofocus to be very unreliable and there is no way to turn it off.

When the wrist camera and gripper are connected directly to a PC like this, we must also control
the gripper over USB. This hasn't been implemented yet but it is very easy to do with [pyRobotiqGripper](https://github.com/castetsb/pyRobotiqGripper).
