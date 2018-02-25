#!/bin/bash

rostopic pub -1 /right_arm_shoulder_roll_joint/command std_msgs/Float64 -- 0.0 &
rostopic pub -1 /right_arm_shoulder_pitch_joint/command std_msgs/Float64 -- 0.0 &
rostopic pub -1 /right_arm_shoulder_yaw_joint/command std_msgs/Float64 -- 0.0 &
rostopic pub -1 /right_arm_elbow_pitch_joint/command std_msgs/Float64 -- 0.0 &
rostopic pub -1 /right_arm_elbow_yaw_joint/command std_msgs/Float64 -- 0.0 &
rostopic pub -1 /right_arm_wrist_pitch_joint/command std_msgs/Float64 -- 0.0 &
rostopic pub -1 /right_arm_wrist_roll_joint/command std_msgs/Float64 -- 0.0 &

rostopic pub -1 /left_arm_shoulder_roll_joint/command std_msgs/Float64 -- 0.0 &
rostopic pub -1 /left_arm_shoulder_pitch_joint/command std_msgs/Float64 -- 0.0 &
rostopic pub -1 /left_arm_shoulder_yaw_joint/command std_msgs/Float64 -- 0.0 &
rostopic pub -1 /left_arm_elbow_pitch_joint/command std_msgs/Float64 -- 0.0 &
rostopic pub -1 /left_arm_elbow_yaw_joint/command std_msgs/Float64 -- 0.0 &
rostopic pub -1 /left_arm_wrist_pitch_joint/command std_msgs/Float64 -- 0.0 &
rostopic pub -1 /left_arm_wrist_roll_joint/command std_msgs/Float64 -- 0.0
