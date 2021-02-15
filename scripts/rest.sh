#!/bin/bash

rostopic pub -1 /right_arm_shoulder_roll_joint/command std_msgs/Float64 -- 1.6 & # -1.5708
rostopic pub -1 /right_arm_shoulder_pitch_joint/command std_msgs/Float64 -- 1.65 & # 1.5708
rostopic pub -1 /right_arm_shoulder_yaw_joint/command std_msgs/Float64 -- 0.05 & # 0.0
rostopic pub -1 /right_arm_elbow_pitch_joint/command std_msgs/Float64 -- 0.05 & # 0.0
rostopic pub -1 /right_arm_elbow_yaw_joint/command std_msgs/Float64 -- -0.11 & # 0.0
rostopic pub -1 /right_arm_wrist_pitch_joint/command std_msgs/Float64 -- 0.0 &
rostopic pub -1 /right_arm_wrist_roll_joint/command std_msgs/Float64 -- 0.0 &

rostopic pub -1 /left_arm_shoulder_roll_joint/command std_msgs/Float64 -- -1.6 & # 1.5708
rostopic pub -1 /left_arm_shoulder_pitch_joint/command std_msgs/Float64 -- 1.66 & # 1.5708
rostopic pub -1 /left_arm_shoulder_yaw_joint/command std_msgs/Float64 -- -0.09 & # 0.0
rostopic pub -1 /left_arm_elbow_pitch_joint/command std_msgs/Float64 -- 0.05 & # 0.0
rostopic pub -1 /left_arm_elbow_yaw_joint/command std_msgs/Float64 -- -0.11 & # 0.0
rostopic pub -1 /left_arm_wrist_pitch_joint/command std_msgs/Float64 -- 0.0 &
rostopic pub -1 /left_arm_wrist_roll_joint/command std_msgs/Float64 -- 0.0
