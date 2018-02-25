/*
  Publishes Robot Joint Publisher to Dynamixel Controllers
*/

/*
  Author: Gert Kanter
*/

#include <string.h>

#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

class Relay
{
  private:
  std::vector<ros::Publisher> publishers;
  public:
  Relay(ros::NodeHandle node)
  {
    std::string topics[20] = {"wheel_left_joint", "wheel_right_joint", "right_arm_shoulder_roll_joint", "right_arm_shoulder_pitch_joint", "right_arm_shoulder_yaw_joint", "right_arm_elbow_pitch_joint", "right_arm_elbow_yaw_joint", "right_arm_wrist_pitch_joint", "right_arm_wrist_roll_joint", "right_arm_gripper_joint", "right_arm_gripper_joint2", "left_arm_shoulder_roll_joint", "left_arm_shoulder_pitch_joint", "left_arm_shoulder_yaw_joint", "left_arm_elbow_pitch_joint", "left_arm_elbow_yaw_joint", "left_arm_wrist_pitch_joint", "left_arm_wrist_roll_joint", "left_arm_gripper_joint", "left_arm_gripper_joint2"};
    for (int i = 0; i < 20; ++i)
      publishers.push_back(node.advertise<std_msgs::Float64>(topics[i] + "/command", 1));

  }
  void stateRelay(const sensor_msgs::JointStateConstPtr& states)
  {
    std_msgs::Float64 p;
    for (int i = 0; i < states->position.size(); ++i)
    {
      p.data = states->position[i];
      publishers[i].publish(p);
    }
    ROS_INFO("Relayed...");
  }
};

void state_relay(){
  ros::NodeHandle node;
  
  Relay relay(node);
  ros::Subscriber joint_states = node.subscribe<sensor_msgs::JointState>("/states_relay", 1, &Relay::stateRelay, &relay);

  ROS_INFO("Initialized...");
  ros::Rate r(10); //an object to maintain specific frequency of a control loop - 1hz
  
  
  while(ros::ok())
  {
    ros::spinOnce();
    
    r.sleep();
  }
}

int main(int argc, char** argv)
{ 
  ros::init(argc, argv, "state_relay");
  state_relay();
  return 0;
}
