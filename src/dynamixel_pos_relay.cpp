/*
  Re-publishes Dynamixel Controller position state data (dynamixel_driver) to a std_msgs/Float64
*/

/*
  Author: Gert Kanter
*/

#include <string.h>

#include <vector>
#include <ros/ros.h>
#include <dynamixel_msgs/JointState.h>
#include <std_msgs/Float64.h>

class Relay
{
  private:
  std::vector<ros::Publisher> publishers;
  std::vector<ros::Subscriber> subscribers;
  public:
  Relay(ros::NodeHandle node)
  {
    std::string topics[18] = {"right_arm_shoulder_roll_joint", "right_arm_shoulder_pitch_joint", "right_arm_shoulder_yaw_joint", "right_arm_elbow_pitch_joint", "right_arm_elbow_yaw_joint", "right_arm_wrist_pitch_joint", "right_arm_wrist_roll_joint", "right_arm_gripper_joint", "right_arm_gripper_joint2", "left_arm_shoulder_roll_joint", "left_arm_shoulder_pitch_joint", "left_arm_shoulder_yaw_joint", "left_arm_elbow_pitch_joint", "left_arm_elbow_yaw_joint", "left_arm_wrist_pitch_joint", "left_arm_wrist_roll_joint", "left_arm_gripper_joint", "left_arm_gripper_joint2"};
    for (int i = 0; i < 18; ++i)
    {
      publishers.push_back(node.advertise<std_msgs::Float64>(topics[i] + "/pos", 1));
      subscribers.push_back(node.subscribe<dynamixel_msgs::JointState>(topics[i] + "/state", 1, boost::bind(&Relay::jointStateMessage, this, _1, i)));
    }
  }
  void jointStateMessage(const dynamixel_msgs::JointStateConstPtr& message, const int id)
  {
    std_msgs::Float64 p;
    p.data = message->current_pos;
    publishers[id].publish(p);
  }
};

void state_relay(){
  ros::NodeHandle node;
  
  Relay relay(node);

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
  ros::init(argc, argv, "dynamixel_pos_relay");
  state_relay();
  return 0;
}
