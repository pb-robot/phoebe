/*
  Publishes Robot Joint States to TF
*/

/*
  Author: Gert Kanter
*/

#include <string.h>

#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <robot_state_publisher/robot_state_publisher.h>

#include <kdl_parser/kdl_parser.hpp>

#include <std_msgs/Float64MultiArray.h>

#include <phoebe/JointMap.h>

class Relay
{
  private:
  ros::NodeHandle node_;
  std::map<std::string, ros::Subscriber> feedback_subscribers;
  std::map<std::string, ros::Subscriber> calibration_subscribers;
  std::map<std::string, double> joint_positions;
  robot_state_publisher::RobotStatePublisher publisher;
  ros::Subscriber jointmap_subscriber;

  public:
  Relay(ros::NodeHandle node, ros::NodeHandle pnode, KDL::Tree tree)
  : node_(node)
  , publisher(tree)
  , jointmap_subscriber(node.subscribe<phoebe::JointMap>("jointmap", 10, &Relay::jointmapMessage, this))
  {
  } 
  void jointmapMessage(const phoebe::JointMapConstPtr& message)
  {
    if (message->feedback_joints.size() == message->tf_joints.size())
    {
      std::vector<std_msgs::String>::const_iterator fbit = message->feedback_joints.begin();
      for (std::vector<std_msgs::String>::const_iterator it = message->tf_joints.begin(); it != message->tf_joints.end(); ++it)
      {
        std::map<std::string, ros::Subscriber>::iterator sit(feedback_subscribers.find(it->data));
        if (sit != feedback_subscribers.end())
        {
          // key found, update
          if (feedback_subscribers[it->data].getTopic() != fbit->data)
          {
            // changed topic
            feedback_subscribers[it->data].shutdown();
            feedback_subscribers[it->data] = node_.subscribe<std_msgs::Float64>(fbit->data, 10, boost::bind(&Relay::stateRelay, this, _1, it->data));
          }
        }
        else
        {
          // new key
          ROS_INFO("Registered '%s' with feedback from '%s'", it->data.c_str(), fbit->data.c_str());
          feedback_subscribers.insert(std::pair<std::string, ros::Subscriber>(it->data, node_.subscribe<std_msgs::Float64>(fbit->data, 10, boost::bind(&Relay::stateRelay, this, _1, it->data))));
          joint_positions.insert(std::pair<std::string, double>(it->data, 0.0));
        }
        ++fbit;
      }
    }
    else
      ROS_ERROR("Incorrect jointmap message!");
  }
  void stateRelay(const std_msgs::Float64ConstPtr& message, std::string topic)
  {
    std::map<std::string, double>::iterator it(joint_positions.find(topic));
    if (it != joint_positions.end())
    {
      joint_positions[topic] = message->data;
    }
  }
  void publish()
  {
    publisher.publishTransforms(joint_positions, ros::Time(ros::Time::now() + ros::Duration(0.1)), "");
    publisher.publishFixedTransforms("");
  }

};

void state_relay()
{
  ros::NodeHandle node;
  ros::NodeHandle pnode("~");
  
  KDL::Tree tree;
  std::string robot_desc_string;
  node.param("robot_description", robot_desc_string, std::string());
  if (!kdl_parser::treeFromString(robot_desc_string, tree))
  {
    ROS_ERROR("Failed to construct KDL tree!");
    ros::shutdown();
  }
  Relay relay(node, pnode, tree);
  
  ROS_INFO("Initialized...");
  ros::Rate r(20); //an object to maintain specific frequency of a control loop - 1hz
  
  while(ros::ok())
  {
    relay.publish(); // Publish the fresh joint values

    ros::spinOnce();
    
    r.sleep();
  }
}

int main(int argc, char** argv)
{ 
  ros::init(argc, argv, "jointstate_publisher");
  state_relay();
  return 0;
}
