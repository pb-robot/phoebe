/*
  Feedback gripper angle conversion
*/

/*
  Author: Gert Kanter
*/

#include <string.h>
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

class Relay
{
  private:
  ros::NodeHandle node_;
  ros::Subscriber feedback_subscriber_;
  ros::Subscriber cmd_subscriber_;
  ros::Publisher feedback_publisher_;
  ros::Publisher cmd_publisher_;
  double r_;
  double l_;
  double d_;

  public:
  Relay(ros::NodeHandle node, std::string feedback_pub, std::string cmd_pub, std::string feedback_sub, std::string cmd_sub, double r, double l, double d)
  : node_(node)
  , feedback_publisher_(node.advertise<std_msgs::Float64>(feedback_pub, 1))
  , cmd_publisher_(node.advertise<std_msgs::Float64>(cmd_pub, 1))
  , feedback_subscriber_(node.subscribe<std_msgs::Float64>(feedback_sub, 10, &Relay::feedbackMessage, this))
  , cmd_subscriber_(node.subscribe<std_msgs::Float64>(cmd_sub, 10, &Relay::cmdMessage, this))
  , r_(r)
  , l_(l)
  , d_(d)
  {
  } 
  void cmdMessage(const std_msgs::Float64ConstPtr& cmd)
  {
    std_msgs::Float64 p;

    p.data = r_*cos(cmd->data) + sqrt(pow(l_, 2) - (d_ + r_*pow(sin(cmd->data), 2)));

    cmd_publisher_.publish(p);

  }
  void feedbackMessage(const std_msgs::Float64ConstPtr& message)
  {
    //message.data
    double gamma = pow(l_, 2) - pow(d_, 2) - pow(message->data, 2) - pow(r_, 2);
    ROS_INFO("gamma = %f", gamma);
    /*sq = ((((4*self.r**2*(self.d**2 + x**2)) - gamma**2)))
    sqr = sq if sq >= 0 else 0.0
    w_1 = math.asin(((-self.d*gamma) - ((x*math.sqrt(sqr)))) / (2*self.r * (self.d**2 + x**2)))
    t_1 = self.findTranslation(-w_1)
    if t_1 > x - 0.01 and t_1 < x + 0.01:
      return(-w_1)
    else:
      return(w_1 - math.pi)*/
  }

};

void state_relay()
{
  ros::NodeHandle pnode("~");
  
  std::string feedback_subscriber;
  std::string cmd_subscriber;
  std::string feedback_publisher;
  std::string cmd_publisher;
  double r;
  double l;
  double d;
  pnode.param("feedback_pub", feedback_publisher, std::string("feedback_pub"));
  pnode.param("cmd_pub", cmd_publisher, std::string("cmd_pub"));
  pnode.param("feedback_sub", feedback_subscriber, std::string("feedback_sub"));
  pnode.param("cmd_sub", cmd_subscriber, std::string("cmd_sub"));
  pnode.param("r", r, double(0));
  if (r == 0)
    ROS_ERROR("r (radius) is uninitialized!");
  pnode.param("l", l, double(0));
  if (l == 0)
    ROS_ERROR("l (length) is uninitialized!");
  pnode.param("d", d, double(0));
  if (d == 0)
    ROS_ERROR("d (offset) is uninitialized!");
  if (r == 0 || l == 0 || d == 0)
    ros::shutdown();

  Relay relay(pnode, feedback_publisher, cmd_publisher, feedback_subscriber, cmd_subscriber, r, l, d);
  
  ROS_INFO("Gripper angle converter initialized...");
  ros::Rate rate(20);
  
  while(ros::ok())
  {
    ros::spinOnce();
    
    rate.sleep();
  }
}

int main(int argc, char** argv)
{ 
  ros::init(argc, argv, "gripperangle_converter");
  state_relay();
  return 0;
}
