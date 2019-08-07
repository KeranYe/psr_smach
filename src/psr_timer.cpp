#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "timer_test");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1);



  ros::Rate loop_rate(1);

  int count = 0;  
  
  double initial_time = ros::WallTime::now().toSec();
  
  while (ros::ok())
  {
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    double current_time = ros::WallTime::now().toSec();

    double elapsed_time = current_time - initial_time;

    ROS_INFO("%f", elapsed_time);

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
