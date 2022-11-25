#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "connection_node");
  ros::NodeHandle n;
  ros::Publisher conn_pub = n.advertise<std_msgs::String>("connection", 50);

  ros::Rate r(10);

  while(n.ok()){ 

    std_msgs::String tempString;
    tempString.data = "Connect";

    conn_pub.publish(tempString);

    ros::spinOnce();
    r.sleep();
  }
}