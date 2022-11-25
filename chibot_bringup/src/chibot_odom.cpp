#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

const double R = 0.0375;
const double L1 = 0.268;
const double L2 = 0.160;

double A = 0.0;
double X = 0.0;
double Y = 0.0;

double FL = 0.0;
double BL = 0.0;
double BR = 0.0;
double FR = 0.0;

double VX = 0.0;
double VY = 0.0;
double VW = 0.0;

double delta_x = 0.0;
double delta_y = 0.0;
double delta_a = 0.0;
  
ros::Time current_time;
ros::Time prev_time;
ros::Duration delta_time;

void cbMessage(const std_msgs::Float64MultiArray &arr) {

  
  current_time = ros::Time::now();

  delta_time = current_time - prev_time;
  double secs = delta_time.toSec();

  FL = -arr.data[0];
  BL = -arr.data[1];
  BR = arr.data[2];
  FR = arr.data[3];

  VX = ( FL + BL + BR + FR) * (R / 4);
  VY = ( FL - BL + BR - FR) * (R / 4);
  VW = ( FL + BL - BR - FR) * (R / (2 * (L1 + L2)));

  delta_x = VX * secs;
  delta_y = VY * secs;
  delta_a = VW * secs;

  A -= delta_a;
  X += (delta_x * cos(A) - delta_y * sin(A));
  Y += (delta_x * sin(A) + delta_y * cos(A));

  prev_time = current_time;
}
  


int main(int argc, char** argv){

  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber sub = n.subscribe("velocity_data", 1000, cbMessage);
  
  tf::TransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped lidar_trans;

  ROS_INFO("Odometry init.");
  ros::Rate r(10);

  while(n.ok()){ 
    
    ros::Time current_time;
    current_time = ros::Time::now();

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(A);

    odom.pose.pose.position.x = X;
    odom.pose.pose.position.y = Y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = VX;
    odom.twist.twist.linear.y = VY;
    odom.twist.twist.angular.z = VW;

    odom_pub.publish(odom);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = VX;
    odom_trans.transform.translation.y = VY;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    
    lidar_trans.header.stamp = current_time;
    lidar_trans.header.frame_id = "base_footprint";
    lidar_trans.child_frame_id = "base_scan";

    lidar_trans.transform.translation.x = -0.14;
    lidar_trans.transform.translation.y = 0;
    lidar_trans.transform.translation.z = 0.12;
    geometry_msgs::Quaternion lidar_quat = tf::createQuaternionMsgFromYaw(0);
    lidar_trans.transform.rotation = lidar_quat;

    broadcaster.sendTransform(lidar_trans);
    broadcaster.sendTransform(odom_trans);
    ros::spinOnce();
    r.sleep();
  }
}
