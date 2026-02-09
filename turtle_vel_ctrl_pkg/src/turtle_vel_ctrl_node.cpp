#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "chao_node");

    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    ros::Rate r(10);

    geometry_msgs::Twist vel_msgs;

    vel_msgs.linear.x = 3;
    vel_msgs.linear.y = 0;
    vel_msgs.linear.z = 0;

    vel_msgs.angular.x = 0;
    vel_msgs.angular.y = 0;
    vel_msgs.angular.z = -1;


    while(ros::ok())
    { 
        vel_pub.publish(vel_msgs);
        r.sleep();
    }

  return 0;
}