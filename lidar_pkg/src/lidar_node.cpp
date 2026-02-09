#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>

ros::Publisher vel_pub;     
int nCount = 0;
void lidarcallback(const sensor_msgs::LaserScan Laser_msg)
{
    float FmidDist = Laser_msg.ranges[180];
    
    ROS_INFO("前方测距：%f",FmidDist);

    geometry_msgs::Twist vel_msg;

    if (nCount>0)
    {   
        nCount--;
        return ;
    }
    

    if (FmidDist<1.5)
    {
        vel_msg.angular.z = 0.3;
        nCount = 50;
    } else{

        vel_msg.linear.x = 0.1;
    }

    vel_pub.publish(vel_msg);

}

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"lidar_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan",10,lidarcallback);

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    while (ros::ok())
    {   


        ros::spinOnce();
    }
    
    return 0;
}



