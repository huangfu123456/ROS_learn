#include <ros/ros.h>
#include <std_msgs/String.h>
#include<qq_msgs/Carry.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "chao_node");

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<qq_msgs::Carry>("kuai_shang_che_kai_hei_qun", 10);

  ros::Rate r(10);

  while(ros::ok())
  { 

    printf("我要开始刷屏了\n");

    qq_msgs::Carry msg;
   
    msg.grade = "王者";
    msg.star = 56;
     msg.data = "国服马超，带飞";
     
    pub.publish(msg);
    r.sleep();
  }

  return 0;
}