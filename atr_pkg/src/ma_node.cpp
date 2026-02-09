#include<ros/ros.h>
#include<std_msgs/String.h>
#include<qq_msgs/Carry.h>



void chao_callback(qq_msgs::Carry msg)
{
    ROS_WARN(msg.grade.c_str());
    ROS_WARN("%d 星",msg.star);
    ROS_INFO(msg.data.c_str());
    
}

int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");//根据系统环境配置本地化设置
    ros::init(argc,argv,"ma_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<qq_msgs::Carry >("kuai_shang_che_kai_hei_qun",10,chao_callback);


    while (ros::ok())
    {
        ros::spinOnce();
    }
    
    return 0;
}
