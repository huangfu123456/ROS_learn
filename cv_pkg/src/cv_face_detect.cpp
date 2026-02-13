#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>     //格式转换工具cv_bridge的头文件
#include<sensor_msgs/image_encodings.h>//图像格式编码的头文件
#include<opencv2/imgproc/imgproc.hpp>//opencv图像处理函数头文件
#include<opencv2/highgui/highgui.hpp>//opencv界面显示函数头文件
#include<opencv2/objdetect/objdetect.hpp>//opencv的检测函数头文件


using namespace cv;
using namespace std;

static CascadeClassifier face_cascade;  /*定义一个CascadeClassifier分类器，
                                        用于目标检测的级联分类器类。*/

static Mat frame_gray;      //用来存储黑白图像
static vector<Rect> faces;  //用来存放人脸检测结果的数组
static vector<Rect>::const_iterator face_iter;//定义和faces数组配套的迭代器


void callbackRGB(const sensor_msgs::Image msg)
{   


    //把图像从ros的image格式转换为cv的mat格式
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat imgOriginal = cv_ptr->image;


    // 转换成黑白图像
    cvtColor( imgOriginal, frame_gray, CV_BGR2GRAY );
	equalizeHist( frame_gray, frame_gray ); //对黑白图片的灰度进行均衡化

    // 检测人脸
	face_cascade.detectMultiScale( frame_gray, faces, 1.1, 9, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );


    // 在彩色原图中标注人脸位置
    if(faces.size() > 0)
    {
        std::vector<cv::Rect>::const_iterator i;
        for (face_iter = faces.begin(); face_iter != faces.end(); ++face_iter) 
        {
            cv::rectangle(
                imgOriginal,
                cv::Point(face_iter->x , face_iter->y),
                cv::Point(face_iter->x + face_iter->width, face_iter->y + face_iter->height),
                CV_RGB(255, 0 , 255),
                2);
        }
    }
    imshow("faces", imgOriginal);
    waitKey(1);







}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo_cv_face_detect");
    namedWindow("faces");

    std::string strLoadFile;
    char const* home = getenv("HOME");
    strLoadFile = home;
    strLoadFile += "/catkin_ws";
    strLoadFile += "/src/wpr_simulation/config/haarcascade_frontalface_alt.xml";

    bool res = face_cascade.load(strLoadFile);
	if (res == false)
	{
		ROS_ERROR("fail to load haarcascade_frontalface_alt.xml");
        return 0;
	}
    ros::NodeHandle nh;
    ros::Subscriber rgb_sub = nh.subscribe("/kinect2/qhd/image_color_rect", 1 , callbackRGB);

    ros::spin();
    return 0;
}



