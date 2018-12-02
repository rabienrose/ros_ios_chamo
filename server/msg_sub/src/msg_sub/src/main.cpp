#include <string>
#include <iostream>
#include <ros/ros.h>
#include <ros/topic.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

using namespace std;

int last_img_seq=-1;
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    
}

void imgCallback(const sensor_msgs::CompressedImage::ConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    if(last_img_seq>0){
        if (msg->header.seq-last_img_seq>1){
            std::cout<<"lose "<<msg->header.seq-last_img_seq<<"images"<<std::endl;
        }
    }
    last_img_seq= msg->header.seq;
    cv::imshow("chamo",cv_ptr->image);
    cv::waitKey(10);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "msg_sub");
    string imuTopic = "imu";
    string imgTopic = "img";
    
    ros::NodeHandle n;
    
    ros::Subscriber sub_imu = n.subscribe(imuTopic, 1000, imuCallback);
    ros::Subscriber sub_img = n.subscribe(imgTopic, 1000, imgCallback);
    
    ros::spin();
    
    return EXIT_SUCCESS;
}


