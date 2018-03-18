#include <ros/ros.h>
#include <cstdio>
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <map>
#include <opencv2/opencv.hpp>
#include <memory>
#include <save_bag/img_chamo.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>


void on_image(const save_bag::img_chamo::ConstPtr &msg){
    cv::Mat temp_img;
    temp_img = cv::imdecode(msg->jpg, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_COLOR);
    cv::Mat small_img;
    float scale=0.75;
    cv::resize(temp_img, small_img, cv::Size(), scale, scale);
    cv::imshow("chamo", small_img);
    std::cout<<msg->frame_id<<std::endl;
    cv::waitKey(1);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "save_bag");
    ros::NodeHandle n;
    ros::Rate r(60);
    ros::Subscriber img_sub = n.subscribe<save_bag::img_chamo>("img_chamo", 10000, on_image);
    rosbag::Bag bag;
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    std::cout << "end loop!!" << std::endl;
    return 0;
};
