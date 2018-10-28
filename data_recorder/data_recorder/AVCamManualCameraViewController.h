#import <UIKit/UIKit.h>
#import <ros/ros.h>
#include "std_msgs/String.h"
#import <QuartzCore/QuartzCore.h>
#import <CoreMotion/CoreMotion.h>
#import <CoreMedia/CoreMedia.h>
#import <AVFoundation/AVFoundation.h>
#include <deque>
#include <rosbag/bag.h>
#include <memory>

@interface AVCamManualCameraViewController : UIViewController<AVCaptureVideoDataOutputSampleBufferDelegate>{
    AVCaptureVideoDataOutput *video_output;
    AVCaptureDevice *videoDevice;
    ros::Publisher img_pub;
    ros::Publisher imu_pub;
    bool need_record;
    int img_count;
    int imu_count;
    bool is_recording_bag;
    bool is_publishing;
    std::vector<std::vector<double>> gyros;
    std::vector<std::vector<double>> acces;
    std::shared_ptr<rosbag::Bag> bag_ptr;
}

@end
