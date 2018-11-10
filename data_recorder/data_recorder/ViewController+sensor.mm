#import "ViewController+sensor.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

@implementation AVCamManualCameraViewController (Sensor)

// Create a UIImage from sample buffer data
- (UIImage *) imageFromSampleBuffer:(CMSampleBufferRef) sampleBuffer
{
    CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    CVPixelBufferLockBaseAddress(imageBuffer, 0);
    void *baseAddress = CVPixelBufferGetBaseAddress(imageBuffer);
    size_t bytesPerRow = CVPixelBufferGetBytesPerRow(imageBuffer);
    size_t width = CVPixelBufferGetWidth(imageBuffer);
    size_t height = CVPixelBufferGetHeight(imageBuffer);
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    CGContextRef context = CGBitmapContextCreate(baseAddress, width, height, 8,
                                                 bytesPerRow, colorSpace, kCGBitmapByteOrder32Little | kCGImageAlphaPremultipliedFirst);
    CGImageRef quartzImage = CGBitmapContextCreateImage(context);
    CVPixelBufferUnlockBaseAddress(imageBuffer,0);
    CGContextRelease(context);
    CGColorSpaceRelease(colorSpace);
    UIImage *image = [UIImage imageWithCGImage:quartzImage];
    CGImageRelease(quartzImage);
    return image;
}


void interDouble(double v1, double v2, double t1, double t2, double& v3_out, double t3){
    v3_out=v1+(v2-v1)*(t3-t1)/(t2-t1);
}

- (void)processIMU_gyro{
    int last_acc_id=-1;
    int last_gyro_id=-1;
    int g_size=gyros.size();
    int a_size=acces.size();
    for(int i=0;i<gyros.size();i++){
        for(int j=0;j<a_size-1;j++){
            if(gyros[i][3]>acces[j][3] && gyros[i][3]<=acces[j+1][3]){
                sensor_msgs::Imu msg;
                double x,y,z;
                interDouble(acces[j][0], acces[j+1][0], acces[j][3], acces[j+1][3], x, gyros[i][3]);
                interDouble(acces[j][1], acces[j+1][1], acces[j][3], acces[j+1][3], y, gyros[i][3]);
                interDouble(acces[j][2], acces[j+1][2], acces[j][3], acces[j+1][3], z, gyros[i][3]);
                msg.linear_acceleration.x=x;
                msg.linear_acceleration.y=y;
                msg.linear_acceleration.z=z;
                msg.angular_velocity.x=gyros[i][0];
                msg.angular_velocity.y=gyros[i][1];
                msg.angular_velocity.z=gyros[i][2];
                static int imu_data_seq=0;
                msg.header.seq=imu_data_seq;
                msg.header.stamp= ros::Time(gyros[i][3]);
                if(is_publishing){
                    imu_pub.publish(msg);
                }
                dispatch_async( self.sessionQueue, ^{
                    if (is_recording_bag){
                        if(bag_ptr->isOpen()){
                            bag_ptr->write("imu0", msg.header.stamp, msg);
                        }
                    }
                });
                imu_data_seq++;
                last_acc_id=j;
                last_gyro_id=i;
                break;
            }
        }
    }
    if(last_acc_id>0){
        if(last_acc_id-1<acces.size()){
            acces.erase(acces.begin(), acces.begin()+last_acc_id);
        }else{
            NSLog(@"test overflow");
        }
    }
    if(last_gyro_id>=0){
        if(last_gyro_id<gyros.size()){
            gyros.erase(gyros.begin(), gyros.begin()+last_gyro_id+1);
        }else{
            NSLog(@"test overflow");
        }
    }
}



@end
