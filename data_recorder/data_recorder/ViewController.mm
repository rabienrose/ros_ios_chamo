//
//  ViewController.m
//  data_recorder
//
//  Created by user on 3/17/18.
//  Copyright © 2018 user. All rights reserved.
//

#import "ViewController.h"
#import <ifaddrs.h>
#import <arpa/inet.h>
#include "common_header.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <iomanip>
@interface ViewController ()
@end

@implementation ViewController
@synthesize defaults, ip_text_field;
@synthesize imgView;
CMMotionManager *motionManager;
    
- (void)setupVideoSession
{
    NSError *error = nil;
    AVCaptureSession *session = [[AVCaptureSession alloc] init];
    session.sessionPreset = AVCaptureSessionPreset640x480;
    NSArray *devices = [AVCaptureDevice devices];
    AVCaptureDevice *myDevice;
    for (AVCaptureDevice *device in devices) {
        if ([device hasMediaType:AVMediaTypeVideo]) {
            if ([device position] == AVCaptureDevicePositionBack) {
                myDevice =device;
            }
        }
    }

    AVCaptureDeviceInput *input = [AVCaptureDeviceInput deviceInputWithDevice:myDevice error:&error];
    if (!input){
        NSLog(@"Device input wrong!!");
    }
    if ([session canAddInput:input]) {
        [session addInput:input];
    }else {
        NSLog(@"add device wrong!!!");
    }
    if ([myDevice isFocusModeSupported:AVCaptureFocusModeLocked]) {
        NSError *error = nil;
        if ([myDevice lockForConfiguration:&error]) {
            myDevice.focusMode = AVCaptureFocusModeLocked;
            [myDevice setFocusModeLockedWithLensPosition:0.8 completionHandler:nil];
            [myDevice unlockForConfiguration];
        }
    }
    if ([myDevice isExposureModeSupported:AVCaptureExposureModeLocked]) {
        NSError *error = nil;
        if ([myDevice lockForConfiguration:&error]) {
            myDevice.exposureMode = AVCaptureExposureModeCustom;
            //std::cout<<myDevice.activeFormat.maxISO<<std::endl;;
            [myDevice setExposureModeCustomWithDuration:CMTimeMakeWithSeconds( 1/33.0, 1000*1000*1000 ) ISO:100 completionHandler:nil];
            [myDevice unlockForConfiguration];
        }
    }
    video_output = [[AVCaptureVideoDataOutput alloc] init];
    NSDictionary *newSettings = @{ (NSString *)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA) };
    video_output.videoSettings = newSettings;
    video_output.minFrameDuration = CMTimeMake(1, 10);
    [video_output setAlwaysDiscardsLateVideoFrames:YES];
    if ([session canAddOutput:video_output]) {
        [session addOutput:video_output];
    }else {
        NSLog(@"add output wrong!!!");
    }
    
    dispatch_queue_t videoDataOutputQueue = dispatch_queue_create("VideoDataOutputQueue", DISPATCH_QUEUE_SERIAL);
    [video_output setSampleBufferDelegate:self queue:videoDataOutputQueue];
    
    AVCaptureVideoPreviewLayer *previewLayer = nil;
    previewLayer = [[AVCaptureVideoPreviewLayer alloc] initWithSession:session];
    //[previewLayer setVideoGravity:AVLayerVideoGravityResizeAspect];
    CGRect layerRect = [[imgView layer] bounds];
    [previewLayer setBounds:layerRect];
    previewLayer.orientation = AVCaptureVideoOrientationLandscapeRight;
    [previewLayer setPosition:CGPointMake(CGRectGetMidX(layerRect),CGRectGetMidY(layerRect))];
    [[imgView layer] addSublayer:previewLayer];
    [session startRunning];
}
    
- (void)captureOutput:(AVCaptureOutput *)captureOutput
didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
       fromConnection:(AVCaptureConnection *)connection {
    //NSLog(@"img: %f", timestamp.value/(double)timestamp.timescale);
    //if(false){
    if(need_record==true){
        
        CMTime timestamp = CMSampleBufferGetPresentationTimeStamp(sampleBuffer);
        //NSLog(@"%f", timestamp.value/(double)timestamp.timescale);
        UIImage *image = [self imageFromSampleBuffer:sampleBuffer];
        cv::Mat img_cv = [mm_Try cvMatFromUIImage:image];
        float time_in_sec = timestamp.value/(double)timestamp.timescale;
        
        sensor_msgs::CompressedImage img_msg;
        cv::Mat img_gray;
        //cv::cvtColor(img_cv, img_gray, CV_BGRA2GRAY);
//        std::vector<unsigned char> binaryBuffer_;
//        cv::imencode(".jpg", img_cv, binaryBuffer_);
//        img_msg.data=binaryBuffer_;
//        img_msg.header.seq=img_count;
//        img_msg.header.stamp.sec=floor(time_in_sec);
//        img_msg.header.stamp.nsec=time_in_sec-floor(time_in_sec);
//        img_msg.format="jpeg";
//        img_pub.publish(img_msg);
//        static double lasttime=-1;
//        if(lasttime<0){
//            lasttime=time_in_sec;
//        }else{
//            std::cout<<time_in_sec - lasttime<<std::endl;
//            lasttime=time_in_sec;
//        }
        
        sensor_msgs::Image img_ros_img;
        cv::cvtColor(img_cv, img_gray, CV_BGRA2GRAY);
        img_ros_img.height=img_gray.rows;
        img_ros_img.width=img_gray.cols;
        img_ros_img.encoding=sensor_msgs::image_encodings::MONO8;
        img_ros_img.is_bigendian=false;
        img_ros_img.step=1*img_ros_img.width;
        img_ros_img.data.assign(img_gray.data, img_gray.data+img_ros_img.step*img_ros_img.height);

        img_ros_img.header.seq=img_count;
        img_ros_img.header.stamp= ros::Time(time_in_sec);
        img_pub.publish(img_ros_img);
        img_count++;
    }
}
    
    // Create a UIImage from sample buffer data
- (UIImage *) imageFromSampleBuffer:(CMSampleBufferRef) sampleBuffer
{
    // Get a CMSampleBuffer's Core Video image buffer for the media data
    CVImageBufferRef imageBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
    // Lock the base address of the pixel buffer
    CVPixelBufferLockBaseAddress(imageBuffer, 0);
    
    // Get the number of bytes per row for the pixel buffer
    void *baseAddress = CVPixelBufferGetBaseAddress(imageBuffer);
    
    // Get the number of bytes per row for the pixel buffer
    size_t bytesPerRow = CVPixelBufferGetBytesPerRow(imageBuffer);
    // Get the pixel buffer width and height
    size_t width = CVPixelBufferGetWidth(imageBuffer);
    size_t height = CVPixelBufferGetHeight(imageBuffer);
    
    // Create a device-dependent RGB color space
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    
    // Create a bitmap graphics context with the sample buffer data
    CGContextRef context = CGBitmapContextCreate(baseAddress, width, height, 8,
                                                 bytesPerRow, colorSpace, kCGBitmapByteOrder32Little | kCGImageAlphaPremultipliedFirst);
    // Create a Quartz image from the pixel data in the bitmap graphics context
    CGImageRef quartzImage = CGBitmapContextCreateImage(context);
    // Unlock the pixel buffer
    CVPixelBufferUnlockBaseAddress(imageBuffer,0);
    
    // Free up the context and color space
    CGContextRelease(context);
    CGColorSpaceRelease(colorSpace);
    
    // Create an image object from the Quartz image
    UIImage *image = [UIImage imageWithCGImage:quartzImage];
    //UIImage *image = [UIImage imageWithCGImage:quartzImage scale:1.0f orientation:UIImageOrientationRight];
    // Release the Quartz image
    CGImageRelease(quartzImage);
    
    return image;
}
    
- (IBAction)start_record:(id)sender{
    need_record=!need_record;
    if(need_record){
        [sender setTitle:@"Stop" forState:UIControlStateNormal];
    }else{
        [sender setTitle:@"Start" forState:UIControlStateNormal];
    }
}
- (IBAction)ip_edit_ended:(id)sender{
    [sender setTitle:@"Connected" forState:UIControlStateNormal];
    [defaults setObject:[ip_text_field.attributedText string] forKey:@"master_uri"];
    [defaults synchronize];
    [self connectToMaster];
}

void interDouble(double v1, double v2, double t1, double t2, double& v3_out, double t3){
    v3_out=v1+(v2-v1)*(t3-t1)/(t2-t1);
}

int getNearFuture(double cur_time, std::vector<std::vector<double>>& data){
    for (int i=0; i<data.size();i++){
        if(cur_time<data[i][3]){
            return i;
        }
    }
    return -1;
}

void getNextTime(double cur_time, std::vector<std::vector<double>>& acce, std::vector<std::vector<double>>& gyro, int& index, bool& is_acce){
    int acce_id = getNearFuture(cur_time, acce);
    int gyro_id = getNearFuture(cur_time, gyro);
    if(acce_id==-1 && gyro_id==-1){
        index=-1;
        return;
    }else{
        if(gyro_id==-1){
            index=acce_id;
            is_acce=true;
            return;
        }
        else if(acce_id==-1){
            index=gyro_id;
            is_acce=false;
            return;
        }else{
            if(acce[acce_id][3]>gyro[gyro_id][3]){
                index=gyro_id;
                is_acce=false;
                return;
            }else{
                index=acce_id;
                is_acce=true;
                return;
            }
        }
    }
}

void findFirstInter(std::vector<std::vector<double>>& acce, std::vector<std::vector<double>>& gyro, int& index, bool& is_acce){
    int acce_id=-1;
    for (int i=0; i<acce.size();i++){
        if(acce[i][4]==0){
            acce_id=i;
            break;
        }
    }
    int gyro_id=-1;
    for (int i=0; i<gyro.size();i++){
        if(gyro[i][4]==0){
            gyro_id=i;
            break;
        }
    }
    if(acce_id==-1 && gyro_id==-1){
        index=-1;
        return;
    }else{
        if(gyro_id==-1){
            index=acce_id;
            is_acce=true;
            return;
        }
        else if(acce_id==-1){
            index=gyro_id;
            is_acce=false;
            return;
        }else{
            if(acce[acce_id][3]>gyro[gyro_id][3]){
                index=gyro_id;
                is_acce=false;
                return;
            }else{
                index=acce_id;
                is_acce=true;
                return;
            }
        }
    }
}

int tryInter(std::vector<double> test, std::vector<std::vector<double>>& query, std::vector<sensor_msgs::Imu>& msgs, bool test_is_acce){
    int query_size=query.size();
    for(int j=0; j<query_size-1;j++){
        if(test[3]>query[j][3] && test[3]<=query[j+1][3]){
            sensor_msgs::Imu msg;
            double x,y,z;
            //std::cout<<query[j][0]<<" : "<<query[j+1][0]<<" : "<<query[j][3]<<std::endl;
            interDouble(query[j][0], query[j+1][0], query[j][3], query[j+1][3], x, test[3]);
            interDouble(query[j][1], query[j+1][1], query[j][3], query[j+1][3], y, test[3]);
            interDouble(query[j][2], query[j+1][2], query[j][3], query[j+1][3], z, test[3]);
            //std::cout<<std::setprecision(20)<<query[j][0]<<" : "<<x<<" : "<<query[j+1][0]<<" | "<<query[j][3]<<" : "<<test[i][3]<<" : "<<query[j+1][3]<<std::endl;
            if(test_is_acce){
                msg.linear_acceleration.x=test[0];
                msg.linear_acceleration.y=test[1];
                msg.linear_acceleration.z=test[2];
                msg.angular_velocity.x=x;
                msg.angular_velocity.y=y;
                msg.angular_velocity.z=z;
            }else{
                msg.linear_acceleration.x=x;
                msg.linear_acceleration.y=y;
                msg.linear_acceleration.z=z;
                msg.angular_velocity.x=test[0];
                msg.angular_velocity.y=test[1];
                msg.angular_velocity.z=test[2];
            }
            static int imu_data_seq=0;
            msg.header.seq=imu_data_seq;
            msg.header.stamp= ros::Time(test[3]);
            msgs.push_back(msg);
            imu_data_seq++;
            //std::cout<<std::setprecision(20)<<msg.linear_acceleration.x<<","<<msg.linear_acceleration.y<<","<<test[3]<<std::endl;
            //NSLog(@"ax: %f, ay: %f, az: %f, gx: %f, gy: %f, gz: %f, t: %f", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, test[3]);
            return j;
        }
    }
    return -1;
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
                //std::cout<<msg.header.stamp.nsec<<std::endl;
                //std::cout<<std::setprecision(20)<<msg.linear_acceleration.x<<","<<msg.linear_acceleration.y<<","<<gyros[i][3]<<std::endl;
                //NSLog(@"ax: %f, ay: %f, az: %f, gx: %f, gy: %f, gz: %f, t: %f", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, gyros[i][3]);
                imu_pub.publish(msg);
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

- (void)processIMU_acce{
    int last_acc_id=-1;
    int last_gyro_id=-1;
    int a_size=acces.size();
    int g_size=gyros.size();
    for(int i=0;i<a_size;i++){
        for(int j=0;j<g_size-1;j++){
            if(acces[i][3]>acces[j][3] && acces[i][3]<=acces[j+1][3]){
                sensor_msgs::Imu msg;
                double x,y,z;
                interDouble(gyros[j][0], gyros[j+1][0], gyros[j][3], gyros[j+1][3], x, acces[i][3]);
                interDouble(gyros[j][1], gyros[j+1][1], gyros[j][3], gyros[j+1][3], y, acces[i][3]);
                interDouble(gyros[j][2], gyros[j+1][2], gyros[j][3], gyros[j+1][3], z, acces[i][3]);
                msg.angular_velocity.x=x;
                msg.angular_velocity.y=y;
                msg.angular_velocity.z=z;
                msg.linear_acceleration.x=acces[i][0];
                msg.linear_acceleration.y=acces[i][1];
                msg.linear_acceleration.z=acces[i][2];
                static int imu_data_seq=0;
                msg.header.seq=imu_data_seq;
                msg.header.stamp= ros::Time(acces[i][3]);
                //std::cout<<msg.header.stamp.nsec<<std::endl;
                //std::cout<<std::setprecision(20)<<msg.linear_acceleration.x<<","<<msg.linear_acceleration.y<<","<<gyros[i][3]<<std::endl;
                //NSLog(@"ax: %f, ay: %f, az: %f, gx: %f, gy: %f, gz: %f, t: %f", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, gyros[i][3]);
                imu_pub.publish(msg);
                imu_data_seq++;
                last_gyro_id=j;
                last_acc_id=i;
                break;
            }
        }
    }
    if(last_gyro_id>0){
        if(last_gyro_id-1<gyros.size()){
            gyros.erase(gyros.begin(), gyros.begin()+last_gyro_id);
        }else{
            NSLog(@"test overflow");
        }
    }
    if(last_acc_id>=0){
        if(last_acc_id<acces.size()){
            acces.erase(acces.begin(), acces.begin()+last_acc_id+1);
        }else{
            NSLog(@"test overflow");
        }
    }
}


- (void)processIMU{
    int index_first;
    bool is_acce_first;
    findFirstInter(acces, gyros, index_first, is_acce_first);
    if(index_first!=-1){
        int last_acc_id=-1;
        int last_gyro_id=-1;
        std::vector<sensor_msgs::Imu> msgs;
        bool first_item=true;
        while(true){
            int index;
            bool is_acce;
            double cur_time;
            if(first_item){
                index=index_first;
                is_acce=is_acce_first;
                if(is_acce_first){
                    cur_time=acces[index_first][3];
                }else{
                    cur_time=gyros[index_first][3];
                }
                first_item=false;
            }else{
                getNextTime(cur_time, acces, gyros, index, is_acce);
                //std::cout<<index<<std::endl;
                if(index!=-1){
                    if(is_acce){
                        cur_time=acces[index][3];
                        if(acces[index][4]==1){
                            continue;
                        }
                    }else{
                        cur_time=gyros[index][3];
                        if(gyros[index][4]==1){
                            continue;
                        }
                    }
                }else{
                    break;
                }
                
            }
            
            int query_id;
            if(is_acce){
                query_id = tryInter(acces[index], gyros, msgs, true);
            }else{
                query_id = tryInter(gyros[index], acces, msgs, false);
            }
            if(query_id!=-1){
                if(is_acce){
                    acces[index][4]=1;
                    last_gyro_id=query_id;
                }else{
                    gyros[index][4]=1;
                    last_acc_id=query_id;
                }
            }
        }
        //std::cout<<last_gyro_id<<"=="<<last_acc_id<<std::endl;
        for(int i=0;i<msgs.size();i++){
            imu_pub.publish(msgs[i]);
        }
        if(last_acc_id>0){
            if(last_acc_id-1<acces.size()){
                acces.erase(acces.begin(), acces.begin()+last_acc_id);
            }else{
                NSLog(@"test overflow");
            }
        }
        if(last_gyro_id>0){
            if(last_gyro_id-1<gyros.size()){
                gyros.erase(gyros.begin(), gyros.begin()+last_gyro_id);
            }else{
                NSLog(@"test overflow");
            }
        }
    }
}

- (void)viewDidLoad {
    [super viewDidLoad];
    need_record=false;
    img_count=0;
    imu_count=0;
    defaults = [NSUserDefaults standardUserDefaults];
    [ip_text_field setText:[defaults objectForKey:@"master_uri"]];
    motionManager = [[CMMotionManager alloc] init];
    NSOperationQueue *quene =[[NSOperationQueue alloc] init];
    quene.maxConcurrentOperationCount=1;
    if (motionManager.accelerometerAvailable){
        motionManager.accelerometerUpdateInterval =0.01;
        [motionManager
         startAccelerometerUpdatesToQueue:quene
         withHandler:
         ^(CMAccelerometerData *data, NSError *error){
             if(need_record==true){
                 std::vector<double> imu;
                 //std::cout<<std::setprecision(20)<<data.acceleration.x<<","<<data.acceleration.y<<","<<data.acceleration.z<<std::endl;
                 imu.resize(5);
                 imu[0]=-data.acceleration.x*9.8;
                 imu[1]=-data.acceleration.y*9.8;
                 imu[2]=-data.acceleration.z*9.8;
                 imu[3]=data.timestamp;
                 imu[4]=0;
                 //std::cout<<"acce"<<" : "<<data.timestamp<<std::endl;
                 acces.push_back(imu);
                 //[self processIMU];
             }
             
         }];
    }
    if (motionManager.gyroAvailable){
        motionManager.gyroUpdateInterval =0.01;
        [motionManager
         startGyroUpdatesToQueue:quene
         withHandler:
         ^(CMGyroData *data, NSError *error){
             if(need_record== true){
                 std::vector<double> imu;
                 imu.resize(5);
                 imu[0]=data.rotationRate.x;
                 imu[1]=data.rotationRate.y;
                 imu[2]=data.rotationRate.z;
                 imu[3]=data.timestamp;
                 imu[4]=0;
                 //std::cout<<std::setprecision(20)<<"gyros"<<" : "<<data.timestamp<<std::endl;
                 gyros.push_back(imu);
                 [self processIMU_gyro];
             }
         }];
    }
    [self setupVideoSession];
}


- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}
    
+ (NSString *)getIPAddress
{
    struct ifaddrs *interfaces = NULL;
    struct ifaddrs *temp_addr = NULL;
    NSString *wifiAddress = nil;
    NSString *usbAddress = nil;
    
    // retrieve the current interfaces - returns 0 on success
    if(!getifaddrs(&interfaces)) {
        // Loop through linked list of interfaces
        temp_addr = interfaces;
        while(temp_addr != NULL) {
            sa_family_t sa_type = temp_addr->ifa_addr->sa_family;
            if(sa_type == AF_INET || sa_type == AF_INET6) {
                NSString *name = [NSString stringWithUTF8String:temp_addr->ifa_name];
                NSString *addr = [NSString stringWithUTF8String:inet_ntoa(((struct sockaddr_in *)temp_addr->ifa_addr)->sin_addr)]; // pdp_ip0
                NSLog(@"NAME: \"%@\" addr: %@", name, addr); // see for yourself

                if([name isEqualToString:@"bridge100"]) {
                    if(![addr isEqualToString:@"0.0.0.0"]){
                        usbAddress = addr;
                    }
                }
                if([name isEqualToString:@"en0"]) {
                    if(![addr isEqualToString:@"0.0.0.0"]){
                        wifiAddress = addr;
                    }
                }
            }
            temp_addr = temp_addr->ifa_next;
        }
        // Free memory
        freeifaddrs(interfaces);
    }
    NSString *addr=nil;
    if(usbAddress){
        addr=usbAddress;
    }else{
        addr=wifiAddress;
    }
    return addr ? addr : @"0.0.0.0";
}
    
- (BOOL)connectToMaster
{
    if([ViewController isValidIp:[ip_text_field.attributedText string]])
    {
        NSString * master_uri = [@"ROS_MASTER_URI=http://" stringByAppendingString:[[ip_text_field.attributedText string] stringByAppendingString:@":11311/"]];
        NSLog(@"%@",master_uri);
        NSString * ip = [@"ROS_IP=" stringByAppendingString:[ViewController getIPAddress]];
        NSLog(@"%@",ip);
        putenv((char *)[master_uri UTF8String]);
        putenv((char *)[ip UTF8String]);
        putenv((char *)"ROS_HOME=/tmp");
        
        int argc = 0;
        char ** argv = NULL;
        if(!ros::isInitialized())
        {
            ros::init(argc,argv,"data_recorder");
            if(ros::master::check())
            {
                NSLog(@"Connected to the ROS master !");
                ros::NodeHandle nn;
                img_pub = nn.advertise<sensor_msgs::Image>("/cam0/image_raw", 10000, false);
                imu_pub = nn.advertise<sensor_msgs::Imu>("/imu0", 10000, false);
            }
            else
            {
                NSLog(@"fail to connect to the ROS master !");
            }
        }
        else
        {
            NSLog(@"ROS already initialised. Can'st change the ROS_MASTER_URI");
        }
    }else{
        NSLog(@"input a right ip !");
        return false;
    }
    return true;
}
    
+ (BOOL)isValidIp:(NSString*)string
{
    struct in_addr pin;
    int success = inet_pton(AF_INET,[string UTF8String],&pin);
    if(success == 1) return YES;
    return NO;
}


@end
