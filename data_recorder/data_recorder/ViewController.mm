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
#include <opencv2/opencv.hpp>
#include "common_header.h"
#include <save_bag/img_chamo.h>

@interface ViewController ()
@end

@implementation ViewController
@synthesize defaults, ip_text_field;
@synthesize imgView;

    
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
    
    if ([myDevice isFocusModeSupported:AVCaptureFocusModeLocked]) {
        NSError *error = nil;
        if ([myDevice lockForConfiguration:&error]) {
            myDevice.focusMode = AVCaptureFocusModeLocked;
            [myDevice unlockForConfiguration];
        }
    }
    if ([myDevice isExposureModeSupported:AVCaptureExposureModeLocked]) {
        NSError *error = nil;
        if ([myDevice lockForConfiguration:&error]) {
            myDevice.exposureMode = AVCaptureExposureModeLocked;
            [myDevice unlockForConfiguration];
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
    if(need_record==true){
        img_count++;
        CMTime timestamp = CMSampleBufferGetPresentationTimeStamp(sampleBuffer);
        //NSLog(@"%f", timestamp.value/(double)timestamp.timescale);
        UIImage *image = [self imageFromSampleBuffer:sampleBuffer];
        cv::Mat img_cv = [mm_Try cvMatFromUIImage:image];
        std::vector<unsigned char> binaryBuffer_;
        cv::imencode(".jpg", img_cv, binaryBuffer_);
        save_bag::img_chamo img_msg;
        img_msg.jpg=binaryBuffer_;
        img_msg.absTimestamp=(timestamp.value/(double)timestamp.timescale)*1000;
        img_msg.frame_id=img_count;
        img_chamo_pub.publish(img_msg);
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
}
- (IBAction)ip_edit_ended:(id)sender{
    [defaults setObject:[ip_text_field.attributedText string] forKey:@"master_uri"];
    [defaults synchronize];
    [self connectToMaster];
}
    
- (void)viewDidLoad {
    [super viewDidLoad];
    need_record=false;
    img_count=0;
    defaults = [NSUserDefaults standardUserDefaults];
    [ip_text_field setText:[defaults objectForKey:@"master_uri"]];
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
    NSString *cellAddress = nil;
    
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
                
                if([name isEqualToString:@"en0"]) {
                    // Interface is the wifi connection on the iPhone
                    wifiAddress = addr;
                } else
                if([name isEqualToString:@"pdp_ip0"]) {
                    // Interface is the cell connection on the iPhone
                    //cellAddress = addr;
                }
            }
            temp_addr = temp_addr->ifa_next;
        }
        // Free memory
        freeifaddrs(interfaces);
    }
    NSString *addr = wifiAddress ? wifiAddress : cellAddress;
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
                img_chamo_pub = nn.advertise<save_bag::img_chamo>("img_chamo", 10000, false);
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
