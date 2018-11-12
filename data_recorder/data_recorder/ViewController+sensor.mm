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

-(void) upload_sel_bag{
    
}

- (void)update_baglist{
    NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSArray *directoryContent = [[NSFileManager defaultManager] contentsOfDirectoryAtPath:[dirPaths objectAtIndex:0] error:NULL];
    for (int count = 0; count < (int)[directoryContent count]; count++)
    {
        if (count ==0){
            sel_filename=[directoryContent objectAtIndex:count];
        }
        //NSLog(@"File %d: %@", (count + 1), [directoryContent objectAtIndex:count]);
    }
    file_list=directoryContent;
    [self.bag_list_ui reloadAllComponents];
    [self.bag_list_ui selectRow:0 inComponent:0 animated:NO];
}

-(void) send_file:(NSString *)filename{
    
    
    NSURL *url = [NSURL URLWithString:@"192.168.1.178:21070/upload_bag"];
    NSURLSessionConfiguration *config = [NSURLSessionConfiguration defaultSessionConfiguration];
    NSURLSession *session = [NSURLSession sessionWithConfiguration:config];
    
    // 2
    NSMutableURLRequest *request = [[NSMutableURLRequest alloc] initWithURL:url];
    request.HTTPMethod = @"POST";
    
    // 3
    NSDictionary *dictionary = @{@"key1": @"value1"};
    NSError *error = nil;
    NSData *data = [NSJSONSerialization dataWithJSONObject:dictionary
                                                   options:kNilOptions error:&error];
    
    if (!error) {
        // 4
        NSURLSessionUploadTask *uploadTask = [session uploadTaskWithRequest:request
                                                                   fromData:data completionHandler:^(NSData *data,NSURLResponse *response,NSError *error) {
                                                                       // Handle response here
                                                                   }];
        
        // 5
        [uploadTask resume];
    
    }
    
//      NSURL *URL = [NSURL URLWithString:@"192.168.1.178:21070/upload_bag"];
//    NSURL *URL_file = [NSURL URLWithString:filename];
//    NSMutableURLRequest* request = [NSMutableURLRequest requestWithURL:URL];
//    [request setHTTPMethod:@"POST"];
//    NSURLSession *session = [NSURLSession sharedSession];
//    NSURLSessionUploadTask *uploadTask = [session uploadTaskWithRequest:request
//                                                               fromFile:URL_file
//                                                      completionHandler:
//                                          ^(NSData *data, NSURLResponse *response, NSError *error) {
//                                              //NSLog([[NSString alloc] initWithData:data encoding:NSUTF8StringEncoding]);
//                                          }];
//    [uploadTask resume];
}

void send_txt(std::string contnent) {
    NSData *data = [@(contnent.c_str()) dataUsingEncoding:NSUTF8StringEncoding];
    NSURL *URL = [NSURL URLWithString:@"http://192.168.1.178:21070/upload_bag"];
    NSMutableURLRequest* request = [NSMutableURLRequest requestWithURL:URL];
    [request setHTTPMethod:@"POST"];
    NSURLSession *session = [NSURLSession sharedSession];
    NSURLSessionUploadTask *uploadTask = [session uploadTaskWithRequest:request
                                                               fromData:data
                                                      completionHandler:
                                          ^(NSData *data, NSURLResponse *response, NSError *error) {
                                              //NSLog([[NSString alloc] initWithData:data encoding:NSUTF8StringEncoding]);
                                          }];
    [uploadTask resume];
}

- (void)upload_baglist:(bool)upload_all filename:(NSString *)filename{
    NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSArray *directoryContent = [[NSFileManager defaultManager] contentsOfDirectoryAtPath:[dirPaths objectAtIndex:0] error:NULL];
    for (int count = 0; count < (int)[directoryContent count]; count++)
    {
        NSError *error = nil;
        NSString *full_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:[directoryContent objectAtIndex:count]];
        if(upload_all){
            
        }else{
            if([filename isEqualToString:[directoryContent objectAtIndex:count]]==YES){
                send_txt("sadfasd");
                //[self send_file:full_addr];
            }
        }
    }
}

- (void)del_baglist:(bool)del_all filename:(NSString *)filename{
    NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    //NSLog(@"File3 count %d", (int)[dirPaths count]);
    NSArray *directoryContent = [[NSFileManager defaultManager] contentsOfDirectoryAtPath:[dirPaths objectAtIndex:0] error:NULL];
    for (int count = 0; count < (int)[directoryContent count]; count++)
    {
        NSError *error = nil;
        NSString *full_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:[directoryContent objectAtIndex:count]];
        if(del_all){
            [[NSFileManager defaultManager] removeItemAtPath:full_addr error:&error];
        }else{
            if([filename isEqualToString:[directoryContent objectAtIndex:count]]==YES){
                [[NSFileManager defaultManager] removeItemAtPath:full_addr error:&error];
            }
        }
    }
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
                            NSString *temp_string =  [self.imu_topic_edit.attributedText string];
                            bag_ptr->write((char *)[temp_string UTF8String], msg.header.stamp, msg);
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
