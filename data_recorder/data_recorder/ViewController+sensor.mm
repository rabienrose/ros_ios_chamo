#import "ViewController+sensor.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <rosbag/view.h>

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

- (void)changeCamSize:(int)cam_size_id{
    [self.session beginConfiguration];
    self.session.sessionPreset = self.camSizes[cam_size_id];
    [self.session commitConfiguration];
    [self.cam_size_btn setTitle:self.camSizesName[cam_size_id] forState:UIControlStateNormal];
    [defaults setObject:self.camSizesName[cam_size_id] forKey:@"img_size"];
    [defaults synchronize];
}

- (void)update_baglist{
    NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSArray *directoryContent = [[NSFileManager defaultManager] contentsOfDirectoryAtPath:[dirPaths objectAtIndex:0] error:NULL];
    bool has_file=false;
    for (int count = 0; count < (int)[directoryContent count]; count++)
    {
        if (count ==0){
            sel_filename=[directoryContent objectAtIndex:count];
            has_file=true;
            break;
        }
        NSLog(@"File %d: %@", (count + 1), [directoryContent objectAtIndex:count]);
    }
    file_list=directoryContent;
    [self.bag_list_ui reloadAllComponents];
    if ((int)[directoryContent count]>0){
        [self.bag_list_ui selectRow:0 inComponent:0 animated:NO];
    }
}

- (NSString*)get_bag_info:(NSString *)filename{
    NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSArray *directoryContent = [[NSFileManager defaultManager] contentsOfDirectoryAtPath:[dirPaths objectAtIndex:0] error:NULL];
    std::map<std::string, int> re_dict;
    for (int count = 0; count < (int)[directoryContent count]; count++)
    {
        NSError *error = nil;
        NSString *full_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:[directoryContent objectAtIndex:count]];
        if([filename isEqualToString:[directoryContent objectAtIndex:count]]==YES){
            rosbag::Bag bag;
            bag.open(std::string([full_addr UTF8String]).c_str(), rosbag::bagmode::Read);
            rosbag::View view(bag);
            int img_count=0;
            rosbag::View::iterator it= view.begin();
            for(;it!=view.end();it++){
                rosbag::MessageInstance m =*it;
                std::cout<<img_count<<":"<<m.getTopic()<<std::endl;
                if (re_dict.count(m.getTopic())==0){
                    re_dict[m.getTopic()]=1;
                }else{
                    re_dict[m.getTopic()]=re_dict[m.getTopic()]+1;
                }
            }
        }
    }
    std::stringstream ss;
    for(std::map<std::string, int>::iterator it=re_dict.begin(); it!=re_dict.end(); it++){
        ss<<it->first<<": "<<it->second<<std::endl;
    }
    NSString *string1 = [NSString stringWithCString:ss.str().c_str() encoding:[NSString defaultCStringEncoding]];
    return string1;
}

- (NSURLResponse *)getLocalFileResponse:(NSString *)urlString
{
    urlString = [urlString stringByAddingPercentEncodingWithAllowedCharacters:[NSCharacterSet URLFragmentAllowedCharacterSet]];
    NSURL *url = [NSURL fileURLWithPath:urlString];
    NSURLRequest *request = [NSURLRequest requestWithURL:url];
    __block NSURLResponse *localResponse = nil;
    dispatch_semaphore_t semaphore = dispatch_semaphore_create(0);
    [[[NSURLSession sharedSession] dataTaskWithRequest:request completionHandler:^(NSData * _Nullable data, NSURLResponse * _Nullable response, NSError * _Nullable error) {
        localResponse = response;
        dispatch_semaphore_signal(semaphore);
    }] resume];
    dispatch_semaphore_wait(semaphore, DISPATCH_TIME_FOREVER);
    return  localResponse;
}

- (NSData *)getHttpBodyWithFilePath:(NSString *)filePath formName:(NSString *)formName reName:(NSString *)reName
{
    NSMutableData *data = [NSMutableData data];
    NSURLResponse *response = [self getLocalFileResponse:filePath];
    NSString *fileType = response.MIMEType;
    if (reName == nil) {
        reName = response.suggestedFilename;
    }
    NSMutableString *headerStrM =[NSMutableString string];
    [headerStrM appendFormat:@"--%@\r\n",@"boundary"];
    [headerStrM appendFormat:@"Content-Disposition: form-data; name=%@; filename=%@\r\n",formName,reName];
    [headerStrM appendFormat:@"Content-Type: %@\r\n\r\n",fileType];
    [data appendData:[headerStrM dataUsingEncoding:NSUTF8StringEncoding]];
    NSData *fileData = [NSData dataWithContentsOfFile:filePath];
    [data appendData:fileData];
    NSMutableString *footerStrM = [NSMutableString stringWithFormat:@"\r\n--%@--\r\n",@"boundary"];
    [data appendData:[footerStrM  dataUsingEncoding:NSUTF8StringEncoding]];
    //    NSLog(@"dataStr=%@",[[NSString alloc] initWithData:data encoding:NSUTF8StringEncoding]);
    return data;
}

-(void) send_file:(NSString *)addr filename:(NSString *)filename{
    NSString *urlString = @"http://192.168.1.178:21070/upload_bag";
    urlString = [urlString stringByAddingPercentEncodingWithAllowedCharacters:[NSCharacterSet URLFragmentAllowedCharacterSet]];
    NSURL *url = [NSURL URLWithString:urlString];
    NSMutableURLRequest *request = [NSMutableURLRequest requestWithURL:url];
    request.HTTPMethod = @"POST";
    NSString *contentType = [NSString stringWithFormat:@"multipart/form-data; boundary=%@",@"boundary"];
    [request setValue:contentType forHTTPHeaderField:@"Content-Type"];
    
    // Get form data from local image path.
    NSData* data = [self getHttpBodyWithFilePath:addr formName:@"file" reName:filename];
    request.HTTPBody = data;
    [request setValue:[NSString stringWithFormat:@"%lu",data.length] forHTTPHeaderField:@"Content-Length"];
    
    dispatch_async( dispatch_get_main_queue(), ^{
        for (int count = 0; count < (int)[file_list count]; count++){
            NSString * cur_string=[file_list objectAtIndex:count];
            if ([cur_string isEqualToString:filename]) {
                file_list[count]=[cur_string stringByAppendingString:@"(*)"];
            }
        }
        [self.bag_list_ui reloadAllComponents];
    } );
    
    // Use dataTask
    NSURLSession *session = [NSURLSession sharedSession];
    [[ session dataTaskWithRequest:request completionHandler:^(NSData * _Nullable data, NSURLResponse * _Nullable response, NSError * _Nullable error) {
        NSString * re_msg;
        if (error == nil) {
            re_msg=@"Upload successed!";
            NSLog(@"upload success：%@",[[NSString alloc] initWithData:data encoding:NSUTF8StringEncoding]);
        } else {
            re_msg=@"Upload failed!";
            NSLog(@"upload error:%@",error);
        }
        dispatch_async( dispatch_get_main_queue(), ^{
            UIAlertController *alert = [UIAlertController alertControllerWithTitle:filename message:re_msg preferredStyle:UIAlertControllerStyleAlert];
            [self presentViewController:alert animated:YES completion:nil];
            [self performSelector:@selector(dismiss:) withObject:alert afterDelay:1.0];
            for (int count = 0; count < (int)[file_list count]; count++)
            {
                NSString * cur_string=[file_list objectAtIndex:count];
                if ([cur_string containsString:filename]) {
                    file_list[count]=filename;
                }
            }
            [self.bag_list_ui reloadAllComponents];
        } );
        
    }] resume];
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
            [self send_file:full_addr filename:[directoryContent objectAtIndex:count]];
        }else{
            if([filename isEqualToString:[directoryContent objectAtIndex:count]]==YES){
                [self send_file:full_addr filename:filename];
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

- (void)dismiss:(UIAlertController *)alert{
    [alert dismissViewControllerAnimated:YES completion:nil];
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
