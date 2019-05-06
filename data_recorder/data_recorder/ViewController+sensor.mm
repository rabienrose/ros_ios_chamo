#include <opencv2/opencv.hpp>
#import "ViewController+sensor.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <rosbag/view.h>
#import <ifaddrs.h>
#import <arpa/inet.h>
#include <sensor_msgs/NavSatFix.h>
#include <math.h>
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

- (void) startIMUUpdate{
    if (self.motionManager.accelerometerAvailable){
        self.motionManager.accelerometerUpdateInterval =0.01;
        [self.motionManager
         startAccelerometerUpdatesToQueue:self.quene
         withHandler:
         ^(CMAccelerometerData *data, NSError *error){
             //NSLog(@"acce: %@", [NSThread currentThread]);
             if(![self.imu_switch isOn]){
                 return;
             }
             std::vector<double> imu;
             imu.resize(5);
             imu[0]=-data.acceleration.x*9.8;
             imu[1]=-data.acceleration.y*9.8;
             imu[2]=-data.acceleration.z*9.8;
             imu[3]=data.timestamp;
             imu[4]=0;
             acces.push_back(imu);
         }];
    }
    if (self.motionManager.gyroAvailable){
        self.motionManager.gyroUpdateInterval =0.01;
        [self.motionManager
         startGyroUpdatesToQueue:self.quene
         withHandler:
         ^(CMGyroData *data, NSError *error){
             //NSLog(@"gyro: %@", [NSThread currentThread]);
             if(![self.imu_switch isOn]){
                 return;
             }
             std::vector<double> imu;
             imu.resize(5);
             imu[0]=data.rotationRate.x;
             imu[1]=data.rotationRate.y;
             imu[2]=data.rotationRate.z;
             imu[3]=data.timestamp;
             imu[4]=0;
             gyros.push_back(imu);
             [self processIMU_gyro];
         }];
    }
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
    double start_time;
    double end_time;
    for (int count = 0; count < (int)[directoryContent count]; count++)
    {
        NSError *error = nil;
        NSString *full_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:[directoryContent objectAtIndex:count]];
        if([filename isEqualToString:[directoryContent objectAtIndex:count]]==YES){
            rosbag::Bag bag;
            try{
                bag.open(std::string([full_addr UTF8String]).c_str(), rosbag::bagmode::Read);
            }
            catch (...){
                return @"bad bag!!";
            }
            if(!bag.isOpen()){
                return @"bad bag!!";
            }
            rosbag::View view(bag);
            int img_count=0;
            start_time = view.getBeginTime().toSec();
            end_time = view.getEndTime().toSec();
            rosbag::View::iterator it= view.begin();
            for(;it!=view.end();it++){
                rosbag::MessageInstance m =*it;
                if (re_dict.count(m.getTopic())==0){
                    re_dict[m.getTopic()]=1;
                }else{
                    re_dict[m.getTopic()]=re_dict[m.getTopic()]+1;
                }
            }
            bag.close();
            break;
        }
    }
    std::stringstream ss;
    for(std::map<std::string, int>::iterator it=re_dict.begin(); it!=re_dict.end(); it++){
        ss<<it->first<<": "<<it->second<<std::endl;
    }
    ss<<"duration: "<<end_time-start_time<<"s";
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
                msg.header.frame_id="map";
                if(is_publishing){
                    imu_pub.publish(msg);
                }
                dispatch_async( self.sessionQueue, ^{
                    if (is_recording_bag){
                        if(bag_ptr->isOpen()){
                            NSString *temp_string =  [self.imu_topic_edit.attributedText string];
                            NSDate * t1 = [NSDate date];
                            NSTimeInterval now = [t1 timeIntervalSince1970];
                            bag_ptr->write((char *)[temp_string UTF8String], ros::Time(now), msg);
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

double pi = 3.1415926535897932384626;
double a = 6378245.0;
double ee = 0.00669342162296594323;


bool outOfChina(double lat, double lon) {
    if (lon < 72.004 || lon > 137.8347) return true;
    if (lat < 0.8293 || lat > 55.8271) return true;
    return false;
}

double transformLat(double x, double y) {
    double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y
    + 0.2 * sqrt(fabs(x));
    ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    ret += (20.0 * sin(y * pi) + 40.0 * sin(y / 3.0 * pi)) * 2.0 / 3.0;
    ret += (160.0 * sin(y / 12.0 * pi) + 320 * sin(y * pi / 30.0)) * 2.0 / 3.0;
    return ret;
}
double transformLon(double x, double y) {
    double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1* sqrt(fabs(x));
    ret += (20.0 * sin(6.0 * x * pi) + 20.0 * sin(2.0 * x * pi)) * 2.0 / 3.0;
    ret += (20.0 * sin(x * pi) + 40.0 * sin(x / 3.0 * pi)) * 2.0 / 3.0;
    ret += (150.0 * sin(x / 12.0 * pi) + 300.0 * sin(x / 30.0 * pi)) * 2.0 / 3.0;
    return ret;
}

void gps84_To_Gcj02(double& lat, double& lon) {
    if (outOfChina(lat, lon)) {
        return;
    }
    double dLat = transformLat(lon - 105.0, lat - 35.0);
    double dLon = transformLon(lon - 105.0, lat - 35.0);
    double radLat = lat / 180.0 * pi;
    double magic = sin(radLat);
    magic = 1 - ee * magic * magic;
    double sqrtMagic = sqrt(magic);
    dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * pi);
    dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * pi);
    lat = lat + dLat;
    lon = lon + dLon;
}

    

- (void)recordGPS:(CLLocation *)newLocation{
    if(![self.gps_switch isOn]){
        return;
    }
    if(sync_sensor_time<0){
        return;
    }
    dispatch_async( dispatch_get_main_queue(), ^{
        self.gps_sign.text=[NSString stringWithFormat:@"GPS: %d m" , (int)newLocation.horizontalAccuracy] ;
    } );
    sensor_msgs::NavSatFix msg;
    msg.latitude=newLocation.coordinate.latitude;
    msg.longitude=newLocation.coordinate.longitude;
    gps84_To_Gcj02(msg.latitude, msg.longitude);
    msg.altitude=newLocation.altitude;
    msg.position_covariance[0]=newLocation.horizontalAccuracy;
    msg.position_covariance[3]=newLocation.horizontalAccuracy;
    msg.position_covariance[6]=newLocation.verticalAccuracy;
    static int gps_data_seq=0;
    msg.header.seq=gps_data_seq;
    gps_data_seq++;
    NSDate* eventDate = newLocation.timestamp;

    double time_change = [eventDate timeIntervalSinceDate:sync_sys_time];
    double howRecent = time_change+sync_sensor_time;

    msg.header.stamp = ros::Time(howRecent);
    msg.header.frame_id="map";
    if(is_publishing){
        gps_pub.publish(msg);
    }
    dispatch_async( self.sessionQueue, ^{
        if (is_recording_bag){
            if(bag_ptr->isOpen()){
                NSString *temp_string =  [self.gps_topic_edit.attributedText string];
                NSDate * t1 = [NSDate date];
                NSTimeInterval now = [t1 timeIntervalSince1970];
                bag_ptr->write((char *)[temp_string UTF8String], ros::Time(now), msg);
            }
        }
    });
    
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
                
//                if([name isEqualToString:@"bridge100"]) {
//                    if(![addr isEqualToString:@"0.0.0.0"]){
//                        usbAddress = addr;
//                    }
//                }
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
    if([AVCamManualCameraViewController isValidIp:[self.mater_ip_input.attributedText string]])
    {
        NSString * master_uri = [@"ROS_MASTER_URI=http://" stringByAppendingString:[[self.mater_ip_input.attributedText string] stringByAppendingString:@":11311/"]];
        NSLog(@"%@",master_uri);
        NSString * ip = [@"ROS_IP=" stringByAppendingString:[AVCamManualCameraViewController getIPAddress]];
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
                NSString *temp_string =  [self.imu_topic_edit.attributedText string];
                imu_pub = nn.advertise<sensor_msgs::Imu>((char *)[temp_string UTF8String], 10000, false);
                temp_string =  [self.img_topic_edit.attributedText string];
                img_pub = nn.advertise<sensor_msgs::CompressedImage>((char *)[temp_string UTF8String], 10000, false);
                temp_string =  [self.gps_topic_edit.attributedText string];
                gps_pub = nn.advertise<sensor_msgs::NavSatFix>((char *)[temp_string UTF8String], 10000, false);
                self.master_sign.hidden=NO;
                self.connectButton.enabled=NO;
            }
            else
            {
                UIAlertController *alert = [UIAlertController alertControllerWithTitle:@"Connect fail!" message:@"Check your network !" preferredStyle:UIAlertControllerStyleAlert];
                [self presentViewController:alert animated:YES completion:nil];
                [self performSelector:@selector(dismiss:) withObject:alert afterDelay:2.0];
                return false;
                
            }
        }
        else
        {
            NSLog(@"ROS already initialised. Can'st change the ROS_MASTER_URI");
        }
    }else{
        UIAlertController *alert = [UIAlertController alertControllerWithTitle:@"Connect fail!" message:@"Input a right ip." preferredStyle:UIAlertControllerStyleAlert];
        [self presentViewController:alert animated:YES completion:nil];
        [self performSelector:@selector(dismiss:) withObject:alert afterDelay:2.0];
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
