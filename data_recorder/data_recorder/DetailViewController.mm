#import "DetailViewController.h"
#include "common_header.h"
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <rosbag/view.h>
#include "common_header.h"
#include <sensor_msgs/NavSatFix.h>

@implementation DetailViewController

UIImage* getFrame(rosbag::Bag& bag,int frame_id, double& timestamp, std::string& topic){
    UIImage *ui_image;
    std::vector<std::string> topics;
    topics.push_back("img");
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int img_count=0;
    rosbag::View::iterator it= view.begin();
    for(;it!=view.end();it++){
        if(img_count==frame_id){
            rosbag::MessageInstance m =*it;
            sensor_msgs::CompressedImagePtr simg = m.instantiate<sensor_msgs::CompressedImage>();
            cv::Mat_<uchar> in(1, simg->data.size(), const_cast<uchar*>(&simg->data[0]));
            cv::Mat rgb_a = cv::imdecode(in, cv::IMREAD_UNCHANGED);
            cv::cvtColor(rgb_a, rgb_a, CV_BGR2BGRA);
            ui_image = [mm_Try UIImageFromCVMat:rgb_a];
            timestamp=simg->header.stamp.toSec();
            break;
        }
        img_count++;
    }
    return ui_image;
}
int getFrameInfo(rosbag::Bag& bag, float& rate, std::string& imu_topic, std::string& img_topic, std::string& gps_topic){
    rosbag::View view(bag);
    rosbag::View::iterator it= view.begin();
    for(;it!=view.end();it++){
        if(img_topic==""){
            rosbag::MessageInstance m =*it;
            if(m.getDataType()=="sensor_msgs/CompressedImage"){
                img_topic=m.getTopic();
            }
        }
        if(imu_topic==""){
            rosbag::MessageInstance m =*it;
            if(m.getDataType()=="sensor_msgs/Imu"){
                imu_topic=m.getTopic();
            }
        }
        if(gps_topic==""){
            rosbag::MessageInstance m =*it;
            if(m.getDataType()=="sensor_msgs/NavSatFix"){
                gps_topic=m.getTopic();
            }
            sensor_msgs::NavSatFixPtr s = m.instantiate<sensor_msgs::NavSatFix>();
        }
        if(gps_topic=="" && imu_topic=="" && img_topic==""){
            break;
        }
    }
    
    std::vector<std::string> topics;
    topics.push_back("img");
    rosbag::View view_img(bag, rosbag::TopicQuery(topics));
    int img_count=0;
    rosbag::View::iterator it_img= view_img.begin();
    for(;it_img!=view_img.end();it_img++){
        img_count++;
    }
    rate = (view_img.getEndTime().toSec()-view_img.getBeginTime().toSec())/(img_count-1);
    return img_count;
}

void getGPSList(rosbag::Bag& bag, LocMap& gps, std::string& topic){
    std::vector<std::string> topics;
    topics.push_back("gps");
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    rosbag::View::iterator it= view.begin();
    for(;it!=view.end();it++){
        rosbag::MessageInstance m =*it;
        sensor_msgs::NavSatFixPtr sgps = m.instantiate<sensor_msgs::NavSatFix>();
        double cur=sgps->header.stamp.toSec();
        CLLocationCoordinate2D coor;
        coor.latitude=sgps->latitude;
        coor.longitude=sgps->longitude;
        gps[cur]=coor;
    }
}

void interGPS(double time_inter, CLLocationCoordinate2D& inter_gps, LocMap& gps){
    LocMap::iterator it= gps.lower_bound(time_inter);
    if(it==gps.begin()){
        inter_gps=gps.begin()->second;
        return;
    }
    double low_time=it->first;
    CLLocationCoordinate2D low_loc=it->second;
    it--;
    double upper_time=it->first;
    CLLocationCoordinate2D upper_loc=it->second;
    inter_gps.longitude=low_loc.longitude+(upper_loc.longitude-low_loc.longitude)*(time_inter-low_time)/(upper_time-low_time);
    inter_gps.latitude=low_loc.latitude+(upper_loc.latitude-low_loc.latitude)*(time_inter-low_time)/(upper_time-low_time);
}

- (void)updateGPS:(double)timestamp {
    CLLocationCoordinate2D inter_gps;
    interGPS(timestamp, inter_gps, gps);
    if(self.me!=NULL){
        [self.map_view removeOverlay:self.me];
    }
    self.me = [MKCircle circleWithCenterCoordinate:inter_gps radius:2];
    [self.map_view addOverlay:self.me];
}

- (IBAction)exit_btn:(id)sender {
    [self dismissViewControllerAnimated:false completion: nil];
}

-(void)viewDidDisappear:(BOOL)animated{
    bag.close();
}
- (IBAction)switch_change:(id)sender {
    normal_speed=self.speed_switch.on;
}

- (void)viewDidLoad
{
	[super viewDidLoad];
    normal_speed=self.speed_switch.on;
    self.sessionQueue = dispatch_queue_create( "session queue", DISPATCH_QUEUE_SERIAL );
    cur_frame_id=0;
    is_playing=false;
    total_img_count=0;
    self.me=NULL;
    NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSArray *directoryContent = [[NSFileManager defaultManager] contentsOfDirectoryAtPath:[dirPaths objectAtIndex:0] error:NULL];
    std::map<std::string, int> re_dict;
    [self.map_view setDelegate:self];
    for (int count = 0; count < (int)[directoryContent count]; count++)
    {
        NSError *error = nil;
        NSString *full_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:[directoryContent objectAtIndex:count]];
        if([self.filename isEqualToString:[directoryContent objectAtIndex:count]]==YES){
            bag_full_addr=std::string([full_addr UTF8String]);
            bag.open(bag_full_addr.c_str(), rosbag::bagmode::Read);
            if(!bag.isOpen()){
                return;
            }
            total_img_count=getFrameInfo(bag, img_rate, imu_topic, img_topic, gps_topic);
            double timestamp;
            self.image_view.image=getFrame(bag, cur_frame_id, timestamp, img_topic);
            self.slider_bar.maximumValue = total_img_count-1;
            getGPSList(bag, gps, gps_topic);
            if(gps.size()>=2){
                std::vector<CLLocationCoordinate2D> coordinateArray;
                LocMap::iterator it=gps.begin();
                for(;it!=gps.end(); it++){
                    coordinateArray.push_back(it->second);
                }
                self.routeLine = [MKPolyline polylineWithCoordinates:(CLLocationCoordinate2D *)coordinateArray.data() count:coordinateArray.size()];
                [self.map_view setVisibleMapRect:[self.routeLine boundingMapRect]]; //If you want the route to be visible
                [self.map_view addOverlay:self.routeLine];
                [self updateGPS:timestamp];
            }
        }
    }
}

- (MKOverlayRenderer *)mapView:(MKMapView *)mapView rendererForOverlay:(id<MKOverlay>)overlay
{
    if(overlay == self.routeLine){
        MKPolylineRenderer *renderer = [[MKPolylineRenderer alloc] initWithPolyline:overlay];
        renderer.strokeColor = [[UIColor blueColor] colorWithAlphaComponent:0.7];
        renderer.lineWidth   = 3;
        return renderer;
    }
    if(overlay == self.me){
        MKCircleRenderer *renderer = [[MKCircleRenderer alloc] initWithCircle:overlay];
        renderer.fillColor   = [[UIColor cyanColor] colorWithAlphaComponent:0.2];
        renderer.strokeColor = [[UIColor blueColor] colorWithAlphaComponent:0.7];
        renderer.lineWidth   = 3;
        return renderer;
    }
    
    
    return nil;
}

- (IBAction)play_btn:(id)sender {
    if(is_playing==false){
        [sender setTitle:@"Stop" forState:UIControlStateNormal];
        is_playing=true;
        self.slider_bar.userInteractionEnabled=false;
        
        dispatch_async( self.sessionQueue, ^{
            for(;cur_frame_id<total_img_count; cur_frame_id++){
                double timestamp;
                NSDate * t1= [NSDate date];
                UIImage* image=getFrame(bag, cur_frame_id, timestamp, img_topic);
                NSDate * t2= [NSDate date];
                double t12=[t2 timeIntervalSinceDate: t1];
                dispatch_async( dispatch_get_main_queue(), ^{
                    self.image_view.image=image;
                    self.slider_bar.value=cur_frame_id;
                    [self updateGPS:timestamp];
                } );
                
                if (normal_speed){
                    if(img_rate- t12>0){
                        [NSThread sleepForTimeInterval:img_rate- t12];
                    }
                }
                if(is_playing==false){
                    break;
                }
            }
        } );
    }else{
        self.slider_bar.userInteractionEnabled=true;
        [sender setTitle:@"Play" forState:UIControlStateNormal];
        is_playing=false;
    }
    
}
- (IBAction)change_slider:(id)sender {
    cur_frame_id=self.slider_bar.value;
    if(last_slider_frame==cur_frame_id){
        return;
    }
    last_slider_frame=cur_frame_id;
    double timestamp;
    self.image_view.image=getFrame(bag, cur_frame_id, timestamp, img_topic);
    [self updateGPS:timestamp];
}


@end
