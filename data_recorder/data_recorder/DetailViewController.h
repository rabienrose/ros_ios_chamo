#import <UIKit/UIKit.h>
#import <AVFoundation/AVFoundation.h>
#include <deque>
#include <memory>
#include <vector>
#import <ros/ros.h>
#import <MapKit/MapKit.h>
#include <rosbag/bag.h>
#import <CoreLocation/CoreLocation.h>
typedef std::map<double, CLLocationCoordinate2D, std::greater<double>> LocMap;
@interface DetailViewController : UIViewController<MKMapViewDelegate>{
    std::string bag_full_addr;
    int total_img_count;
    float img_rate;
    int cur_frame_id;
    bool is_playing;
    int last_slider_frame;
    LocMap gps;
    rosbag::Bag bag;
    std::string imu_topic;
    std::string img_topic;
    std::string gps_topic;
    bool normal_speed;
}
@property (weak, nonatomic) IBOutlet UISwitch *speed_switch;
@property (weak, nonatomic) IBOutlet MKMapView *map_view;
@property (weak, nonatomic) IBOutlet UIImageView *image_view;
@property (weak, nonatomic) IBOutlet UISlider *slider_bar;
@property (nonatomic, strong) NSString *filename;
@property (nonatomic) dispatch_queue_t sessionQueue;
@property (weak, nonatomic) IBOutlet UIButton *play_btn;
@property (nonatomic, retain) MKPolyline *routeLine;
@property (nonatomic, retain) MKCircle *me;

@end
