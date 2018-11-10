#import <UIKit/UIKit.h>
#import <QuartzCore/QuartzCore.h>
#import <CoreMotion/CoreMotion.h>
#import <CoreMedia/CoreMedia.h>
#import <AVFoundation/AVFoundation.h>
#include <deque>
#include <memory>
#include <vector>
#import "AVCamManualPreviewView.h"
#import <ros/ros.h>
#include <rosbag/bag.h>
#include "std_msgs/String.h"

typedef NS_ENUM( NSInteger, AVCamManualSetupResult ) {
    AVCamManualSetupResultSuccess,
    AVCamManualSetupResultCameraNotAuthorized,
    AVCamManualSetupResultSessionConfigurationFailed
};

@interface AVCamManualCameraViewController : UIViewController<AVCaptureVideoDataOutputSampleBufferDelegate>{
    AVCaptureVideoDataOutput *video_output;
    AVCaptureDevice *videoDevice;
    bool need_record;
    int img_count;
    int imu_count;
    bool is_recording_bag;
    bool is_publishing;
    std::vector<std::vector<double> > gyros;
    std::vector<std::vector<double> > acces;
    NSUserDefaults *defaults;
    std::string focus_mode_std;
    std::string exposure_mode_std;
    float cache_focus_posi;
    ros::Publisher img_pub;
    ros::Publisher imu_pub;
    std::shared_ptr<rosbag::Bag> bag_ptr;
    
    
}
@property (strong, nonatomic) IBOutlet UIView *settingPanel;
@property (weak, nonatomic) IBOutlet UISegmentedControl *settingGroupControl;
@property (nonatomic, weak) IBOutlet AVCamManualPreviewView *previewView;
@property (nonatomic, weak) IBOutlet UIButton *recordButton;
@property (nonatomic, weak) IBOutlet UIButton *cameraButton;
@property (nonatomic, weak) IBOutlet UIButton *pubButton;

@property (nonatomic) NSArray *focusModes;
@property (nonatomic, weak) IBOutlet UIView *manualHUDFocusView;
@property (nonatomic, weak) IBOutlet UISegmentedControl *focusModeControl;
@property (nonatomic, weak) IBOutlet UISlider *lensPositionSlider;
@property (nonatomic, weak) IBOutlet UILabel *lensPositionNameLabel;
@property (nonatomic, weak) IBOutlet UILabel *lensPositionValueLabel;

@property (nonatomic) NSArray *exposureModes;
@property (nonatomic, weak) IBOutlet UIView *manualHUDExposureView;
@property (nonatomic, weak) IBOutlet UISegmentedControl *exposureModeControl;
@property (nonatomic, weak) IBOutlet UISlider *exposureDurationSlider;
@property (nonatomic, weak) IBOutlet UILabel *exposureDurationNameLabel;
@property (nonatomic, weak) IBOutlet UILabel *exposureDurationValueLabel;
@property (nonatomic, weak) IBOutlet UISlider *ISOSlider;
@property (nonatomic, weak) IBOutlet UILabel *ISONameLabel;
@property (nonatomic, weak) IBOutlet UILabel *ISOValueLabel;
@property (weak, nonatomic) IBOutlet UIView *topicView;
@property (nonatomic, weak) IBOutlet UIView *manualHUDIPView;

// Session management
@property (nonatomic) dispatch_queue_t sessionQueue;
@property (nonatomic) AVCaptureSession *session;
@property (nonatomic) AVCaptureDeviceInput *videoDeviceInput;
@property (nonatomic) AVCaptureDeviceDiscoverySession *videoDeviceDiscoverySession;
@property (nonatomic) AVCaptureDevice *videoDevice;

// Utilities
@property (nonatomic) AVCamManualSetupResult setupResult;


@end
