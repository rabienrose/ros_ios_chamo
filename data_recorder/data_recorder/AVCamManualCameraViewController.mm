#import "AVCamManualCameraViewController.h"
#import "AVCamManualPreviewView.h"
#import <ifaddrs.h>
#import <arpa/inet.h>
#include "common_header.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <iomanip>

static void * SessionRunningContext = &SessionRunningContext;
static void * FocusModeContext = &FocusModeContext;
static void * ExposureModeContext = &ExposureModeContext;
static void * WhiteBalanceModeContext = &WhiteBalanceModeContext;
static void * LensPositionContext = &LensPositionContext;
static void * ExposureDurationContext = &ExposureDurationContext;
static void * ISOContext = &ISOContext;
static void * ExposureTargetBiasContext = &ExposureTargetBiasContext;
static void * ExposureTargetOffsetContext = &ExposureTargetOffsetContext;
static void * DeviceWhiteBalanceGainsContext = &DeviceWhiteBalanceGainsContext;

typedef NS_ENUM( NSInteger, AVCamManualSetupResult ) {
	AVCamManualSetupResultSuccess,
	AVCamManualSetupResultCameraNotAuthorized,
	AVCamManualSetupResultSessionConfigurationFailed
};

typedef NS_ENUM( NSInteger, AVCamManualCaptureMode ) {
	AVCamManualCaptureModePhoto = 0,
	AVCamManualCaptureModeMovie = 1
};

@interface AVCamManualCameraViewController ()

@property (nonatomic, weak) IBOutlet AVCamManualPreviewView *previewView;
@property (nonatomic, weak) IBOutlet UIButton *recordButton;
@property (nonatomic, weak) IBOutlet UIButton *cameraButton;
@property (nonatomic, weak) IBOutlet UIButton *pubButton;
@property (nonatomic, weak) IBOutlet UIButton *HUDButton;

@property (nonatomic, weak) IBOutlet UIView *manualHUD;

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
@property (nonatomic, weak) IBOutlet UISlider *exposureTargetBiasSlider;
@property (nonatomic, weak) IBOutlet UILabel *exposureTargetBiasNameLabel;
@property (nonatomic, weak) IBOutlet UILabel *exposureTargetBiasValueLabel;
@property (nonatomic, weak) IBOutlet UISlider *exposureTargetOffsetSlider;
@property (nonatomic, weak) IBOutlet UILabel *exposureTargetOffsetNameLabel;
@property (nonatomic, weak) IBOutlet UILabel *exposureTargetOffsetValueLabel;

@property (nonatomic) NSArray *whiteBalanceModes;
@property (nonatomic, weak) IBOutlet UIView *manualHUDWhiteBalanceView;
@property (nonatomic, weak) IBOutlet UISegmentedControl *whiteBalanceModeControl;
@property (nonatomic, weak) IBOutlet UISlider *temperatureSlider;
@property (nonatomic, weak) IBOutlet UILabel *temperatureNameLabel;
@property (nonatomic, weak) IBOutlet UILabel *temperatureValueLabel;
@property (nonatomic, weak) IBOutlet UISlider *tintSlider;
@property (nonatomic, weak) IBOutlet UILabel *tintNameLabel;
@property (nonatomic, weak) IBOutlet UILabel *tintValueLabel;

@property (nonatomic, weak) IBOutlet UIView *manualHUDOtherView;
@property (nonatomic, weak) IBOutlet UISegmentedControl *lensStabilizationControl;

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

CMMotionManager *motionManager;

@implementation AVCamManualCameraViewController

static const float kExposureDurationPower = 5; // Higher numbers will give the slider more sensitivity at shorter durations
static const float kExposureMinimumDuration = 1.0/1000; // Limit exposure duration to a useful range

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

#pragma mark View Controller Life Cycle

- (void)viewDidLoad
{
	[super viewDidLoad];
    need_record=true;
    is_recording_bag=false;
    is_publishing=false;

	// Disable UI until the session starts running
	self.cameraButton.enabled = YES;
	self.recordButton.enabled = YES;
	self.pubButton.enabled = NO;
	self.HUDButton.enabled = YES;
	
	self.manualHUD.hidden = YES;
	self.manualHUDIPView.hidden = YES;
	self.manualHUDFocusView.hidden = YES;
	self.manualHUDExposureView.hidden = YES;
	self.manualHUDWhiteBalanceView.hidden = YES;
	self.manualHUDOtherView.hidden = YES;
	
	// Create the AVCaptureSession
	self.session = [[AVCaptureSession alloc] init];

	// Create a device discovery session
	NSArray<NSString *> *deviceTypes = @[AVCaptureDeviceTypeBuiltInWideAngleCamera, AVCaptureDeviceTypeBuiltInDuoCamera, AVCaptureDeviceTypeBuiltInTelephotoCamera];
	self.videoDeviceDiscoverySession = [AVCaptureDeviceDiscoverySession discoverySessionWithDeviceTypes:deviceTypes mediaType:AVMediaTypeVideo position:AVCaptureDevicePositionUnspecified];

	// Set up the preview view
	self.previewView.session = self.session;
	
	// Communicate with the session and other session objects on this queue
    NSOperationQueue *quene =[[NSOperationQueue alloc] init];
    quene.maxConcurrentOperationCount=1;
	self.sessionQueue = dispatch_queue_create( "session queue", DISPATCH_QUEUE_SERIAL );
    //self.sessionQueue = quene.underlyingQueue;

	self.setupResult = AVCamManualSetupResultSuccess;
	// Check video authorization status. Video access is required and audio access is optional.
	// If audio access is denied, audio is not recorded during movie recording.
	switch ( [AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo] )
	{
		case AVAuthorizationStatusAuthorized:
		{
			// The user has previously granted access to the camera
			break;
		}
		case AVAuthorizationStatusNotDetermined:
		{
			dispatch_suspend( self.sessionQueue );
			[AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo completionHandler:^( BOOL granted ) {
				if ( ! granted ) {
					self.setupResult = AVCamManualSetupResultCameraNotAuthorized;
				}
				dispatch_resume( self.sessionQueue );
			}];
			break;
		}
		default:
		{
			self.setupResult = AVCamManualSetupResultCameraNotAuthorized;
			break;
		}
	}

    dispatch_async( self.sessionQueue, ^{
        [self configureSession];
    } );
    
    motionManager = [[CMMotionManager alloc] init];
    if (motionManager.accelerometerAvailable){
        motionManager.accelerometerUpdateInterval =0.1;
        [motionManager
         startAccelerometerUpdatesToQueue:quene
         withHandler:
         ^(CMAccelerometerData *data, NSError *error){
             //NSLog(@"acce: %@", [NSThread currentThread]);
             if(need_record==true){
                 std::vector<double> imu;
                 imu.resize(5);
                 imu[0]=-data.acceleration.x*9.8;
                 imu[1]=-data.acceleration.y*9.8;
                 imu[2]=-data.acceleration.z*9.8;
                 imu[3]=data.timestamp;
                 imu[4]=0;
                 acces.push_back(imu);
             }
         }];
    }
    if (motionManager.gyroAvailable){
        motionManager.gyroUpdateInterval =0.1;
        [motionManager
         startGyroUpdatesToQueue:quene
         withHandler:
         ^(CMGyroData *data, NSError *error){
             //NSLog(@"gyro: %@", [NSThread currentThread]);
             if(need_record== true){
                 std::vector<double> imu;
                 imu.resize(5);
                 imu[0]=data.rotationRate.x;
                 imu[1]=data.rotationRate.y;
                 imu[2]=data.rotationRate.z;
                 imu[3]=data.timestamp;
                 imu[4]=0;
                 gyros.push_back(imu);
                 [self processIMU_gyro];
             }
         }];
    }
}

- (void)viewWillAppear:(BOOL)animated
{
	[super viewWillAppear:animated];

	dispatch_async( self.sessionQueue, ^{
		switch ( self.setupResult )
		{
			case AVCamManualSetupResultSuccess:
			{
				// Only setup observers and start the session running if setup succeeded
				[self addObservers];
				[self.session startRunning];
				break;
			}
			case AVCamManualSetupResultCameraNotAuthorized:
			{
				dispatch_async( dispatch_get_main_queue(), ^{
					NSString *message = NSLocalizedString( @"AVCamManual doesn't have permission to use the camera, please change privacy settings", @"Alert message when the user has denied access to the camera" );
					UIAlertController *alertController = [UIAlertController alertControllerWithTitle:@"AVCamManual" message:message preferredStyle:UIAlertControllerStyleAlert];
					UIAlertAction *cancelAction = [UIAlertAction actionWithTitle:NSLocalizedString( @"OK", @"Alert OK button" ) style:UIAlertActionStyleCancel handler:nil];
					[alertController addAction:cancelAction];
					// Provide quick access to Settings
					UIAlertAction *settingsAction = [UIAlertAction actionWithTitle:NSLocalizedString( @"Settings", @"Alert button to open Settings" ) style:UIAlertActionStyleDefault handler:^( UIAlertAction *action ) {
						[[UIApplication sharedApplication] openURL:[NSURL URLWithString:UIApplicationOpenSettingsURLString] options:@{} completionHandler:nil];
					}];
					[alertController addAction:settingsAction];
					[self presentViewController:alertController animated:YES completion:nil];
				} );
				break;
			}
			case AVCamManualSetupResultSessionConfigurationFailed:
			{
				dispatch_async( dispatch_get_main_queue(), ^{
					NSString *message = NSLocalizedString( @"Unable to capture media", @"Alert message when something goes wrong during capture session configuration" );
					UIAlertController *alertController = [UIAlertController alertControllerWithTitle:@"AVCamManual" message:message preferredStyle:UIAlertControllerStyleAlert];
					UIAlertAction *cancelAction = [UIAlertAction actionWithTitle:NSLocalizedString( @"OK", @"Alert OK button" ) style:UIAlertActionStyleCancel handler:nil];
					[alertController addAction:cancelAction];
					[self presentViewController:alertController animated:YES completion:nil];
				} );
				break;
			}
		}
	} );
}

- (void)viewDidDisappear:(BOOL)animated
{
	dispatch_async( self.sessionQueue, ^{
		if ( self.setupResult == AVCamManualSetupResultSuccess ) {
			[self.session stopRunning];
			[self removeObservers];
		}
	} );

	[super viewDidDisappear:animated];
}

- (void)viewWillTransitionToSize:(CGSize)size withTransitionCoordinator:(id<UIViewControllerTransitionCoordinator>)coordinator
{
	[super viewWillTransitionToSize:size withTransitionCoordinator:coordinator];
	
	UIDeviceOrientation deviceOrientation = [UIDevice currentDevice].orientation;
	
	if ( UIDeviceOrientationIsPortrait( deviceOrientation ) || UIDeviceOrientationIsLandscape( deviceOrientation ) ) {
		AVCaptureVideoPreviewLayer *previewLayer = (AVCaptureVideoPreviewLayer *)self.previewView.layer;
		previewLayer.connection.videoOrientation = (AVCaptureVideoOrientation)deviceOrientation;
	}
}

- (UIInterfaceOrientationMask)supportedInterfaceOrientations
{
    return UIInterfaceOrientationMaskAll;
}

- (BOOL)shouldAutorotate
{
    return YES;
}

- (BOOL)prefersStatusBarHidden
{
    return YES;
}

#pragma mark HUD

- (void)configureManualHUD
{
    // Manual focus controls
    self.focusModes = @[@(AVCaptureFocusModeContinuousAutoFocus), @(AVCaptureFocusModeLocked)];
    
    self.focusModeControl.enabled = ( self.videoDevice != nil );
    self.focusModeControl.selectedSegmentIndex = [self.focusModes indexOfObject:@(self.videoDevice.focusMode)];
    for ( NSNumber *mode in self.focusModes ) {
        [self.focusModeControl setEnabled:[self.videoDevice isFocusModeSupported:(AVCaptureFocusMode)mode.intValue] forSegmentAtIndex:[self.focusModes indexOfObject:mode]];
    }
    
    self.lensPositionSlider.minimumValue = 0.0;
    self.lensPositionSlider.maximumValue = 1.0;
    self.lensPositionSlider.value = self.videoDevice.lensPosition;
    self.lensPositionSlider.enabled = ( self.videoDevice && self.videoDevice.focusMode == AVCaptureFocusModeLocked && [self.videoDevice isFocusModeSupported:AVCaptureFocusModeLocked] );
    
    // Manual exposure controls
    self.exposureModes = @[@(AVCaptureExposureModeContinuousAutoExposure), @(AVCaptureExposureModeLocked), @(AVCaptureExposureModeCustom)];
    
    
    self.exposureModeControl.enabled = ( self.videoDevice != nil );
    self.exposureModeControl.selectedSegmentIndex = [self.exposureModes indexOfObject:@(self.videoDevice.exposureMode)];
    for ( NSNumber *mode in self.exposureModes ) {
        [self.exposureModeControl setEnabled:[self.videoDevice isExposureModeSupported:(AVCaptureExposureMode)mode.intValue] forSegmentAtIndex:[self.exposureModes indexOfObject:mode]];
    }
    
    // Use 0-1 as the slider range and do a non-linear mapping from the slider value to the actual device exposure duration
    self.exposureDurationSlider.minimumValue = 0;
    self.exposureDurationSlider.maximumValue = 1;
    double exposureDurationSeconds = CMTimeGetSeconds( self.videoDevice.exposureDuration );
    double minExposureDurationSeconds = MAX( CMTimeGetSeconds( self.videoDevice.activeFormat.minExposureDuration ), kExposureMinimumDuration );
    double maxExposureDurationSeconds = CMTimeGetSeconds( self.videoDevice.activeFormat.maxExposureDuration );
    // Map from duration to non-linear UI range 0-1
    double p = ( exposureDurationSeconds - minExposureDurationSeconds ) / ( maxExposureDurationSeconds - minExposureDurationSeconds ); // Scale to 0-1
    self.exposureDurationSlider.value = pow( p, 1 / kExposureDurationPower ); // Apply inverse power
    self.exposureDurationSlider.enabled = ( self.videoDevice && self.videoDevice.exposureMode == AVCaptureExposureModeCustom );
    
    self.ISOSlider.minimumValue = self.videoDevice.activeFormat.minISO;
    self.ISOSlider.maximumValue = self.videoDevice.activeFormat.maxISO;
    self.ISOSlider.value = self.videoDevice.ISO;
    self.ISOSlider.enabled = ( self.videoDevice.exposureMode == AVCaptureExposureModeCustom );
    
    self.exposureTargetBiasSlider.minimumValue = self.videoDevice.minExposureTargetBias;
    self.exposureTargetBiasSlider.maximumValue = self.videoDevice.maxExposureTargetBias;
    self.exposureTargetBiasSlider.value = self.videoDevice.exposureTargetBias;
    self.exposureTargetBiasSlider.enabled = ( self.videoDevice != nil );
    
    self.exposureTargetOffsetSlider.minimumValue = self.videoDevice.minExposureTargetBias;
    self.exposureTargetOffsetSlider.maximumValue = self.videoDevice.maxExposureTargetBias;
    self.exposureTargetOffsetSlider.value = self.videoDevice.exposureTargetOffset;
    self.exposureTargetOffsetSlider.enabled = NO;
    
    // Manual white balance controls
    self.whiteBalanceModes = @[@(AVCaptureWhiteBalanceModeContinuousAutoWhiteBalance), @(AVCaptureWhiteBalanceModeLocked)];
    
    self.whiteBalanceModeControl.enabled = (self.videoDevice != nil);
    self.whiteBalanceModeControl.selectedSegmentIndex = [self.whiteBalanceModes indexOfObject:@(self.videoDevice.whiteBalanceMode)];
    for ( NSNumber *mode in self.whiteBalanceModes ) {
        [self.whiteBalanceModeControl setEnabled:[self.videoDevice isWhiteBalanceModeSupported:(AVCaptureWhiteBalanceMode)mode.intValue] forSegmentAtIndex:[self.whiteBalanceModes indexOfObject:mode]];
    }
    
    AVCaptureWhiteBalanceGains whiteBalanceGains = self.videoDevice.deviceWhiteBalanceGains;
    AVCaptureWhiteBalanceTemperatureAndTintValues whiteBalanceTemperatureAndTint = [self.videoDevice temperatureAndTintValuesForDeviceWhiteBalanceGains:whiteBalanceGains];
    
    self.temperatureSlider.minimumValue = 3000;
    self.temperatureSlider.maximumValue = 8000;
    self.temperatureSlider.value = whiteBalanceTemperatureAndTint.temperature;
    self.temperatureSlider.enabled = ( self.videoDevice && self.videoDevice.whiteBalanceMode == AVCaptureWhiteBalanceModeLocked );
    
    self.tintSlider.minimumValue = -150;
    self.tintSlider.maximumValue = 150;
    self.tintSlider.value = whiteBalanceTemperatureAndTint.tint;
    self.tintSlider.enabled = ( self.videoDevice && self.videoDevice.whiteBalanceMode == AVCaptureWhiteBalanceModeLocked );
    
    self.lensStabilizationControl.enabled = ( self.videoDevice != nil );
    self.lensStabilizationControl.selectedSegmentIndex = 0;
    
}

- (IBAction)toggleHUD:(id)sender
{
    self.manualHUD.hidden = ! self.manualHUD.hidden;
}

- (IBAction)changeManualHUD:(id)sender
{
    UISegmentedControl *control = sender;
    
    self.manualHUDIPView.hidden = ( control.selectedSegmentIndex == 0 ) ? NO : YES;
    self.manualHUDFocusView.hidden = ( control.selectedSegmentIndex == 1 ) ? NO : YES;
    self.manualHUDExposureView.hidden = ( control.selectedSegmentIndex == 2 ) ? NO : YES;
    self.manualHUDWhiteBalanceView.hidden = ( control.selectedSegmentIndex == 3 ) ? NO : YES;
    self.manualHUDOtherView.hidden = ( control.selectedSegmentIndex == 4 ) ? NO : YES;
}

- (void)setSlider:(UISlider *)slider highlightColor:(UIColor *)color
{
    slider.tintColor = color;
    
    if ( slider == self.lensPositionSlider ) {
        self.lensPositionNameLabel.textColor = self.lensPositionValueLabel.textColor = slider.tintColor;
    }
    else if ( slider == self.exposureDurationSlider ) {
        self.exposureDurationNameLabel.textColor = self.exposureDurationValueLabel.textColor = slider.tintColor;
    }
    else if ( slider == self.ISOSlider ) {
        self.ISONameLabel.textColor = self.ISOValueLabel.textColor = slider.tintColor;
    }
    else if ( slider == self.exposureTargetBiasSlider ) {
        self.exposureTargetBiasNameLabel.textColor = self.exposureTargetBiasValueLabel.textColor = slider.tintColor;
    }
    else if ( slider == self.temperatureSlider ) {
        self.temperatureNameLabel.textColor = self.temperatureValueLabel.textColor = slider.tintColor;
    }
    else if ( slider == self.tintSlider ) {
        self.tintNameLabel.textColor = self.tintValueLabel.textColor = slider.tintColor;
    }
}

- (IBAction)sliderTouchBegan:(id)sender
{
    UISlider *slider = (UISlider *)sender;
    [self setSlider:slider highlightColor:[UIColor colorWithRed:0.0 green:122.0/255.0 blue:1.0 alpha:1.0]];
}

- (IBAction)sliderTouchEnded:(id)sender
{
    UISlider *slider = (UISlider *)sender;
    [self setSlider:slider highlightColor:[UIColor yellowColor]];
}

- (void)captureOutput:(AVCaptureOutput *)captureOutput
didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
       fromConnection:(AVCaptureConnection *)connection {
    //NSLog(@"img: %@", [NSThread currentThread]);
    //NSLog(@"img: %f", timestamp.value/(double)timestamp.timescale);
    //if(false){
    if(need_record==true){
        
        CMTime timestamp = CMSampleBufferGetPresentationTimeStamp(sampleBuffer);
        //NSLog(@"%f", timestamp.value/(double)timestamp.timescale);
        UIImage *image = [self imageFromSampleBuffer:sampleBuffer];
        cv::Mat img_cv = [mm_Try cvMatFromUIImage:image];
        float time_in_sec = timestamp.value/(double)timestamp.timescale;
        
        sensor_msgs::CompressedImage img_ros_img;
        //cv::Mat img_gray;
        //cv::cvtColor(img_cv, img_gray, CV_BGRA2GRAY);
        std::vector<unsigned char> binaryBuffer_;
        cv::imencode(".jpg", img_cv, binaryBuffer_);
        img_ros_img.data=binaryBuffer_;
        img_ros_img.header.seq=img_count;
        img_ros_img.header.stamp= ros::Time(time_in_sec);
        img_ros_img.format="jpeg";
        //        static double lasttime=-1;
        //        if(lasttime<0){
        //            lasttime=time_in_sec;
        //        }else{
        //            std::cout<<time_in_sec - lasttime<<std::endl;
        //            lasttime=time_in_sec;
        //        }
        
        //        sensor_msgs::Image img_ros_img;
        //        cv::cvtColor(img_cv, img_gray, CV_BGRA2GRAY);
        //        img_ros_img.height=img_gray.rows;
        //        img_ros_img.width=img_gray.cols;
        //        img_ros_img.encoding=sensor_msgs::image_encodings::MONO8;
        //        img_ros_img.is_bigendian=false;
        //        img_ros_img.step=1*img_ros_img.width;
        //        img_ros_img.data.assign(img_gray.data, img_gray.data+img_ros_img.step*img_ros_img.height);
        //
        //        img_ros_img.header.seq=img_count;
        //        img_ros_img.header.stamp= ros::Time(time_in_sec);
        if(is_publishing){
            img_pub.publish(img_ros_img);
        }
        dispatch_async( self.sessionQueue, ^{
            if(is_recording_bag){
                if(bag_ptr->isOpen()){
                    bag_ptr->write("/cam0", img_ros_img.header.stamp, img_ros_img);
                }
            }
        });
        
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

#pragma mark Session Management

// Should be called on the session queue
- (void)configureSession
{
    if ( self.setupResult != AVCamManualSetupResultSuccess ) {
        return;
    }
    
    NSError *error = nil;
    
    [self.session beginConfiguration];
    
    self.session.sessionPreset = AVCaptureSessionPreset640x480;
    
    // Add video input
    videoDevice = [AVCaptureDevice defaultDeviceWithDeviceType:AVCaptureDeviceTypeBuiltInWideAngleCamera mediaType:AVMediaTypeVideo position:AVCaptureDevicePositionUnspecified];
    AVCaptureDeviceInput *videoDeviceInput = [AVCaptureDeviceInput deviceInputWithDevice:videoDevice error:&error];
    if ( ! videoDeviceInput ) {
        NSLog( @"Could not create video device input: %@", error );
        self.setupResult = AVCamManualSetupResultSessionConfigurationFailed;
        [self.session commitConfiguration];
        return;
    }
    if ( [self.session canAddInput:videoDeviceInput] ) {
        [self.session addInput:videoDeviceInput];
        self.videoDeviceInput = videoDeviceInput;
        self.videoDevice = videoDevice;
        
        dispatch_async( dispatch_get_main_queue(), ^{
            UIInterfaceOrientation statusBarOrientation = [UIApplication sharedApplication].statusBarOrientation;
            AVCaptureVideoOrientation initialVideoOrientation = AVCaptureVideoOrientationPortrait;
            if ( statusBarOrientation != UIInterfaceOrientationUnknown ) {
                initialVideoOrientation = (AVCaptureVideoOrientation)statusBarOrientation;
            }
            
            AVCaptureVideoPreviewLayer *previewLayer = (AVCaptureVideoPreviewLayer *)self.previewView.layer;
            previewLayer.connection.videoOrientation = initialVideoOrientation;
        } );
    }
    else {
        NSLog( @"Could not add video device input to the session" );
        self.setupResult = AVCamManualSetupResultSessionConfigurationFailed;
        [self.session commitConfiguration];
        return;
    }
    
    video_output = [[AVCaptureVideoDataOutput alloc] init];
    NSDictionary *newSettings = @{ (NSString *)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA) };
    video_output.videoSettings = newSettings;
    [video_output setAlwaysDiscardsLateVideoFrames:YES];
    if ([self.session canAddOutput:video_output]) {
        [self.session addOutput:video_output];
    }else {
        NSLog(@"add output wrong!!!");
    }
    
    [video_output setSampleBufferDelegate:self queue:self.sessionQueue];
    [self.videoDevice setActiveVideoMinFrameDuration:CMTimeMake(1, 10)];

    [self.session commitConfiguration];
    
    dispatch_async( dispatch_get_main_queue(), ^{
        [self configureManualHUD];
    } );
}

#pragma mark Device Configuration

- (IBAction)chooseNewCamera:(id)sender
{
    // Present all available cameras
    UIAlertController *cameraOptionsController = [UIAlertController alertControllerWithTitle:@"Choose a camera" message:nil preferredStyle:UIAlertControllerStyleActionSheet];
    UIAlertAction *cancelAction = [UIAlertAction actionWithTitle:@"Cancel" style:UIAlertActionStyleCancel handler:nil];
    [cameraOptionsController addAction:cancelAction];
    for ( AVCaptureDevice *device in self.videoDeviceDiscoverySession.devices ) {
        UIAlertAction *newDeviceOption = [UIAlertAction actionWithTitle:device.localizedName style:UIAlertActionStyleDefault handler:^(UIAlertAction * _Nonnull action) {
            [self changeCameraWithDevice:device];
        }];
        [cameraOptionsController addAction:newDeviceOption];
    }
    
    [self presentViewController:cameraOptionsController animated:YES completion:nil];
}

- (void)changeCameraWithDevice:(AVCaptureDevice *)newVideoDevice
{
    // Check if device changed
    if ( newVideoDevice == self.videoDevice ) {
        return;
    }
    
    self.manualHUD.userInteractionEnabled = NO;
    self.cameraButton.enabled = NO;
    self.recordButton.enabled = NO;
    self.pubButton.enabled = NO;
    self.HUDButton.enabled = NO;
    
    dispatch_async( self.sessionQueue, ^{
        AVCaptureDeviceInput *newVideoDeviceInput = [AVCaptureDeviceInput deviceInputWithDevice:newVideoDevice error:nil];
        
        [self.session beginConfiguration];
        
        // Remove the existing device input first, since using the front and back camera simultaneously is not supported
        [self.session removeInput:self.videoDeviceInput];
        if ( [self.session canAddInput:newVideoDeviceInput] ) {
            [[NSNotificationCenter defaultCenter] removeObserver:self name:AVCaptureDeviceSubjectAreaDidChangeNotification object:self.videoDevice];
            
            [[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(subjectAreaDidChange:) name:AVCaptureDeviceSubjectAreaDidChangeNotification object:newVideoDevice];
            
            [self.session addInput:newVideoDeviceInput];
            self.videoDeviceInput = newVideoDeviceInput;
            self.videoDevice = newVideoDevice;
        }
        else {
            [self.session addInput:self.videoDeviceInput];
        }
        
        [self.session commitConfiguration];
        
        dispatch_async( dispatch_get_main_queue(), ^{
            [self configureManualHUD];
            
            self.cameraButton.enabled = YES;
            self.recordButton.enabled = YES;
            self.pubButton.enabled = YES;
            self.HUDButton.enabled = YES;
            self.manualHUD.userInteractionEnabled = YES;
        } );
    } );
}

- (IBAction)changeFocusMode:(id)sender
{
    UISegmentedControl *control = sender;
    AVCaptureFocusMode mode = (AVCaptureFocusMode)[self.focusModes[control.selectedSegmentIndex] intValue];
    
    NSError *error = nil;
    
    if ( [self.videoDevice lockForConfiguration:&error] ) {
        if ( [self.videoDevice isFocusModeSupported:mode] ) {
            self.videoDevice.focusMode = mode;
        }
        else {
            NSLog( @"Focus mode %@ is not supported. Focus mode is %@.", [self stringFromFocusMode:mode], [self stringFromFocusMode:self.videoDevice.focusMode] );
            self.focusModeControl.selectedSegmentIndex = [self.focusModes indexOfObject:@(self.videoDevice.focusMode)];
        }
        [self.videoDevice unlockForConfiguration];
    }
    else {
        NSLog( @"Could not lock device for configuration: %@", error );
    }
}

- (IBAction)changeLensPosition:(id)sender
{
    UISlider *control = sender;
    NSError *error = nil;
    
    if ( [self.videoDevice lockForConfiguration:&error] ) {
        [self.videoDevice setFocusModeLockedWithLensPosition:control.value completionHandler:nil];
        [self.videoDevice unlockForConfiguration];
    }
    else {
        NSLog( @"Could not lock device for configuration: %@", error );
    }
}

- (void)focusWithMode:(AVCaptureFocusMode)focusMode exposeWithMode:(AVCaptureExposureMode)exposureMode atDevicePoint:(CGPoint)point monitorSubjectAreaChange:(BOOL)monitorSubjectAreaChange
{
    dispatch_async( self.sessionQueue, ^{
        AVCaptureDevice *device = self.videoDevice;
        
        NSError *error = nil;
        if ( [device lockForConfiguration:&error] ) {
            // Setting (focus/exposure)PointOfInterest alone does not initiate a (focus/exposure) operation
            // Call -set(Focus/Exposure)Mode: to apply the new point of interest
            if ( focusMode != AVCaptureFocusModeLocked && device.isFocusPointOfInterestSupported && [device isFocusModeSupported:focusMode] ) {
                device.focusPointOfInterest = point;
                device.focusMode = focusMode;
            }
            
            if ( exposureMode != AVCaptureExposureModeCustom && device.isExposurePointOfInterestSupported && [device isExposureModeSupported:exposureMode] ) {
                device.exposurePointOfInterest = point;
                device.exposureMode = exposureMode;
            }
            
            device.subjectAreaChangeMonitoringEnabled = monitorSubjectAreaChange;
            [device unlockForConfiguration];
        }
        else {
            NSLog( @"Could not lock device for configuration: %@", error );
        }
    } );
}

- (IBAction)focusAndExposeTap:(UIGestureRecognizer *)gestureRecognizer
{
    CGPoint devicePoint = [(AVCaptureVideoPreviewLayer *)self.previewView.layer captureDevicePointOfInterestForPoint:[gestureRecognizer locationInView:[gestureRecognizer view]]];
    [self focusWithMode:self.videoDevice.focusMode exposeWithMode:self.videoDevice.exposureMode atDevicePoint:devicePoint monitorSubjectAreaChange:YES];
}

- (IBAction)changeExposureMode:(id)sender
{
    UISegmentedControl *control = sender;
    AVCaptureExposureMode mode = (AVCaptureExposureMode)[self.exposureModes[control.selectedSegmentIndex] intValue];
    NSError *error = nil;
    
    if ( [self.videoDevice lockForConfiguration:&error] ) {
        if ( [self.videoDevice isExposureModeSupported:mode] ) {
            self.videoDevice.exposureMode = mode;
        }
        else {
            NSLog( @"Exposure mode %@ is not supported. Exposure mode is %@.", [self stringFromExposureMode:mode], [self stringFromExposureMode:self.videoDevice.exposureMode] );
            self.exposureModeControl.selectedSegmentIndex = [self.exposureModes indexOfObject:@(self.videoDevice.exposureMode)];
        }
        [self.videoDevice unlockForConfiguration];
    }
    else {
        NSLog( @"Could not lock device for configuration: %@", error );
    }
}

- (IBAction)changeExposureDuration:(id)sender
{
    UISlider *control = sender;
    NSError *error = nil;
    
    double p = pow( control.value, kExposureDurationPower ); // Apply power function to expand slider's low-end range
    double minDurationSeconds = MAX( CMTimeGetSeconds( self.videoDevice.activeFormat.minExposureDuration ), kExposureMinimumDuration );
    double maxDurationSeconds = CMTimeGetSeconds( self.videoDevice.activeFormat.maxExposureDuration );
    double newDurationSeconds = p * ( maxDurationSeconds - minDurationSeconds ) + minDurationSeconds; // Scale from 0-1 slider range to actual duration
    
    if ( [self.videoDevice lockForConfiguration:&error] ) {
        [self.videoDevice setExposureModeCustomWithDuration:CMTimeMakeWithSeconds( newDurationSeconds, 1000*1000*1000 )  ISO:AVCaptureISOCurrent completionHandler:nil];
        [self.videoDevice unlockForConfiguration];
    }
    else {
        NSLog( @"Could not lock device for configuration: %@", error );
    }
}

- (IBAction)changeISO:(id)sender
{
    UISlider *control = sender;
    NSError *error = nil;
    
    if ( [self.videoDevice lockForConfiguration:&error] ) {
        [self.videoDevice setExposureModeCustomWithDuration:AVCaptureExposureDurationCurrent ISO:control.value completionHandler:nil];
        [self.videoDevice unlockForConfiguration];
    }
    else {
        NSLog( @"Could not lock device for configuration: %@", error );
    }
}

- (IBAction)changeExposureTargetBias:(id)sender
{
    UISlider *control = sender;
    NSError *error = nil;
    
    if ( [self.videoDevice lockForConfiguration:&error] ) {
        [self.videoDevice setExposureTargetBias:control.value completionHandler:nil];
        [self.videoDevice unlockForConfiguration];
    }
    else {
        NSLog( @"Could not lock device for configuration: %@", error );
    }
}

- (IBAction)changeWhiteBalanceMode:(id)sender
{
    UISegmentedControl *control = sender;
    AVCaptureWhiteBalanceMode mode = (AVCaptureWhiteBalanceMode)[self.whiteBalanceModes[control.selectedSegmentIndex] intValue];
    NSError *error = nil;
    
    if ( [self.videoDevice lockForConfiguration:&error] ) {
        if ( [self.videoDevice isWhiteBalanceModeSupported:mode] ) {
            self.videoDevice.whiteBalanceMode = mode;
        }
        else {
            NSLog( @"White balance mode %@ is not supported. White balance mode is %@.", [self stringFromWhiteBalanceMode:mode], [self stringFromWhiteBalanceMode:self.videoDevice.whiteBalanceMode] );
            self.whiteBalanceModeControl.selectedSegmentIndex = [self.whiteBalanceModes indexOfObject:@(self.videoDevice.whiteBalanceMode)];
        }
        [self.videoDevice unlockForConfiguration];
    }
    else {
        NSLog( @"Could not lock device for configuration: %@", error );
    }
}

- (void)setWhiteBalanceGains:(AVCaptureWhiteBalanceGains)gains
{
    NSError *error = nil;
    
    if ( [self.videoDevice lockForConfiguration:&error] ) {
        AVCaptureWhiteBalanceGains normalizedGains = [self normalizedGains:gains]; // Conversion can yield out-of-bound values, cap to limits
        [self.videoDevice setWhiteBalanceModeLockedWithDeviceWhiteBalanceGains:normalizedGains completionHandler:nil];
        [self.videoDevice unlockForConfiguration];
    }
    else {
        NSLog( @"Could not lock device for configuration: %@", error );
    }
}

- (IBAction)changeTemperature:(id)sender
{
    AVCaptureWhiteBalanceTemperatureAndTintValues temperatureAndTint = {
        .temperature = self.temperatureSlider.value,
        .tint = self.tintSlider.value,
    };
    
    [self setWhiteBalanceGains:[self.videoDevice deviceWhiteBalanceGainsForTemperatureAndTintValues:temperatureAndTint]];
}

- (IBAction)changeTint:(id)sender
{
    AVCaptureWhiteBalanceTemperatureAndTintValues temperatureAndTint = {
        .temperature = self.temperatureSlider.value,
        .tint = self.tintSlider.value,
    };
    
    [self setWhiteBalanceGains:[self.videoDevice deviceWhiteBalanceGainsForTemperatureAndTintValues:temperatureAndTint]];
}

- (IBAction)lockWithGrayWorld:(id)sender
{
    [self setWhiteBalanceGains:self.videoDevice.grayWorldDeviceWhiteBalanceGains];
}

- (AVCaptureWhiteBalanceGains)normalizedGains:(AVCaptureWhiteBalanceGains)gains
{
    AVCaptureWhiteBalanceGains g = gains;
    
    g.redGain = MAX( 1.0, g.redGain );
    g.greenGain = MAX( 1.0, g.greenGain );
    g.blueGain = MAX( 1.0, g.blueGain );
    
    g.redGain = MIN( self.videoDevice.maxWhiteBalanceGain, g.redGain );
    g.greenGain = MIN( self.videoDevice.maxWhiteBalanceGain, g.greenGain );
    g.blueGain = MIN( self.videoDevice.maxWhiteBalanceGain, g.blueGain );
    
    return g;
}

- (IBAction)toggleBagRecording:(id)sender
{
    if(!is_recording_bag){
        dispatch_async( self.sessionQueue, ^{
            NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
            NSString *full_addr = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:@"chamo.bag"];
            char *docsPath;
            docsPath = (char*)[full_addr cStringUsingEncoding:[NSString defaultCStringEncoding]];
            std::string full_file_name(docsPath);
            std::cout<<full_file_name<<std::endl;
            bag_ptr.reset(new rosbag::Bag());
            bag_ptr->open(full_file_name.c_str(), rosbag::bagmode::Write);
            is_recording_bag=true;
        });
        [sender setTitle:@"Stop" forState:UIControlStateNormal];
        
    }else{
        is_recording_bag=false;
        dispatch_async( self.sessionQueue, ^{
            bag_ptr->close();
            NSLog(@"close the bag");
        });
        [sender setTitle:@"Record" forState:UIControlStateNormal];
    }
    
}

- (IBAction)toggleMsgPublish:(id)sender
{
}


#pragma mark KVO and Notifications

- (void)addObservers
{
    [self addObserver:self forKeyPath:@"videoDevice.focusMode" options:(NSKeyValueObservingOptionOld | NSKeyValueObservingOptionNew) context:FocusModeContext];
    [self addObserver:self forKeyPath:@"videoDevice.lensPosition" options:NSKeyValueObservingOptionNew context:LensPositionContext];
    [self addObserver:self forKeyPath:@"videoDevice.exposureMode" options:(NSKeyValueObservingOptionOld | NSKeyValueObservingOptionNew) context:ExposureModeContext];
    [self addObserver:self forKeyPath:@"videoDevice.exposureDuration" options:NSKeyValueObservingOptionNew context:ExposureDurationContext];
    [self addObserver:self forKeyPath:@"videoDevice.ISO" options:NSKeyValueObservingOptionNew context:ISOContext];
    [self addObserver:self forKeyPath:@"videoDevice.exposureTargetBias" options:NSKeyValueObservingOptionNew context:ExposureTargetBiasContext];
    [self addObserver:self forKeyPath:@"videoDevice.exposureTargetOffset" options:NSKeyValueObservingOptionNew context:ExposureTargetOffsetContext];
    [self addObserver:self forKeyPath:@"videoDevice.whiteBalanceMode" options:(NSKeyValueObservingOptionOld | NSKeyValueObservingOptionNew) context:WhiteBalanceModeContext];
    [self addObserver:self forKeyPath:@"videoDevice.deviceWhiteBalanceGains" options:NSKeyValueObservingOptionNew context:DeviceWhiteBalanceGainsContext];
}

- (void)removeObservers
{
    [[NSNotificationCenter defaultCenter] removeObserver:self];
    [self removeObserver:self forKeyPath:@"videoDevice.focusMode" context:FocusModeContext];
    [self removeObserver:self forKeyPath:@"videoDevice.lensPosition" context:LensPositionContext];
    [self removeObserver:self forKeyPath:@"videoDevice.exposureMode" context:ExposureModeContext];
    [self removeObserver:self forKeyPath:@"videoDevice.exposureDuration" context:ExposureDurationContext];
    [self removeObserver:self forKeyPath:@"videoDevice.ISO" context:ISOContext];
    [self removeObserver:self forKeyPath:@"videoDevice.exposureTargetBias" context:ExposureTargetBiasContext];
    [self removeObserver:self forKeyPath:@"videoDevice.exposureTargetOffset" context:ExposureTargetOffsetContext];
    [self removeObserver:self forKeyPath:@"videoDevice.whiteBalanceMode" context:WhiteBalanceModeContext];
    [self removeObserver:self forKeyPath:@"videoDevice.deviceWhiteBalanceGains" context:DeviceWhiteBalanceGainsContext];
}

- (void)observeValueForKeyPath:(NSString *)keyPath ofObject:(id)object change:(NSDictionary *)change context:(void *)context
{
    id oldValue = change[NSKeyValueChangeOldKey];
    id newValue = change[NSKeyValueChangeNewKey];
    
    if ( context == FocusModeContext ) {
        if ( newValue && newValue != [NSNull null] ) {
            AVCaptureFocusMode newMode = (AVCaptureFocusMode)[newValue intValue];
            dispatch_async( dispatch_get_main_queue(), ^{
                self.focusModeControl.selectedSegmentIndex = [self.focusModes indexOfObject:@(newMode)];
                self.lensPositionSlider.enabled = ( newMode == AVCaptureFocusModeLocked );
                
                if ( oldValue && oldValue != [NSNull null] ) {
                    AVCaptureFocusMode oldMode = (AVCaptureFocusMode)[oldValue intValue];
                    NSLog( @"focus mode: %@ -> %@", [self stringFromFocusMode:oldMode], [self stringFromFocusMode:newMode] );
                }
                else {
                    NSLog( @"focus mode: %@", [self stringFromFocusMode:newMode] );
                }
            } );
        }
    }
    else if ( context == LensPositionContext ) {
        if ( newValue && newValue != [NSNull null] ) {
            AVCaptureFocusMode focusMode = self.videoDevice.focusMode;
            float newLensPosition = [newValue floatValue];
            dispatch_async( dispatch_get_main_queue(), ^{
                if ( focusMode != AVCaptureFocusModeLocked ) {
                    self.lensPositionSlider.value = newLensPosition;
                }
                
                self.lensPositionValueLabel.text = [NSString stringWithFormat:@"%.1f", newLensPosition];
            } );
        }
    }
    else if ( context == ExposureModeContext ) {
        if ( newValue && newValue != [NSNull null] ) {
            AVCaptureExposureMode newMode = (AVCaptureExposureMode)[newValue intValue];
            if ( oldValue && oldValue != [NSNull null] ) {
                AVCaptureExposureMode oldMode = (AVCaptureExposureMode)[oldValue intValue];
                /*
                 It’s important to understand the relationship between exposureDuration and the minimum frame rate as represented by activeVideoMaxFrameDuration.
                 In manual mode, if exposureDuration is set to a value that's greater than activeVideoMaxFrameDuration, then activeVideoMaxFrameDuration will
                 increase to match it, thus lowering the minimum frame rate. If exposureMode is then changed to automatic mode, the minimum frame rate will
                 remain lower than its default. If this is not the desired behavior, the min and max frameRates can be reset to their default values for the
                 current activeFormat by setting activeVideoMaxFrameDuration and activeVideoMinFrameDuration to kCMTimeInvalid.
                 */
                if ( oldMode != newMode && oldMode == AVCaptureExposureModeCustom ) {
                    NSError *error = nil;
                    if ( [self.videoDevice lockForConfiguration:&error] ) {
                        self.videoDevice.activeVideoMaxFrameDuration = kCMTimeInvalid;
                        self.videoDevice.activeVideoMinFrameDuration = kCMTimeInvalid;
                        [self.videoDevice unlockForConfiguration];
                    }
                    else {
                        NSLog( @"Could not lock device for configuration: %@", error );
                    }
                }
            }
            dispatch_async( dispatch_get_main_queue(), ^{
                
                self.exposureModeControl.selectedSegmentIndex = [self.exposureModes indexOfObject:@(newMode)];
                self.exposureDurationSlider.enabled = ( newMode == AVCaptureExposureModeCustom );
                self.ISOSlider.enabled = ( newMode == AVCaptureExposureModeCustom );
                
                if ( oldValue && oldValue != [NSNull null] ) {
                    AVCaptureExposureMode oldMode = (AVCaptureExposureMode)[oldValue intValue];
                    NSLog( @"exposure mode: %@ -> %@", [self stringFromExposureMode:oldMode], [self stringFromExposureMode:newMode] );
                }
                else {
                    NSLog( @"exposure mode: %@", [self stringFromExposureMode:newMode] );
                }
            } );
        }
    }
    else if ( context == ExposureDurationContext ) {
        if ( newValue && newValue != [NSNull null] ) {
            double newDurationSeconds = CMTimeGetSeconds( [newValue CMTimeValue] );
            AVCaptureExposureMode exposureMode = self.videoDevice.exposureMode;
            
            double minDurationSeconds = MAX( CMTimeGetSeconds( self.videoDevice.activeFormat.minExposureDuration ), kExposureMinimumDuration );
            double maxDurationSeconds = CMTimeGetSeconds( self.videoDevice.activeFormat.maxExposureDuration );
            // Map from duration to non-linear UI range 0-1
            double p = ( newDurationSeconds - minDurationSeconds ) / ( maxDurationSeconds - minDurationSeconds ); // Scale to 0-1
            dispatch_async( dispatch_get_main_queue(), ^{
                if ( exposureMode != AVCaptureExposureModeCustom ) {
                    self.exposureDurationSlider.value = pow( p, 1 / kExposureDurationPower ); // Apply inverse power
                }
                if ( newDurationSeconds < 1 ) {
                    int digits = MAX( 0, 2 + floor( log10( newDurationSeconds ) ) );
                    self.exposureDurationValueLabel.text = [NSString stringWithFormat:@"1/%.*f", digits, 1/newDurationSeconds];
                }
                else {
                    self.exposureDurationValueLabel.text = [NSString stringWithFormat:@"%.2f", newDurationSeconds];
                }
            } );
        }
    }
    else if ( context == ISOContext ) {
        if ( newValue && newValue != [NSNull null] ) {
            float newISO = [newValue floatValue];
            AVCaptureExposureMode exposureMode = self.videoDevice.exposureMode;
            
            dispatch_async( dispatch_get_main_queue(), ^{
                if ( exposureMode != AVCaptureExposureModeCustom ) {
                    self.ISOSlider.value = newISO;
                }
                self.ISOValueLabel.text = [NSString stringWithFormat:@"%i", (int)newISO];
            } );
        }
    }
    else if ( context == ExposureTargetBiasContext ) {
        if ( newValue && newValue != [NSNull null] ) {
            float newExposureTargetBias = [newValue floatValue];
            dispatch_async( dispatch_get_main_queue(), ^{
                self.exposureTargetBiasValueLabel.text = [NSString stringWithFormat:@"%.1f", newExposureTargetBias];
            } );
        }
    }
    else if ( context == ExposureTargetOffsetContext ) {
        if ( newValue && newValue != [NSNull null] ) {
            float newExposureTargetOffset = [newValue floatValue];
            dispatch_async( dispatch_get_main_queue(), ^{
                self.exposureTargetOffsetSlider.value = newExposureTargetOffset;
                self.exposureTargetOffsetValueLabel.text = [NSString stringWithFormat:@"%.1f", newExposureTargetOffset];
            } );
        }
    }
    else if ( context == WhiteBalanceModeContext ) {
        if ( newValue && newValue != [NSNull null] ) {
            AVCaptureWhiteBalanceMode newMode = (AVCaptureWhiteBalanceMode)[newValue intValue];
            dispatch_async( dispatch_get_main_queue(), ^{
                self.whiteBalanceModeControl.selectedSegmentIndex = [self.whiteBalanceModes indexOfObject:@(newMode)];
                self.temperatureSlider.enabled = ( newMode == AVCaptureWhiteBalanceModeLocked );
                self.tintSlider.enabled = ( newMode == AVCaptureWhiteBalanceModeLocked );
                
                if ( oldValue && oldValue != [NSNull null] ) {
                    AVCaptureWhiteBalanceMode oldMode = (AVCaptureWhiteBalanceMode)[oldValue intValue];
                    NSLog( @"white balance mode: %@ -> %@", [self stringFromWhiteBalanceMode:oldMode], [self stringFromWhiteBalanceMode:newMode] );
                }
            } );
        }
    }
    else if ( context == DeviceWhiteBalanceGainsContext ) {
        if ( newValue && newValue != [NSNull null] ) {
            AVCaptureWhiteBalanceGains newGains;
            [newValue getValue:&newGains];
            AVCaptureWhiteBalanceTemperatureAndTintValues newTemperatureAndTint = [self.videoDevice temperatureAndTintValuesForDeviceWhiteBalanceGains:newGains];
            AVCaptureWhiteBalanceMode whiteBalanceMode = self.videoDevice.whiteBalanceMode;
            dispatch_async( dispatch_get_main_queue(), ^{
                if ( whiteBalanceMode != AVCaptureExposureModeLocked ) {
                    self.temperatureSlider.value = newTemperatureAndTint.temperature;
                    self.tintSlider.value = newTemperatureAndTint.tint;
                }
                
                self.temperatureValueLabel.text = [NSString stringWithFormat:@"%i", (int)newTemperatureAndTint.temperature];
                self.tintValueLabel.text = [NSString stringWithFormat:@"%i", (int)newTemperatureAndTint.tint];
            } );
        }
    }
    else {
        [super observeValueForKeyPath:keyPath ofObject:object change:change context:context];
    }
}

- (void)subjectAreaDidChange:(NSNotification *)notification
{
    CGPoint devicePoint = CGPointMake( 0.5, 0.5 );
    [self focusWithMode:self.videoDevice.focusMode exposeWithMode:self.videoDevice.exposureMode atDevicePoint:devicePoint monitorSubjectAreaChange:NO];
}

#pragma mark Utilities

- (NSString *)stringFromFocusMode:(AVCaptureFocusMode)focusMode
{
    NSString *string = @"INVALID FOCUS MODE";
    
    if ( focusMode == AVCaptureFocusModeLocked ) {
        string = @"Locked";
    }
    else if ( focusMode == AVCaptureFocusModeAutoFocus ) {
        string = @"Auto";
    }
    else if ( focusMode == AVCaptureFocusModeContinuousAutoFocus ) {
        string = @"ContinuousAuto";
    }
    
    return string;
}

- (NSString *)stringFromExposureMode:(AVCaptureExposureMode)exposureMode
{
    NSString *string = @"INVALID EXPOSURE MODE";
    
    if ( exposureMode == AVCaptureExposureModeLocked ) {
        string = @"Locked";
    }
    else if ( exposureMode == AVCaptureExposureModeAutoExpose ) {
        string = @"Auto";
    }
    else if ( exposureMode == AVCaptureExposureModeContinuousAutoExposure ) {
        string = @"ContinuousAuto";
    }
    else if ( exposureMode == AVCaptureExposureModeCustom ) {
        string = @"Custom";
    }
    
    return string;
}

- (NSString *)stringFromWhiteBalanceMode:(AVCaptureWhiteBalanceMode)whiteBalanceMode
{
    NSString *string = @"INVALID WHITE BALANCE MODE";
    
    if ( whiteBalanceMode == AVCaptureWhiteBalanceModeLocked ) {
        string = @"Locked";
    }
    else if ( whiteBalanceMode == AVCaptureWhiteBalanceModeAutoWhiteBalance ) {
        string = @"Auto";
    }
    else if ( whiteBalanceMode == AVCaptureWhiteBalanceModeContinuousAutoWhiteBalance ) {
        string = @"ContinuousAuto";
    }
    
    return string;
}

@end
