#import "AVCamManualCameraViewController.h"
#import "ViewController+setting.h"
#import "ViewController+sensor.h"
#import <ifaddrs.h>
#import <arpa/inet.h>
#include "common_header.h"
#include <opencv2/opencv.hpp>
#include <iomanip>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

CMMotionManager *motionManager;
@implementation AVCamManualCameraViewController

- (void)viewDidLoad
{
	[super viewDidLoad];
    need_record=true;
    is_recording_bag=false;
    is_publishing=false;
    [self loadConfig];
    
	// Disable UI until the session starts running
	self.cameraButton.enabled = YES;
	self.recordButton.enabled = YES;
	self.pubButton.enabled = NO;
	self.manualHUDIPView.hidden = YES;
	self.manualHUDFocusView.hidden = YES;
	self.manualHUDExposureView.hidden = YES;
    self.settingPanel.hidden=YES;
	
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
        motionManager.accelerometerUpdateInterval =0.01;
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
        motionManager.gyroUpdateInterval =0.01;
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

- (IBAction)changeManualHUD:(id)sender
{
    self.manualHUDFocusView.hidden = YES;
    self.manualHUDExposureView.hidden = YES;
    self.manualHUDIPView.hidden = YES;
    self.topicView.hidden= YES;
    UISegmentedControl *control = sender;
    if(control.selectedSegmentIndex == 0){
        self.manualHUDIPView.hidden = NO;
        self.topicView.hidden= NO;
    }else if(control.selectedSegmentIndex == 1){
        self.manualHUDFocusView.hidden = NO;
        self.manualHUDExposureView.hidden = NO;
    }
}

- (IBAction)sliderTouchBegan:(id)sender
{
}

- (IBAction)sliderTouchEnded:(id)sender
{
    UISlider *slider = (UISlider *)sender;
    if ( slider == self.lensPositionSlider ) {
        
    }
    else if ( slider == self.exposureDurationSlider ) {
        
    }
    else if ( slider == self.ISOSlider ) {
        
    }
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
        if(is_publishing){
            img_pub.publish(img_ros_img);
        }
        dispatch_async( self.sessionQueue, ^{
            if(is_recording_bag){
                if(bag_ptr->isOpen()){
                    bag_ptr->write("cam0", img_ros_img.header.stamp, img_ros_img);
                }
            }
        });
        
        img_count++;
    }
}

- (IBAction)toggleSetting:(id)sender {
    self.settingPanel.hidden = !self.settingPanel.hidden;
}
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
    
    self.cameraButton.enabled = NO;
    self.recordButton.enabled = NO;
    self.pubButton.enabled = NO;
    
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
    
    [defaults setObject:@"custom" forKey:@"focus_mode"];
    [defaults synchronize];
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

@end
