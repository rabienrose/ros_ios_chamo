#import "ViewController+setting.h"

static void * FocusModeContext = &FocusModeContext;
static void * ExposureModeContext = &ExposureModeContext;
static void * LensPositionContext = &LensPositionContext;
static void * ExposureDurationContext = &ExposureDurationContext;
static void * ISOContext = &ISOContext;

@implementation AVCamManualCameraViewController (Setting)

- (void) loadConfig{

    defaults = [NSUserDefaults standardUserDefaults];
    NSString* imu_topic_ns = [defaults objectForKey:@"imu_topic"];
    NSString* img_topic_ns = [defaults objectForKey:@"img_topic"];
    NSString* gps_topic_ns = [defaults objectForKey:@"gps_topic"];
    NSString* img_hz_ns = [defaults objectForKey:@"img_hz"];
    
    if (imu_topic_ns==nil){
        imu_topic_ns=@"imu";
    }
    if (img_topic_ns==nil){
        img_topic_ns=@"img";
    }
    if (gps_topic_ns==nil){
        gps_topic_ns=@"gps";
    }
    if (img_hz_ns==nil){
        img_hz_ns=@"10";
    }
    [self.imu_topic_edit setText:imu_topic_ns];
    [self.img_topic_edit setText:img_topic_ns];
    [self.gps_topic_edit setText:gps_topic_ns];
    [self.img_hz_edit setText:img_hz_ns];
    
    NSString* img_size_ns = [defaults objectForKey:@"img_size"];
    if (img_size_ns==nil){
        img_size_ns=@"640x480";
    }
    [self.cam_size_btn setTitle:img_size_ns forState:UIControlStateNormal];
    
    NSString* focus_mode = [defaults objectForKey:@"focus_mode"];
    if (focus_mode==nil){
        focus_mode_std="auto";
    }else{
        focus_mode_std = std::string([focus_mode UTF8String]);
    }
    
    NSString* exposure_mode = [defaults objectForKey:@"exposure_mode"];
    if (exposure_mode==nil){
        exposure_mode_std="auto";
    }else{
        exposure_mode_std = std::string([exposure_mode UTF8String]);
    }
    
    NSString* focus_posi_ns = [defaults objectForKey:@"focus_posi"];
    if (focus_posi_ns==nil){
        cache_focus_posi=-1;
    }else{
        cache_focus_posi = atof(std::string([focus_posi_ns UTF8String]).c_str());
    }
    
    NSString* exposure_ns = [defaults objectForKey:@"exposure_t"];
    if (exposure_ns==nil){
        cache_exposure_t=-1;
    }else{
        cache_exposure_t = atof(std::string([exposure_ns UTF8String]).c_str());
    }
    
    NSString* iso_ns = [defaults objectForKey:@"iso"];
    if (iso_ns==nil){
        cache_iso=-1;
    }else{
        cache_iso = atof(std::string([iso_ns UTF8String]).c_str());
    }

}

- (void)configureManualHUD
{
    // Manual focus controls
    self.focusModes = @[@(AVCaptureFocusModeContinuousAutoFocus), @(AVCaptureFocusModeLocked)];
    
    self.focusModeControl.enabled = ( self.videoDevice != nil );
    self.focusModeControl.selectedSegmentIndex = [self.focusModes indexOfObject:@(self.videoDevice.focusMode)];
    for ( NSNumber *mode in self.focusModes ) {
        [self.focusModeControl setEnabled:[self.videoDevice isFocusModeSupported:(AVCaptureFocusMode)mode.intValue] forSegmentAtIndex:[self.focusModes indexOfObject:mode]];
    }
    [self.img_switch setOn:YES animated:NO];
    [self.imu_switch setOn:YES animated:NO];
    [self.gps_switch setOn:YES animated:NO];
    
    
    self.lensPositionSlider.minimumValue = 0.0;
    self.lensPositionSlider.maximumValue = 1.0;
    if (cache_focus_posi<0){
        self.lensPositionSlider.value = self.videoDevice.lensPosition;
    }else{
        self.lensPositionSlider.value=cache_focus_posi;
    }
    std::cout<<"lensPosition: "<<self.lensPositionSlider.value<<std::endl;
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
    double exposureDurationSeconds;
    if(cache_exposure_t<0){
        exposureDurationSeconds = CMTimeGetSeconds( self.videoDevice.exposureDuration );
    }else{
        exposureDurationSeconds = cache_exposure_t;
    }
    
    double minExposureDurationSeconds = MAX( CMTimeGetSeconds( self.videoDevice.activeFormat.minExposureDuration ), kExposureMinimumDuration );
    double maxExposureDurationSeconds = CMTimeGetSeconds( self.videoDevice.activeFormat.maxExposureDuration );
    // Map from duration to non-linear UI range 0-1
    double p = ( exposureDurationSeconds - minExposureDurationSeconds ) / ( maxExposureDurationSeconds - minExposureDurationSeconds ); // Scale to 0-1
    self.exposureDurationSlider.value = pow( p, 1 / kExposureDurationPower ); // Apply inverse power
    self.exposureDurationSlider.enabled = ( self.videoDevice && self.videoDevice.exposureMode == AVCaptureExposureModeCustom );
    std::cout<<"exposure time: "<<exposureDurationSeconds<<std::endl;
    self.ISOSlider.minimumValue = self.videoDevice.activeFormat.minISO;
    self.ISOSlider.maximumValue = self.videoDevice.activeFormat.maxISO;
    if(cache_iso<0){
        self.ISOSlider.value = self.videoDevice.ISO;
    }else{
        self.ISOSlider.value=cache_iso;
    }
    
    std::cout<<"iso: "<<self.ISOSlider.value<<std::endl;
    self.ISOSlider.enabled = ( self.videoDevice.exposureMode == AVCaptureExposureModeCustom );
    
}

// Should be called on the session queue
- (void)configureSession
{
    if ( self.setupResult != AVCamManualSetupResultSuccess ) {
        return;
    }
    NSError *error = nil;
    [self.session beginConfiguration];
    NSString * btn_string=[self.cam_size_btn titleForState:UIControlStateNormal];
    bool find_his_size=false;
    for (int i=0; i<[self.camSizesName count]; i++){
        if([self.camSizesName[i] isEqualToString:btn_string]){
            self.session.sessionPreset = self.camSizes[i];
            find_his_size=true;
            break;
        }
    }
    if(find_his_size==false){
        self.session.sessionPreset =AVCaptureSessionPreset640x480;
        [self.cam_size_btn setTitle:@"640x480" forState:UIControlStateNormal];
    }
    
    self.videoDevice = [AVCaptureDevice defaultDeviceWithDeviceType:AVCaptureDeviceTypeBuiltInWideAngleCamera mediaType:AVMediaTypeVideo position:AVCaptureDevicePositionUnspecified];
    AVCaptureDeviceInput *videoDeviceInput = [AVCaptureDeviceInput deviceInputWithDevice:self.videoDevice error:&error];
    if ( ! videoDeviceInput ) {
        NSLog( @"Could not create video device input: %@", error );
        self.setupResult = AVCamManualSetupResultSessionConfigurationFailed;
        [self.session commitConfiguration];
        return;
    }
    if ( [self.session canAddInput:videoDeviceInput] ) {
        [self.session addInput:videoDeviceInput];
        self.videoDeviceInput = videoDeviceInput;
        self.videoDevice = self.videoDevice;
        
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

    if ( [self.videoDevice lockForConfiguration:&error] ) {
        AVCaptureFocusMode mode_focus;
        if (focus_mode_std=="custom"){
            mode_focus=AVCaptureFocusModeLocked;
        }else if(focus_mode_std=="auto"){
            mode_focus=AVCaptureFocusModeContinuousAutoFocus;
        }else{
            NSLog( @"load focus mode failed!");
            self.setupResult = AVCamManualSetupResultSessionConfigurationFailed;
            [self.session commitConfiguration];
            return;
        }
        if ( [self.videoDevice isFocusModeSupported:mode_focus] ) {
            self.videoDevice.focusMode = mode_focus;
        }
        
        AVCaptureExposureMode mode_exposure;
        if (exposure_mode_std=="custom"){
            mode_exposure=AVCaptureExposureModeCustom;
        }else if(exposure_mode_std=="auto"){
            mode_exposure=AVCaptureExposureModeContinuousAutoExposure;
        }else if(exposure_mode_std=="lock"){
            mode_exposure=AVCaptureExposureModeLocked;
        }else{
            NSLog( @"load exposure mode failed!");
            self.setupResult = AVCamManualSetupResultSessionConfigurationFailed;
            [self.session commitConfiguration];
            return;
        }
        if([self.videoDevice isExposureModeSupported:mode_exposure]){
            self.videoDevice.exposureMode=mode_exposure;
        }
        if(focus_mode_std=="custom"){
            if(cache_focus_posi>0){
                [self.videoDevice setFocusModeLockedWithLensPosition:cache_focus_posi completionHandler:nil];
            }
        }
        if(exposure_mode_std=="custom"){
            if(cache_exposure_t>0){
                [self.videoDevice setExposureModeCustomWithDuration:CMTimeMakeWithSeconds( cache_exposure_t, 1000*1000*1000 ) ISO:AVCaptureISOCurrent completionHandler:nil];
            }
            if(cache_iso>0){
                [self.videoDevice setExposureModeCustomWithDuration:AVCaptureExposureDurationCurrent ISO:cache_iso completionHandler:nil];
            }
            
        }
        
        
        [self.videoDevice unlockForConfiguration];
    }else {
        NSLog( @"Could not lock device for configuration: %@", error );
    }

    self.video_output = [[AVCaptureVideoDataOutput alloc] init];
    NSDictionary *newSettings = @{ (NSString *)kCVPixelBufferPixelFormatTypeKey : @(kCVPixelFormatType_32BGRA) };
    self.video_output.videoSettings = newSettings;
    [self.video_output setAlwaysDiscardsLateVideoFrames:YES];
    if ([self.session canAddOutput:self.video_output]) {
        [self.session addOutput:self.video_output];
    }else {
        NSLog(@"add output wrong!!!");
    }
    
    [self.video_output setSampleBufferDelegate:self queue:self.sessionQueue];
    NSString *temp_string =  [self.img_hz_edit.attributedText string];
    int x = [temp_string intValue];
    if(x<=30 && x>=1){
        [self.videoDevice setActiveVideoMinFrameDuration:CMTimeMake(1, x)];
    }else{
        [self.videoDevice setActiveVideoMinFrameDuration:CMTimeMake(1, 10)];
    }
    
    [self.session commitConfiguration];
    
    dispatch_async( dispatch_get_main_queue(), ^{
        [self configureManualHUD];
    } );
}

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

- (void)addObservers
{
    [self addObserver:self forKeyPath:@"videoDevice.focusMode" options:(NSKeyValueObservingOptionOld | NSKeyValueObservingOptionNew) context:FocusModeContext];
    [self addObserver:self forKeyPath:@"videoDevice.lensPosition" options:NSKeyValueObservingOptionNew context:LensPositionContext];
    [self addObserver:self forKeyPath:@"videoDevice.exposureMode" options:(NSKeyValueObservingOptionOld | NSKeyValueObservingOptionNew) context:ExposureModeContext];
    [self addObserver:self forKeyPath:@"videoDevice.exposureDuration" options:NSKeyValueObservingOptionNew context:ExposureDurationContext];
    [self addObserver:self forKeyPath:@"videoDevice.ISO" options:NSKeyValueObservingOptionNew context:ISOContext];
}

- (void)removeObservers
{
    [[NSNotificationCenter defaultCenter] removeObserver:self];
    [self removeObserver:self forKeyPath:@"videoDevice.focusMode" context:FocusModeContext];
    [self removeObserver:self forKeyPath:@"videoDevice.lensPosition" context:LensPositionContext];
    [self removeObserver:self forKeyPath:@"videoDevice.exposureMode" context:ExposureModeContext];
    [self removeObserver:self forKeyPath:@"videoDevice.exposureDuration" context:ExposureDurationContext];
    [self removeObserver:self forKeyPath:@"videoDevice.ISO" context:ISOContext];
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
                self.lensPositionSlider.value = newLensPosition;
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
                self.exposureDurationSlider.value = pow( p, 1 / kExposureDurationPower ); // Apply inverse power
                if ( newDurationSeconds < 1 ) {
                    int digits = MAX( 0, 2 + floor( log10( newDurationSeconds ) ) );
                    self.exposureDurationValueLabel.text = [NSString stringWithFormat:@"1/%.*f", digits, 1/newDurationSeconds];
                    //std::cout<<"new expo time: "<<newDurationSeconds<<std::endl;
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
                self.ISOSlider.value = newISO;
                self.ISOValueLabel.text = [NSString stringWithFormat:@"%i", (int)newISO];
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


- (void)focusWithMode:(AVCaptureFocusMode)focusMode exposeWithMode:(AVCaptureExposureMode)exposureMode atDevicePoint:(CGPoint)point monitorSubjectAreaChange:(BOOL)monitorSubjectAreaChange
{
    dispatch_async( self.sessionQueue, ^{
        AVCaptureDevice *device = self.videoDevice;
        
        NSError *error = nil;
        if ( [device lockForConfiguration:&error] ) {
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



@end
