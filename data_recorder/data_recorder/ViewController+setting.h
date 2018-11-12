#import "AVCamManualCameraViewController.h"
static const float kExposureDurationPower = 5; // Higher numbers will give the slider more sensitivity at shorter durations
static const float kExposureMinimumDuration = 1.0/1000; // Limit exposure duration to a useful range
@interface AVCamManualCameraViewController (Setting)

- (void) loadConfig;
- (void) configureManualHUD;
- (void) configureSession;
- (NSString *)stringFromFocusMode:(AVCaptureFocusMode)focusMode;
- (NSString *)stringFromExposureMode:(AVCaptureExposureMode)exposureMode;
- (void)addObservers;
- (void)removeObservers;
- (void)focusWithMode:(AVCaptureFocusMode)focusMode exposeWithMode:(AVCaptureExposureMode)exposureMode atDevicePoint:(CGPoint)point monitorSubjectAreaChange:(BOOL)monitorSubjectAreaChange;
- (void)subjectAreaDidChange:(NSNotification *)notification;
- (void)changeCameraWithDevice:(AVCaptureDevice *)newVideoDevice;

@end
