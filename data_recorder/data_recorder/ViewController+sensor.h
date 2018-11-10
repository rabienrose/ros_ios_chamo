#import "AVCamManualCameraViewController.h"
@interface AVCamManualCameraViewController (Sensor)
- (void)processIMU_gyro;
- (UIImage *) imageFromSampleBuffer:(CMSampleBufferRef) sampleBuffer;
@end
