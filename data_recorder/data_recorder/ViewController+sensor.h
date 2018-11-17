#import "AVCamManualCameraViewController.h"
@interface AVCamManualCameraViewController (Sensor)
- (void)processIMU_gyro;
- (UIImage *) imageFromSampleBuffer:(CMSampleBufferRef) sampleBuffer;
- (void)update_baglist;
- (void)del_baglist:(bool)del_all filename:(NSString *)filename;
- (void)upload_baglist:(bool)upload_all filename:(NSString *)filename;
@end
