//
//  ViewController.h
//  data_recorder
//
//  Created by user on 3/17/18.
//  Copyright © 2018 user. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <ros/ros.h>
#include "std_msgs/String.h"

@interface ViewController : UIViewController{
    ros::Publisher text_chamo_pub;
}
@property (weak, nonatomic) IBOutlet UITextField *ip_text_field;
    
- (IBAction)ip_edit_ended:(id)sender;
+ (NSString *)getIPAddress;
+ (BOOL)isValidIp:(NSString*)string;


@end

