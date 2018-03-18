//
//  ViewController.m
//  data_recorder
//
//  Created by user on 3/17/18.
//  Copyright © 2018 user. All rights reserved.
//

#import "ViewController.h"

@interface ViewController ()
@end

@implementation ViewController
- (IBAction)ip_edit_ended:(id)sender
    {
        NSLog(@"press me!!");
        std_msgs::String s_msg;
        s_msg.data="hello world!";
        text_chamo_pub.publish(s_msg);
    }
    
- (void)viewDidLoad {
    [super viewDidLoad];
    
    NSString * master_uri = @"ROS_MASTER_URI=http://192.168.31.156:11311/";
    NSLog(@"%@",master_uri);
    
    NSString * ip = @"ROS_IP=192.168.31.24";
    NSLog(@"%@",ip);
    
    putenv((char *)[master_uri UTF8String]);
    putenv((char *)[ip UTF8String]);
    putenv((char *)"ROS_HOME=/tmp");
    
    int argc = 0;
    char ** argv = NULL;
    if(!ros::isInitialized())
    {
        ros::init(argc,argv,"data_recorder");
    }
    else
    {
        NSLog(@"ROS already initialised. Can't change the ROS_MASTER_URI");
    }
    
    if(ros::master::check())
    {
        NSLog(@"Connected to the ROS master !");
        ros::NodeHandle nn;
        text_chamo_pub = nn.advertise<std_msgs::String>("Gps", 1, true);
    }
    else
    {
        NSLog(@"fail to connet to the ROS master !");
    }
}


- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}


@end
