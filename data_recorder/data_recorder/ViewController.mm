//
//  ViewController.m
//  data_recorder
//
//  Created by user on 3/17/18.
//  Copyright © 2018 user. All rights reserved.
//

#import "ViewController.h"
#import <ifaddrs.h>
#import <arpa/inet.h>

@interface ViewController ()
@end

@implementation ViewController
@synthesize defaults, ip_text_field;
    
- (IBAction)start_record:(id)sender{
    NSLog(@"press me!!");
    std_msgs::String s_msg;
    s_msg.data="hello world!";
    text_chamo_pub.publish(s_msg);
}
- (IBAction)ip_edit_ended:(id)sender{
    [defaults setObject:[ip_text_field.attributedText string] forKey:@"master_uri"];
    [defaults synchronize];
    [self connectToMaster];
}
    
- (void)viewDidLoad {
    [super viewDidLoad];
    defaults = [NSUserDefaults standardUserDefaults];
    [ip_text_field setText:[defaults objectForKey:@"master_uri"]];
    [self connectToMaster];
}


- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}
    
+ (NSString *)getIPAddress
{
    struct ifaddrs *interfaces = NULL;
    struct ifaddrs *temp_addr = NULL;
    NSString *wifiAddress = nil;
    NSString *cellAddress = nil;
    
    // retrieve the current interfaces - returns 0 on success
    if(!getifaddrs(&interfaces)) {
        // Loop through linked list of interfaces
        temp_addr = interfaces;
        while(temp_addr != NULL) {
            sa_family_t sa_type = temp_addr->ifa_addr->sa_family;
            if(sa_type == AF_INET || sa_type == AF_INET6) {
                NSString *name = [NSString stringWithUTF8String:temp_addr->ifa_name];
                NSString *addr = [NSString stringWithUTF8String:inet_ntoa(((struct sockaddr_in *)temp_addr->ifa_addr)->sin_addr)]; // pdp_ip0
                NSLog(@"NAME: \"%@\" addr: %@", name, addr); // see for yourself
                
                if([name isEqualToString:@"en0"]) {
                    // Interface is the wifi connection on the iPhone
                    wifiAddress = addr;
                } else
                if([name isEqualToString:@"pdp_ip0"]) {
                    // Interface is the cell connection on the iPhone
                    //cellAddress = addr;
                }
            }
            temp_addr = temp_addr->ifa_next;
        }
        // Free memory
        freeifaddrs(interfaces);
    }
    NSString *addr = wifiAddress ? wifiAddress : cellAddress;
    return addr ? addr : @"0.0.0.0";
}
    
- (BOOL)connectToMaster
{
    if([ViewController isValidIp:[ip_text_field.attributedText string]])
    {
        NSString * master_uri = [@"ROS_MASTER_URI=http://" stringByAppendingString:[[ip_text_field.attributedText string] stringByAppendingString:@":11311/"]];
        NSLog(@"%@",master_uri);
        NSString * ip = [@"ROS_IP=" stringByAppendingString:[ViewController getIPAddress]];
        NSLog(@"%@",ip);
        putenv((char *)[master_uri UTF8String]);
        putenv((char *)[ip UTF8String]);
        putenv((char *)"ROS_HOME=/tmp");
        
        int argc = 0;
        char ** argv = NULL;
        if(!ros::isInitialized())
        {
            ros::init(argc,argv,"data_recorder");
            if(ros::master::check())
            {
                NSLog(@"Connected to the ROS master !");
                ros::NodeHandle nn;
                text_chamo_pub = nn.advertise<std_msgs::String>("Gps", 1, true);
            }
            else
            {
                NSLog(@"fail to connect to the ROS master !");
            }
        }
        else
        {
            NSLog(@"ROS already initialised. Can'st change the ROS_MASTER_URI");
        }
    }else{
        NSLog(@"input a right ip !");
        return false;
    }
    return true;
}
    
+ (BOOL)isValidIp:(NSString*)string
{
    struct in_addr pin;
    int success = inet_pton(AF_INET,[string UTF8String],&pin);
    if(success == 1) return YES;
    return NO;
}


@end
