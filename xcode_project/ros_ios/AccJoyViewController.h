//
//  AccJoyViewController.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "ros_joy.h"

@class AccelerometerFilter;

@interface AccJoyViewController : UIViewController <UIAccelerometerDelegate>
{
@public
    RosJoy * ros_controller_;
    
@protected
    UIAccelerometer * accelerometer;
    UIBarButtonItem *pause;
    AccelerometerFilter *filter;
    BOOL isPaused;
    CGPoint center;
}

@property (readonly) BOOL isPaused;
@property (nonatomic, retain) UIAccelerometer * accelerometer;
@property(nonatomic, retain) IBOutlet UIBarButtonItem *pause;
@property (strong, nonatomic) IBOutlet UIImageView *image;

- (IBAction)pauseOrResume:(id)sender;

@end
