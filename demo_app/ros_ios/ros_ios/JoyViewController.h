//
//  JoyViewController.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "ros_joy.h"

@interface JoyViewController : UIViewController
{
    RosJoy * ros_controller_;
    NSTimer * timer;
    BOOL ballPushed;
    CGPoint currentPoint;
    CGPoint center;
}

@property (strong, nonatomic) IBOutlet UIImageView *image;
@property (strong, nonatomic) IBOutlet UIView *ballView;

- (void)timerCB;

- (IBAction)startTimer;

- (IBAction)stopTimer;

- (void)generateCmds;

- (void)vibrate;

@end
