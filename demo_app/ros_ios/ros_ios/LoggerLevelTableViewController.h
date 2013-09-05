//
//  LoggerLevelTableViewController.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-03-19.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "Log.h"
#import "ros_logger.h"

@interface LoggerLevelTableViewController : UITableViewController
{
    NSTimer * timer;
}

@property RosLogger * ros_controller_;
@property NSInteger lvl;

@end