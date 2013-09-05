//
//  LoggerDetailTableViewController.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-03-19.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "Log.h"

#import "ros_logger.h"

@interface LoggerDetailTableViewController : UITableViewController

@property RosLogger * ros_controller_;
@property NSInteger index;
@property NSInteger lvl;
@property (strong, nonatomic) IBOutlet UILabel *level;
@property (strong, nonatomic) IBOutlet UILabel *time;
@property (strong, nonatomic) IBOutlet UILabel *node;
@property (strong, nonatomic) IBOutlet UILabel *message;


@end
