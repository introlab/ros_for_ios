//
//  LoggerDetailTableViewController.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-03-19.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "Log.h"

@interface LoggerDetailTableViewController : UITableViewController

@property (nonatomic, strong) Log * log;
@property (strong, nonatomic) IBOutlet UILabel *level;
@property (strong, nonatomic) IBOutlet UILabel *time;
@property (strong, nonatomic) IBOutlet UILabel *node;
@property (strong, nonatomic) IBOutlet UILabel *message;


@end
