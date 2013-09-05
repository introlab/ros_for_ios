//
//  LoggerDetailTableViewController.m
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-03-19.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "LoggerDetailTableViewController.h"

@interface LoggerDetailTableViewController ()

@end

@implementation LoggerDetailTableViewController

@synthesize ros_controller_;
@synthesize index;
@synthesize lvl;
@synthesize level;
@synthesize time;
@synthesize node;
@synthesize message;


- (id)initWithStyle:(UITableViewStyle)style
{
    self = [super initWithStyle:style];
    if (self) {
        // Custom initialization
    }
    return self;
}

- (void)viewDidLoad
{
    [super viewDidLoad];

    NSLog(@"LoggerDetailTableViewController : viewDidLoad");
    
    if(lvl == DEBUG)
        level.text = @"[DEBUG]";
    else if(lvl == INFO)
        level.text = @"[INFO]";
    else if(lvl == WARN)
        level.text = @"[WARN]";
    else if(lvl == ERROR)
        level.text = @"[ERROR]";
    else if(lvl == FATAL)
        level.text = @"[FATAL]";
    
    time.text = [NSString stringWithFormat:@"%d",
                 ros_controller_->getLogStamp(lvl, index)];
    node.text = [NSString stringWithUTF8String:
                 ros_controller_->getLogName(lvl,index)];
    message.text = [NSString stringWithUTF8String:
                    ros_controller_->getLogMsg(lvl,index)];
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)viewWillAppear:(BOOL)animated
{
    NSLog(@"LoggerDetailTableViewController : viewWillAppear");
}

- (void)viewWillDisappear:(BOOL)animated
{
    NSLog(@"LoggerDetailTableViewController : viewWillDisappear");
}

-(void)dealloc
{
    NSLog(@"LoggerDetailTableViewController : dealloc");
}

@end
