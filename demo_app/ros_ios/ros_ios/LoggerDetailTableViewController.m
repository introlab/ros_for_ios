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
    
    // Uncomment the following line to preserve selection between presentations.
    // self.clearsSelectionOnViewWillAppear = NO;
    
    // Uncomment the following line to display an Edit button in the navigation bar for this view controller.
    // self.navigationItem.rightBarButtonItem = self.editButtonItem;
    
    if(self.log.level == DEBUG)
        level.text = @"[DEBUG]";
    else if(self.log.level == INFO)
        level.text = @"[INFO]";
    else if(self.log.level == WARN)
        level.text = @"[WARN]";
    else if(self.log.level == ERROR)
        level.text = @"[ERROR]";
    else if(self.log.level == FATAL)
        level.text = @"[FATAL]";
    
    time.text = self.log.stamp;
    node.text = self.log.node;
    message.text = self.log.message;
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)viewWillAppear:(BOOL)animated
{
    NSLog(@"viewWillAppear");
}

- (void)viewWillDisappear:(BOOL)animated
{
    NSLog(@"viewWillDisappear");
}

-(void)dealloc
{
    NSLog(@"dealloc");
}

@end
