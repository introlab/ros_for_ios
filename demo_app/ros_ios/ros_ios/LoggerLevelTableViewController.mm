//
//  LoggerLevelTableViewController.m
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-03-19.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "LoggerLevelTableViewController.h"
#import "LoggerLevelTableViewCell.h"

#import "LoggerDetailTableViewController.h"

@interface LoggerLevelTableViewController ()

@end

@implementation LoggerLevelTableViewController

@synthesize tableView;
@synthesize ros_controller_;
@synthesize lvl;

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
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (void)viewWillAppear:(BOOL)animated
{
    NSLog(@"viewWillAppear");
    [self startTimer];
}

- (void)viewWillDisappear:(BOOL)animated
{
    NSLog(@"viewWillDisappear");
    [self stopTimer];
}

-(void)dealloc
{
    NSLog(@"dealloc");
}

- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender
{
    if ([[segue identifier] isEqualToString:@"showDetails"])
    {
        LoggerDetailTableViewController * detailViewController = [segue destinationViewController];
        NSIndexPath *indexPath = [self.tableView indexPathForCell:sender];
        detailViewController.index = [indexPath row];
        detailViewController.lvl = lvl;
        detailViewController.ros_controller_ = ros_controller_;
    }
}

#pragma mark - Table view data source

- (NSInteger)numberOfSectionsInTableView:(UITableView *)tableView
{
    // Return the number of sections.
    return 1;
}

- (NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section
{
    // Return the number of rows in the section.
    return ros_controller_->getNumberOfLogs(lvl);
}

- (UITableViewCell *)tableView:(UITableView *)tableView cellForRowAtIndexPath:(NSIndexPath *)indexPath
{
    static NSString *CellIdentifier = @"loggerLevelTableCell";
    
    LoggerLevelTableViewCell *cell = [self.tableView
                                      dequeueReusableCellWithIdentifier:CellIdentifier];
    if (cell == nil) {
        cell = [[LoggerLevelTableViewCell alloc]
                initWithStyle:UITableViewCellStyleDefault
                reuseIdentifier:CellIdentifier];
    }
    
    // Configure the cell...
        
    cell.node.text = [NSString stringWithUTF8String:
                      ros_controller_->getLogName(lvl,[indexPath row])];
    
    cell.message.text = [NSString stringWithUTF8String:
                         ros_controller_->getLogMsg(lvl,[indexPath row])];
    return cell;
    
}

- (BOOL)tableView:(UITableView *)tableView canEditRowAtIndexPath:(NSIndexPath *)indexPath
{
    // Return NO if you do not want the specified item to be editable.
    return NO;
}

#pragma mark - Table view delegate

- (void)tableView:(UITableView *)tableView didSelectRowAtIndexPath:(NSIndexPath *)indexPath
{
    // Navigation logic may go here. Create and push another view controller.
    /*
     <#DetailViewController#> *detailViewController = [[<#DetailViewController#> alloc] initWithNibName:@"<#Nib name#>" bundle:nil];
     // ...
     // Pass the selected object to the new view controller.
     [self.navigationController pushViewController:detailViewController animated:YES];
     */
}

/*- (void)tableView:(UITableView *)tableView accessoryButtonTappedForRowWithIndexPath:(NSIndexPath *)indexPath
 {
 [self performSegueWithIdentifier: @"showDetails" sender: [self.tableView cellForRowAtIndexPath: indexPath]];
 }*/

#pragma mark – Refresh the UI

- (void)timerCB
{
    if(ros_controller_->newLogReceived())
    {
        [self.tableView reloadData];
        ros_controller_->newLogReceived(false);
    }
}

- (IBAction)startTimer
{
    timer = [NSTimer scheduledTimerWithTimeInterval:0.1 target:self selector:@selector(timerCB) userInfo:nil repeats:YES];
}

- (IBAction)stopTimer
{
    [timer invalidate];
}

@end
