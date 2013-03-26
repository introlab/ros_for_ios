//
//  LoggerViewController.mm
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "LoggerViewController.h"
#import "LoggerTableViewCell.h"
#import "LoggerLevelTableViewController.h"

@interface LoggerViewController ()

@end

@implementation LoggerViewController

@synthesize pause;
@synthesize logs;

- (void)viewDidLoad
{
    [super viewDidLoad];
    [self.view setMultipleTouchEnabled:NO];
	// Do any additional setup after loading the view, typically from a nib.

    isPaused = NO;
    newData = NO;
    
    LoggerPreferencesViewController *svc = [self.tabBarController.viewControllers objectAtIndex:1];
    svc.delegate = self;
    
    debugEnabled = YES;
    infoEnabled = YES;
    warnEnabled = YES;
    errorEnabled = YES;
    fatalEnabled = YES;
    
    logs  = [[NSMutableArray alloc] initWithCapacity:NB_LEVEL];
    for(LogLevel i = 0; i < NB_LEVEL; i++)
        [logs addObject:[[NSMutableArray alloc] init]];
    
    ros_controller_ = new RosLogger();
}

- (void) newLogReceived:(Log*)log
{
    if((log.level == DEBUG && debugEnabled)
        || (log.level == INFO && infoEnabled)
            || (log.level == WARN && warnEnabled)
                || (log.level == ERROR && errorEnabled)
                    || (log.level == FATAL && fatalEnabled))
    {
        [logs[log.level] addObject:log];
        newData = YES;
    }
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
    NSLog(@"didReceiveMemoryWarning");
}

- (void)viewWillAppear:(BOOL)animated
{
    NSLog(@"viewWillAppear");
    [self startTimer];
    ros_controller_->view_controller_ = self;
}

- (void)viewWillDisappear:(BOOL)animated
{
    NSLog(@"viewWillDisappear");
    [self stopTimer];
    ros_controller_->view_controller_ = nil;
}

-(void)dealloc
{
    NSLog(@"dealloc");
    delete ros_controller_;
    ros_controller_ = nil;
}

- (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender
{
    if ([[segue identifier] isEqualToString:@"showDetails"])
    {
        LoggerLevelTableViewController * detailViewController = [segue destinationViewController];
        NSIndexPath *indexPath = [self.tableView indexPathForCell:sender];
        detailViewController.levelLogs = [self.logs objectAtIndex:[indexPath row]];
    }
    else
    {
        ros_controller_->view_controller_ = nil;
    }
}

- (IBAction)pauseOrResume:(id)sender
{
    if(isPaused)
	{
		isPaused = NO;
		pause.title = @"Pause";
	}
	else
	{
		isPaused = YES;
		pause.title = @"Resume";
	}
}

#pragma mark – Table view data source

- (NSInteger)numberOfSectionsInTableView:(UITableView *)tableView
{
    // Return the number of sections.
    return 1;
}

- (NSInteger)tableView:(UITableView *)tableView numberOfRowsInSection:(NSInteger)section
{
    // Return the number of rows in the section.
        NSLog(@"%d",[self.logs[1] count]);
    return [self.logs count];
}

- (UITableViewCell *)tableView:(UITableView *)tableView cellForRowAtIndexPath:(NSIndexPath *)indexPath
{
    static NSString *CellIdentifier = @"loggerTableCell";
    
    LoggerTableViewCell *cell = [tableView
                              dequeueReusableCellWithIdentifier:CellIdentifier];
    if (cell == nil) {
        cell = [[LoggerTableViewCell alloc]
                initWithStyle:UITableViewCellStyleDefault
                reuseIdentifier:CellIdentifier];
    }
    
    // Configure the cell...
    NSString * level;
    
    if([indexPath row] == DEBUG)
        level = @"[DEBUG]";
    else if([indexPath row] == INFO)
        level = @"[INFO]";
    else if([indexPath row] == WARN)
        level = @"[WARN]";
    else if([indexPath row] == ERROR)
        level = @"[ERROR]";
    else if([indexPath row] == FATAL)
        level = @"[FATAL]";
    
    cell.level.text = level;
    
    cell.number.text = [NSString stringWithFormat:@"%d", [self.logs[[indexPath row]] count]];
    return cell;
}

#pragma mark – LoggerPreferencesViewController Delegate Protocol

- (void) switchDebugDidChanged: (LoggerPreferencesViewController *)controller data:(BOOL) enable
{
    debugEnabled = enable;
}

- (void) switchInfoDidChanged: (LoggerPreferencesViewController *)controller data:(BOOL) enable
{
    infoEnabled = enable;
}

- (void) switchWarnDidChanged: (LoggerPreferencesViewController *)controller data:(BOOL) enable
{
    warnEnabled = enable;
}

- (void) switchErrorDidChanged: (LoggerPreferencesViewController *)controller data:(BOOL) enable
{
    errorEnabled = enable;
}

- (void) switchFatalDidChanged: (LoggerPreferencesViewController *)controller data:(BOOL) enable
{
    fatalEnabled = enable;
}

- (void) buttonSendMailPushed:(LoggerPreferencesViewController *)controller
{
    [self openMail:controller];
}

- (void) buttonClearLogsPushed:(LoggerPreferencesViewController *)controller
{
    [self.logs removeAllObjects];
    [self.tableView reloadData];
}

- (void) buttonSavePushed:(LoggerPreferencesViewController *)controller
{

}

- (IBAction)openMail:(id)sender
{
    if ([MFMailComposeViewController canSendMail])
    {
        MFMailComposeViewController *mailer = [[MFMailComposeViewController alloc] init];
        mailer.mailComposeDelegate = self;
        [mailer setSubject:@"ROS"];
        NSString * emailBody = [[NSString alloc] init];
        
        for(int j = 0; j < logs.count; j++)
        {
            NSMutableArray * levelLogs = logs[j];
            
            if(j == DEBUG)
                emailBody = [emailBody stringByAppendingString:@"[DEBUG]"];
            else if(j == INFO)
                emailBody = [emailBody stringByAppendingString:@"[INFO]"];
            else if(j == WARN)
                emailBody = [emailBody stringByAppendingString:@"[WARN]"];
            else if(j == ERROR)
                emailBody = [emailBody stringByAppendingString:@"[ERROR]"];
            else if(j == FATAL)
                emailBody = [emailBody stringByAppendingString:@"[FATAL]"];
            
            emailBody = [emailBody stringByAppendingString:@"\r\n"];
            
            for(int i = 0; i < levelLogs.count; i++)
            {
                Log * log = levelLogs[i];
            
                emailBody = [emailBody stringByAppendingString:@" "];
                emailBody = [emailBody stringByAppendingString:log.stamp];
                emailBody = [emailBody stringByAppendingString:@" "];
                emailBody = [emailBody stringByAppendingString:log.node];
                emailBody = [emailBody stringByAppendingString:@" "];
                emailBody = [emailBody stringByAppendingString:log.message];
                emailBody = [emailBody stringByAppendingString:@"\r\n"];
            }
        }
        
        [mailer setMessageBody:emailBody isHTML:NO];
        [self presentViewController:mailer animated:YES completion:nil];
    }
    else
    {
        UIAlertView *alert = [[UIAlertView alloc] initWithTitle:@"Failure" message:@"Your device doesn't support the composer sheet" delegate:nil cancelButtonTitle:@"OK" otherButtonTitles:nil];
        [alert show];
    }
}

- (void)mailComposeController:(MFMailComposeViewController*)controller didFinishWithResult:(MFMailComposeResult)result error:(NSError*)error
{
    switch (result)
    {
        case MFMailComposeResultCancelled:
            NSLog(@"Mail cancelled: you cancelled the operation and no email message was queued.");
            break;
        case MFMailComposeResultSaved:
            NSLog(@"Mail saved: you saved the email message in the drafts folder.");
            break;
        case MFMailComposeResultSent:
            NSLog(@"Mail send: the email message is queued in the outbox. It is ready to send.");
            break;
        case MFMailComposeResultFailed:
            NSLog(@"Mail failed: the email message was not saved or queued, possibly due to an error.");
            break;
        default:
            NSLog(@"Mail not sent.");
            break;
    }
    // Remove the mail view
    [self dismissViewControllerAnimated:YES completion:nil];
}

#pragma mark – Refresh the UI

- (void)timerCB
{
    if(newData && !isPaused)
    {
        [self.tableView reloadData];
        newData = NO;
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
