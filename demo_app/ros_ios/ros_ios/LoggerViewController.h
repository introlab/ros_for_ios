//
//  LoggerViewController.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "ros_logger.h"
#import "LoggerPreferencesViewController.h"
#import <MessageUI/MessageUI.h>

#import "Log.h"

@interface LoggerViewController : UITableViewController <LoggerPreferencesViewControllerDelegate, MFMailComposeViewControllerDelegate>
{
@public
    RosLogger * ros_controller_;
    BOOL isPaused;
    BOOL newData;
    BOOL debugEnabled;
    BOOL infoEnabled;
    BOOL warnEnabled;
    BOOL errorEnabled;
    BOOL fatalEnabled;
    
@protected
    UIBarButtonItem * pause;
    NSTimer * timer;
}

typedef NS_ENUM(NSInteger, LogsType) {
    LEVEL = 0,
    TIME,
    MESSAGE,
    NODE,
    NB_FIELD
};

@property(nonatomic, retain) IBOutlet UIBarButtonItem * pause;
@property (nonatomic, strong) NSMutableArray * logs;

- (void) newLogReceived:(Log*)log;
- (IBAction)pauseOrResume:(id)sender;
- (IBAction)openMail:(id)sender;

@end
