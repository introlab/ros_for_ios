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
    BOOL debugEnabled;
    BOOL infoEnabled;
    BOOL warnEnabled;
    BOOL errorEnabled;
    BOOL fatalEnabled;
    
@protected
    NSTimer * timer;
}

typedef NS_ENUM(NSInteger, LogsType) {
    LEVEL = 0,
    TIME,
    MESSAGE,
    NODE,
    NB_FIELD
};

- (IBAction)openMail:(id)sender;

@end
