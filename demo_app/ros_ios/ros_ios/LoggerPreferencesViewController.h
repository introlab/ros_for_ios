//
//  LoggerPreferencesViewController.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-03-18.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>

@class LoggerPreferencesViewController;
@protocol LoggerPreferencesViewControllerDelegate <NSObject>

-(void) switchDebugDidChanged: (LoggerPreferencesViewController *)controller data:(BOOL) enable;
-(void) switchInfoDidChanged: (LoggerPreferencesViewController *)controller data:(BOOL) enable;
-(void) switchWarnDidChanged: (LoggerPreferencesViewController *)controller data:(BOOL) enable;
-(void) switchErrorDidChanged: (LoggerPreferencesViewController *)controller data:(BOOL) enable;
-(void) switchFatalDidChanged: (LoggerPreferencesViewController *)controller data:(BOOL) enable;
-(void) buttonSendMailPushed: (LoggerPreferencesViewController *)controller;
-(void) buttonClearLogsPushed: (LoggerPreferencesViewController *)controller;
-(void) buttonSavePushed: (LoggerPreferencesViewController *)controller;

@end

@interface LoggerPreferencesViewController : UITableViewController
@property (strong, nonatomic) IBOutlet UISwitch *switchDebug;
@property (strong, nonatomic) IBOutlet UISwitch *switchInfo;
@property (strong, nonatomic) IBOutlet UISwitch *switchWarn;
@property (strong, nonatomic) IBOutlet UISwitch *switchError;
@property (strong, nonatomic) IBOutlet UISwitch *switchFatal;
@property (strong, nonatomic) IBOutlet UILabel *messageNumber;
@property(weak,nonatomic) id<LoggerPreferencesViewControllerDelegate> delegate;

- (IBAction)switchDebugChanged:(id)sender;
- (IBAction)switchInfoChanged:(id)sender;
- (IBAction)switchWarnChanged:(id)sender;
- (IBAction)switchErrorChanged:(id)sender;
- (IBAction)switchFatalChanged:(id)sender;

- (IBAction)sendMail:(id)sender;
- (IBAction)clearLogs:(id)sender;
- (IBAction)save:(id)sender;

@end
