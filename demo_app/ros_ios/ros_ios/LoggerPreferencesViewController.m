//
//  LoggerPreferencesViewController.m
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-03-18.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "LoggerPreferencesViewController.h"

@interface LoggerPreferencesViewController ()

@end

@implementation LoggerPreferencesViewController

- (id)initWithNibName:(NSString *)nibNameOrNil bundle:(NSBundle *)nibBundleOrNil
{
    self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
    if (self) {
        // Custom initialization
    }
    return self;
}

- (void)viewDidLoad
{
    [super viewDidLoad];
	// Do any additional setup after loading the view.
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

- (IBAction)switchDebugChanged:(id)sender
{
    BOOL value = _switchDebug.on;
    [self.delegate switchDebugDidChanged:self data:value];
}

- (IBAction)switchInfoChanged:(id)sender
{
    BOOL value = _switchInfo.on;
    [self.delegate switchInfoDidChanged:self data:value];
}

- (IBAction)switchWarnChanged:(id)sender
{
    BOOL value = _switchWarn.on;
    [self.delegate switchWarnDidChanged:self data:value];
}

- (IBAction)switchErrorChanged:(id)sender
{
    BOOL value = _switchError.on;
    [self.delegate switchErrorDidChanged:self data:value];
}

- (IBAction)switchFatalChanged:(id)sender
{
    BOOL value = _switchFatal.on;
    [self.delegate switchFatalDidChanged:self data:value];
}

- (IBAction)sendMail:(id)sender
{
    [self.delegate buttonSendMailPushed:self];
}

- (IBAction)clearLogs:(id)sender
{
    [self.delegate buttonClearLogsPushed:self];
}

- (IBAction)save:(id)sender
{
    [self.delegate buttonSavePushed:self];
}
@end
