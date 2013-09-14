//
//  AccJoyViewController.mm
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "AccJoyViewController.h"
#import "AccelerometerFilter.h"

#define kUpdateFrequency    60.0

@interface AccJoyViewController ()

@end

@implementation AccJoyViewController

@synthesize isPaused, accelerometer, pause;

- (void)viewDidLoad
{
    [super viewDidLoad];
    [self.view setMultipleTouchEnabled:NO];
    // Do any additional setup after loading the view, typically from a nib.
    NSLog(@"AccJoyViewController : viewDidLoad");
    
    ros_controller_ = new RosJoy();
    
    isPaused = NO;
    
    [self changeFilter:[LowpassFilter class]];
    [[UIAccelerometer sharedAccelerometer] setUpdateInterval:1.0 / kUpdateFrequency];
    [[UIAccelerometer sharedAccelerometer] setDelegate:self];
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
    NSLog(@"didReceiveMemoryWarning");
}

- (void)viewWillAppear:(BOOL)animated
{
    NSLog(@"AccJoyViewController : viewWillAppear");
    center.x = self.view.frame.size.width/2;
    center.y = self.view.frame.size.height/2-22;
}

- (void)viewWillDisappear:(BOOL)animated
{
    NSLog(@"AccJoyViewController : viewWillDisappear");
    [[UIAccelerometer sharedAccelerometer] setDelegate:nil];
    delete ros_controller_;
}

-(void)dealloc
{
    NSLog(@"AccJoyViewController : dealloc");
}

- (void)accelerometer:(UIAccelerometer *)accelerometer didAccelerate:(UIAcceleration *)acceleration
{
    self.image.image = nil;
    
    if(!isPaused)
    {
        double k_x = 0.5;
        double k_y = -0.5;
        double k_theta = -0.5;
        
        [filter addAcceleration:acceleration];
        
        ros_controller_->sendCmds(k_x*filter.y, k_y*0.0, k_theta*filter.x);
        
        UIGraphicsBeginImageContext(self.view.frame.size);
        [self.image.image drawInRect:self.view.frame];
        CGContextMoveToPoint(UIGraphicsGetCurrentContext(), center.x, center.y);
        CGContextAddLineToPoint(UIGraphicsGetCurrentContext(), center.x+50*filter.x, center.y);
        CGContextSetLineCap(UIGraphicsGetCurrentContext(), kCGLineCapRound);
        CGContextSetLineWidth(UIGraphicsGetCurrentContext(), 10.0);
        CGContextSetRGBStrokeColor(UIGraphicsGetCurrentContext(), 1, 0, 0, 1.0);
        CGContextStrokePath(UIGraphicsGetCurrentContext());
        
        CGContextMoveToPoint(UIGraphicsGetCurrentContext(), center.x, center.y);
        CGContextAddLineToPoint(UIGraphicsGetCurrentContext(), center.x, center.y-50*filter.y);
        CGContextSetRGBStrokeColor(UIGraphicsGetCurrentContext(), 0, 0, 1, 1.0);
        CGContextStrokePath(UIGraphicsGetCurrentContext());
        
        self.image.image = UIGraphicsGetImageFromCurrentImageContext();
        UIGraphicsEndImageContext();
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

- (void)changeFilter:(Class)filterClass
{
    // Ensure that the new filter class is different from the current one...
    if(filterClass != [filter class])
    {
        filter = [[filterClass alloc] initWithSampleRate:kUpdateFrequency cutoffFrequency:5.0];
        // Set the adaptive flag
        filter.adaptive = YES;
    }
}

@end
