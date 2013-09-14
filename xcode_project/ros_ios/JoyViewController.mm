//
//  JoyViewController.mm
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "JoyViewController.h"
#import <QuartzCore/QuartzCore.h>
#import <AudioToolbox/AudioToolbox.h>

#import <vector>
#import <math.h>

@interface JoyViewController ()

@end

@implementation JoyViewController

@synthesize ballView;

- (void)viewDidLoad
{
    [super viewDidLoad];
    [self.view setMultipleTouchEnabled:NO];
	// Do any additional setup after loading the view, typically from a nib.
    NSLog(@"AccJoyViewController : viewDidLoad");
    
    ros_controller_ = new RosJoy();
    
    ballView = [[UIImageView alloc] initWithFrame:CGRectMake(0, 0, 50, 50)];
    [ballView setBackgroundColor:[UIColor redColor]];
    ballView.layer.cornerRadius = 25;
    ballView.userInteractionEnabled = YES;
    [self.view addSubview:ballView];
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
    ballView.center = center;
    
    [self startTimer];
}

- (void)viewWillDisappear:(BOOL)animated
{
    NSLog(@"AccJoyViewController : viewWillDisappear");
    [self stopTimer];
    delete ros_controller_;
}

-(void)dealloc
{
    NSLog(@"AccJoyViewController : dealloc");
}

- (void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event
{
    UITouch *touch = [touches anyObject];
    
    if (touch.view == self.ballView)
    {
        [ballView setBackgroundColor:[UIColor blueColor]];
        [self vibrate];
        ballPushed = YES;
    }
}

- (void)touchesMoved:(NSSet *)touches withEvent:(UIEvent *)event
{
    UITouch *touch = [touches anyObject];
    CGPoint touch_pt = [touch locationInView:self.view];
    
    if(touch.view == self.ballView)
    {
        currentPoint = touch_pt;
        
        self.image.image = nil;
        
        UIGraphicsBeginImageContext(self.view.frame.size);
        [self.image.image drawInRect:self.view.frame];
        CGContextMoveToPoint(UIGraphicsGetCurrentContext(), center.x, center.y);
        CGContextAddLineToPoint(UIGraphicsGetCurrentContext(), currentPoint.x, currentPoint.y);
        CGContextSetLineCap(UIGraphicsGetCurrentContext(), kCGLineCapRound);
        CGContextSetLineWidth(UIGraphicsGetCurrentContext(), 5.0);
        CGContextSetRGBStrokeColor(UIGraphicsGetCurrentContext(), 1, 1, 1, 1.0);
        CGContextSetBlendMode(UIGraphicsGetCurrentContext(),kCGBlendModeNormal);
        
        CGContextStrokePath(UIGraphicsGetCurrentContext());
        self.image.image = UIGraphicsGetImageFromCurrentImageContext();
        UIGraphicsEndImageContext();
        
        ballView.center = currentPoint;
    }
}

- (void)touchesEnded:(NSSet *)touches withEvent:(UIEvent *)event
{
    
    UITouch *touch = [touches anyObject];
    
    if (touch.view == self.ballView)
    {
        self.image.image = nil;
        [ballView setBackgroundColor:[UIColor redColor]];
        [self vibrate];
        ballPushed = NO;
        
        ballView.center = center;
    }
}

- (void)timerCB
{
    [self generateCmds];
}

- (IBAction)startTimer
{
    timer = [NSTimer scheduledTimerWithTimeInterval:0.1 target:self selector:@selector(timerCB) userInfo:nil repeats:YES];
}

- (IBAction)stopTimer
{
    [timer invalidate];
}

- (void)generateCmds
{
    double k_x = 2.5;
    double k_y = -2.5;
    double k_theta = -2.5;
    
    if (ballPushed)
    {
        CGPoint cmdPoint = currentPoint;
        cmdPoint.x -= self.view.frame.size.width/2;
        cmdPoint.x /= self.view.frame.size.width/2;
        cmdPoint.y = self.view.frame.size.height/2 - cmdPoint.y;
        cmdPoint.y /= self.view.frame.size.height/2;
        ros_controller_->sendCmds(k_x*cmdPoint.y, k_y*0.0, k_theta*cmdPoint.x);
    }
}

- (void)vibrate
{
    AudioServicesPlaySystemSound(kSystemSoundID_Vibrate);
}

@end
