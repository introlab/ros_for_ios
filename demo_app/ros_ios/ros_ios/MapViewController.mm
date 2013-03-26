//
//  MapViewController.mm
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "MapViewController.h"
#import <QuartzCore/QuartzCore.h>

#import <vector>
#import <math.h>

@interface MapViewController ()

@end

@implementation MapViewController

@synthesize mapView;
@synthesize goalView;
@synthesize label;

- (void)viewDidLoad
{
    [super viewDidLoad];
    [self.view setMultipleTouchEnabled:NO];
	// Do any additional setup after loading the view, typically from a nib.
    
    //CGRect robot_frame = CGRectMake(50, 150, 10, 10);
    //_robotView = [[UIImageView alloc] initWithFrame:robot_frame];
    //[_robotView setBackgroundColor:[UIColor redColor]];
    //_robotView.layer.cornerRadius = 5;
    //[self.view addSubview:_robotView];
    
    CGRect goal_frame = CGRectMake(self.view.frame.size.width/2, self.view.frame.size.height/2-22, 10, 10);

    goalView = [[UIImageView alloc] initWithFrame:goal_frame];
    [goalView setBackgroundColor:[UIColor blueColor]];
    goalView.layer.cornerRadius = 5;
    goalView.userInteractionEnabled = YES;
    [self.view addSubview:goalView];
    
    ros_controller_ = new RosPlanner();
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
    ros_controller_->view_controller_ = self;
}

- (void)viewWillDisappear:(BOOL)animated
{
    NSLog(@"viewWillDisappear");
    ros_controller_->view_controller_ = nil;
}

-(void)dealloc
{
    NSLog(@"dealloc");
    delete ros_controller_;
}

-(void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event
{    
    UITouch *touch = [touches anyObject];
    CGPoint touchPoint = [touch locationInView:self.view];

    if(CGRectContainsPoint(mapView.frame, touchPoint))
    {
        goalView.center = touchPoint;
    }
}

- (void)touchesMoved:(NSSet *)touches withEvent:(UIEvent *)event
{
    UITouch *touch = [touches anyObject];
    CGPoint touchPoint = [touch locationInView:self.view];
    
    if(/*touch.view == self.goalView && */CGRectContainsPoint(mapView.frame, touchPoint))
    {
        goalView.center = touchPoint;
        
        touchPoint = [self.view convertPoint:touchPoint toView:mapView];
        NSString * pos = [NSString stringWithFormat:@"X:%d Y:%d", (int)round(touchPoint.x), (int)round(touchPoint.y)];
        label.text = pos;
    }
}

- (void)touchesEnded:(NSSet *)touches withEvent:(UIEvent *)event
{

}

- (IBAction)buttonGetPlanPressed:(id)sender
{
    /*CGPoint goal = [self convertPointFromUIViewToImage:goalView.center];
    
    if(ros_controller_->checkGoal(goal))
    {
        std::vector<CGPoint> plan = ros_controller_->getPlan(goal);
        
        for (size_t i = 0; i < plan.size()-1; i++)
        {
            CGPoint pose_1 = [self convertPointFromImageToUIView:plan[i]];
            CGPoint pose_2 = [self convertPointFromImageToUIView:plan[i+1]];
            
            _drawImage.image = nil;
            
            UIGraphicsBeginImageContext(self.view.frame.size);
            [self.drawImage.image drawInRect:self.drawImage.frame];
            CGContextMoveToPoint(UIGraphicsGetCurrentContext(), pose_1.x, pose_1.y);
            CGContextAddLineToPoint(UIGraphicsGetCurrentContext(), pose_2.x, pose_2.y);
            CGContextSetLineCap(UIGraphicsGetCurrentContext(), kCGLineCapRound);
            CGContextSetLineWidth(UIGraphicsGetCurrentContext(), 2.0);
            CGContextSetRGBStrokeColor(UIGraphicsGetCurrentContext(), 0, 0, 0, 1.0);
            CGContextSetBlendMode(UIGraphicsGetCurrentContext(),kCGBlendModeNormal);
            
            CGContextStrokePath(UIGraphicsGetCurrentContext());
            self.drawImage.image = UIGraphicsGetImageFromCurrentImageContext();
            UIGraphicsEndImageContext();
        }
    }
    else
    {
        UIAlertView * alert = [[UIAlertView alloc] initWithTitle:@"Wrong Goal" message:@"Move the goal to a valid position" delegate:self cancelButtonTitle:@"Ok" otherButtonTitles:nil];
        [alert show];
    }*/
}

- (IBAction)buttonGoPressed:(id)sender
{
    CGPoint goal = [self convertPointFromUIViewToImage:goalView.center];
    
    NSLog(@"%f %f", goal.x, goal.y);
    
    if(ros_controller_->checkGoal(goal))
    {
        ros_controller_->sendGoal(goal);
    }
    else
    {
        UIAlertView * alert = [[UIAlertView alloc] initWithTitle:@"Wrong Goal" message:@"Move the goal to a valid position" delegate:self cancelButtonTitle:@"Ok" otherButtonTitles:nil];
        [alert show];
    }
}

- (CGPoint)convertPointFromUIViewToImage:(CGPoint)in_point
{
    CGPoint out_point = {0.0,0.0};

    if (mapView.image != nil)
    {
        
        CGFloat x_ratio = in_point.x / mapView.frame.size.width;
        CGFloat y_ratio = in_point.y / mapView.frame.size.height;
    
        CGImageRef cgImage = [mapView.image CGImage];
    
        out_point.x = x_ratio * CGImageGetWidth(cgImage);
        out_point.y = y_ratio * CGImageGetHeight(cgImage);
    }
    
    return out_point;
}

- (CGPoint)convertPointFromImageToUIView:(CGPoint)in_point
{
    CGPoint out_point = {0.0,0.0};
    
    if (mapView.image != nil)
    {
        CGImageRef cgImage = [mapView.image CGImage];
        
        CGFloat x_ratio = in_point.x / CGImageGetWidth(cgImage);
        CGFloat y_ratio = in_point.y / CGImageGetHeight(cgImage);
        
        out_point.x = x_ratio * mapView.frame.size.width;
        out_point.y = y_ratio * mapView.frame.size.height;

    }    
    return out_point;
}

@end
