//
//  MapViewController.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "ros_planner.h"

@interface MapViewController : UIViewController
{
    RosPlanner * ros_controller_;
}

@property (weak, nonatomic) IBOutlet UIImageView *mapView;
@property (weak, nonatomic) IBOutlet UIImageView *drawImage;
@property (strong, nonatomic) IBOutlet UIView *goalView;
//@property (strong, nonatomic) IBOutlet UIView *robotView;
@property (strong, nonatomic) IBOutlet UILabel *label;

- (IBAction)buttonGetPlanPressed:(id)sender;
- (IBAction)buttonGoPressed:(id)sender;

- (CGPoint)convertPointFromUIViewToImage:(CGPoint)in_point;

- (CGPoint)convertPointFromImageToUIView:(CGPoint)in_point;

@end
