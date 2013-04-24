//
//  MapViewController.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <GLKit/GLKit.h>
#import "ros_planner.h"

@interface MapViewController : GLKViewController
{
    RosPlanner * ros_controller_;
}

- (IBAction)buttonGoPressed:(id)sender;

@end
