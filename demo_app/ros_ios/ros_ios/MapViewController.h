//
//  MapViewController.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <GLKit/GLKit.h>
#import <OpenGLES/ES2/gl.h>
#import <OpenGLES/ES2/glext.h>
#import <QuartzCore/QuartzCore.h>
#import "ros_planner.h"

@interface MapViewController : GLKViewController
{
    RosPlanner * ros_controller_;
    EAGLContext * glContext_;
    CAEAGLLayer * glLayer_;
    GLuint colorRenderBuffer_;
    GLuint positionSlot_;
    GLuint projectionUniform_;
    GLuint modelViewUniform_;
    
    GLfloat camPos_[3];
    
    GLKMatrix4 projectionMatrix_;
    GLKMatrix4 modelMatrix_;
    GLKMatrix4 viewMatrix_;
    
    GLuint texture_;
    GLuint texCoordSlot_;
    GLuint textureUniform_;
}

@property (nonatomic, strong) EAGLContext *glContext;
@property (nonatomic, strong) CAEAGLLayer *glLayer;

- (IBAction)buttonGoPressed:(id)sender;
- (IBAction)tapDetected:(UITapGestureRecognizer *)sender;
- (IBAction)pinchDetected:(UIPinchGestureRecognizer *)sender;

@end
