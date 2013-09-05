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
    GLuint framebuffer_;
    GLuint colorRenderBuffer_;
    GLuint depthRenderBuffer_;
    GLuint vao1_;
    GLuint vao2_;
    GLuint vertexBufferMap_;
    GLuint indexBufferMap_;
    GLuint vertexBufferRobot_;
    GLuint indexBufferRobot_;
    GLuint programHandleMap_;
    GLuint programHandleRobot_;
    GLuint positionMapSlot_;
    GLuint sourceMapSlot_;
    GLuint positionRobotSlot_;
    GLuint sourceRobotSlot_;
    GLuint projectionMaptUniform_;
    GLuint modelViewMapUniform_;
    GLuint projectionRobotUniform_;
    GLuint modelViewRobotUniform_;
    
    GLKVector3 camPos_, center_;
    
    GLKMatrix4 projectionMatrix_;
    GLKMatrix4 modelMatrix_;
    GLKMatrix4 viewMatrix_;
}

@property (nonatomic, strong) EAGLContext *glContext;
@property (nonatomic, strong) CAEAGLLayer *glLayer;
@property (strong, nonatomic) IBOutlet UISegmentedControl *viewTypeSelector;

- (IBAction)buttonGoPressed:(id)sender;
- (IBAction)viewTypeChanged:(id)sender;
- (IBAction)tapDetected:(UITapGestureRecognizer *)sender;
- (IBAction)pinchDetected:(UIPinchGestureRecognizer *)sender;
- (IBAction)panDetected:(UIPanGestureRecognizer *)sender;

@end
