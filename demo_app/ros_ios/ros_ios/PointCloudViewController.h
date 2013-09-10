//
//  ViewController.h
//  opengl_kinect
//
//  Created by Ronan Chauvin on 2013-06-04.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <GLKit/GLKit.h>
#import <OpenGLES/ES2/gl.h>
#import <OpenGLES/ES2/glext.h>
#import <QuartzCore/QuartzCore.h>
#include "ros_kinect.h"

@interface PointCloudViewController : GLKViewController
{
    EAGLContext * glContext_;
    CAEAGLLayer * glLayer_;
    GLuint frameBuffer_;
    GLuint colorRenderBuffer_;
    GLuint depthRenderBuffer_;
    GLuint vertexBuffer_;
    GLuint indexBuffer_;
    GLuint positionSlot_;
    GLuint projectionRWUniform_;
    GLuint projectionUniform_;
    GLuint modelViewUniform_;
    GLuint texture_;
    
    GLfloat camPos_[3];
    
    GLKMatrix4 projectionMatrix_;
    GLKMatrix4 modelMatrix_;
    GLKMatrix4 viewMatrix_;
    
    GLuint texCoordSlot_;
    GLuint textureUniform_;
    RosKinect * ros_controller_;
}

@property (nonatomic, strong) EAGLContext *glContext;
@property (nonatomic, strong) CAEAGLLayer *glLayer;

- (IBAction)tapDetected:(UITapGestureRecognizer *)sender;
- (IBAction)pinchDetected:(UIPinchGestureRecognizer *)sender;

@end
