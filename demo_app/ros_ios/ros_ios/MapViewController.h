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
#import "MapImage.h"

@interface MapViewController : GLKViewController
{
    RosPlanner * ros_controller_;
    EAGLContext * _glContext;
    CAEAGLLayer * _glLayer;
    GLuint _colorRenderBuffer;
    GLuint _positionSlot;
    GLuint _colorSlot;
    GLuint _projectionUniform;
    GLuint _modelViewUniform;
    
    //float _currentRotation;
    GLKMatrix4 _transMatrix;
    GLKMatrix4 _rotMatrix;
    GLKVector3 _anchor_position;
    GLKVector3 _current_position;
    GLKQuaternion _quatStart;
    GLKQuaternion _quat;
    
    GLuint _floorTexture;
    GLuint _texCoordSlot;
    GLuint _textureUniform;
    
    MapImage * _texture;
}

@property (nonatomic, strong) EAGLContext *glContext;
@property (nonatomic, strong) CAEAGLLayer *glLayer;

@property (nonatomic, strong) MapImage *texture;

- (IBAction)buttonGoPressed:(id)sender;
- (IBAction)tapDetected:(UITapGestureRecognizer *)sender;
- (IBAction)pinchDetected:(UIPinchGestureRecognizer *)sender;

@end
