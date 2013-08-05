
//
//  PointCloudViewController.mm
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "PointCloudViewController.h"

#define RESAMPLING_FACTOR 2

typedef struct _vertexStruct
{
    GLfloat position[3];
    GLfloat texPosition[2];
} vertexStruct;

enum {
    ATTRIB_POSITION,
    ATTRIB_TEXT_POSITION,
    NUM_ATTRIBUTES
};

@interface PointCloudViewController () {
    GLfloat width;
    GLfloat height;
    GLfloat width_d;
    GLfloat height_d;
    
    vertexStruct * vertices;
    GLuint * indices;
    
    BOOL initialized;
    
    float rotX;
    float rotY;
    float zoom;
}

@end

@implementation PointCloudViewController

@synthesize glContext;
@synthesize glLayer;

- (id)initWithNibName:(NSString *)nibNameOrNil bundle:(NSBundle *)nibBundleOrNil
{
    self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
    if (self) {
        // Custom initialization
    }
    return self;
}

- (void)didReceiveMemoryWarning
{
    // Releases the view if it doesn't have a superview.
    [super didReceiveMemoryWarning];
    // Release any cached data, images, etc that aren't in use.
    
}

- (void)viewDidLoad
{
    [super viewDidLoad];
	// Do any additional setup after loading the view, typically from a nib.
    
    ros_controller_ = new RosKinect();
    
    [self setupGL];
    
    //GLKView *view = (GLKView *)self.view;
    //view.context = self.glContext;
}

- (void)dealloc
{
    NSLog(@"dealloc");
    
    if ([EAGLContext currentContext] == self.glContext) {
        [EAGLContext setCurrentContext:nil];
    }
    self.glContext = nil;
    
    free(indices);
    free(vertices);
    
    delete ros_controller_;
}

- (void)setupLayer
{
    self.glLayer = (CAEAGLLayer*) self.view.layer;
    self.glLayer.opaque = YES;
}

- (void)setupContext
{
    EAGLRenderingAPI api = kEAGLRenderingAPIOpenGLES2;
    self.glContext = [[EAGLContext alloc] initWithAPI:api];
    if (!self.glContext) {
        NSLog(@"Failed to initialize OpenGLES 2.0 context");
        exit(1);
    }
    
    if (![EAGLContext setCurrentContext:self.glContext]) {
        NSLog(@"Failed to set current OpenGL context");
        exit(1);
    }
}

- (void)setupRenderBuffer
{
    glGenRenderbuffers(1, &colorRenderBuffer_);
    glBindRenderbuffer(GL_RENDERBUFFER, colorRenderBuffer_);
    [self.glContext renderbufferStorage:GL_RENDERBUFFER fromDrawable:self.glLayer];
}

- (void)setupFrameBuffer
{
    GLuint framebuffer;
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                              GL_RENDERBUFFER, colorRenderBuffer_);
    
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER) ;
    if(status != GL_FRAMEBUFFER_COMPLETE) {
        NSLog(@"failed to make complete framebuffer object %x", status);
    }
}

- (GLuint)compileShader:(NSString*)shaderName withType:(GLenum)shaderType
{
    NSString* shaderPath = [[NSBundle mainBundle] pathForResource:shaderName
                                                           ofType:@"glsl"];
    NSError* error;
    NSString* shaderString = [NSString stringWithContentsOfFile:shaderPath
                                                       encoding:NSUTF8StringEncoding error:&error];
    if (!shaderString) {
        NSLog(@"Error loading shader: %@", error.localizedDescription);
        exit(1);
    }
    
    GLuint shaderHandle = glCreateShader(shaderType);
    
    const char * shaderStringUTF8 = [shaderString UTF8String];
    int shaderStringLength = [shaderString length];
    glShaderSource(shaderHandle, 1, &shaderStringUTF8, &shaderStringLength);
    
    glCompileShader(shaderHandle);
    
    GLint compileSuccess;
    glGetShaderiv(shaderHandle, GL_COMPILE_STATUS, &compileSuccess);
    if (compileSuccess == GL_FALSE) {
        GLchar messages[256];
        glGetShaderInfoLog(shaderHandle, sizeof(messages), 0, &messages[0]);
        NSString *messageString = [NSString stringWithUTF8String:messages];
        NSLog(@"%@", messageString);
        exit(1);
    }
    
    return shaderHandle;
}

- (void)compileShaders
{
    GLuint vertexShader = [self compileShader:@"SimpleVertex"
                                     withType:GL_VERTEX_SHADER];
    GLuint fragmentShader = [self compileShader:@"SimpleFragment"
                                       withType:GL_FRAGMENT_SHADER];
    
    GLuint programHandle = glCreateProgram();
    glAttachShader(programHandle, vertexShader);
    glAttachShader(programHandle, fragmentShader);
    glLinkProgram(programHandle);
    
    GLint linkSuccess;
    glGetProgramiv(programHandle, GL_LINK_STATUS, &linkSuccess);
    if (linkSuccess == GL_FALSE) {
        GLchar messages[256];
        glGetProgramInfoLog(programHandle, sizeof(messages), 0, &messages[0]);
        NSString *messageString = [NSString stringWithUTF8String:messages];
        NSLog(@"%@", messageString);
        exit(1);
    }
    
    glUseProgram(programHandle);
    
    positionSlot_ = glGetAttribLocation(programHandle, "Position");
    glEnableVertexAttribArray(positionSlot_);
    
    projectionRWUniform_ = glGetUniformLocation(programHandle, "ProjectionRealWorld");
    projectionUniform_ = glGetUniformLocation(programHandle, "Projection");
    modelViewUniform_ = glGetUniformLocation(programHandle, "Modelview");
    
    texCoordSlot_ = glGetAttribLocation(programHandle, "TexCoordIn");
    glEnableVertexAttribArray(texCoordSlot_);
    textureUniform_ = glGetUniformLocation(programHandle, "Texture");
}

- (void)setupTexture
{
    glGenTextures(1, &texture_);
    glBindTexture(GL_TEXTURE_2D, texture_);

    // use linear filetring
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    // clamp to edge
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

- (void)setupVBOs
{
    GLuint vertexBuffer;
    
    glGenBuffers(1, &vertexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
    
    GLuint indexBuffer;
    
    glGenBuffers(1, &indexBuffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
}

- (void)updatePointCloud
{
    if(!initialized)
    {
        ros_controller_->mtxRGBLock();
        width_d = (GLfloat) (ros_controller_->getWdth());
        height_d = (GLfloat) (ros_controller_->getHeight());
        ros_controller_->mtxRGBUnlock();
        
        width = width_d/RESAMPLING_FACTOR;
        height = height_d/RESAMPLING_FACTOR;
        
        [self setupGeometry];
        
        initialized = YES;
    }
    
    ros_controller_->mtxDepthLock();
    GLfloat * depth_data = (GLfloat *) ros_controller_->getDepth();
    
    if(indices==nil)
        indices = (GLuint *) malloc(sizeof(GLuint[6])*width*height);
    if(vertices==nil)
        vertices = (vertexStruct *) malloc(sizeof(vertexStruct[6])*width*height);
    
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            int index = i*width+j;
            int quad_index = 4 * index;
            
            const GLuint quadIndices[] =
            {
                quad_index, quad_index, quad_index+1,
                quad_index+2, quad_index+3, quad_index+3
            };
            
            GLfloat depth = depth_data[i*RESAMPLING_FACTOR*(int)width_d+j*RESAMPLING_FACTOR];
            
            if(depth != depth)
                depth = 0.0;
            
            const vertexStruct quad[] =
            {
                {{-0.5-width/2.0+j, -0.5-height/2.0+i, depth}, {j/width,i/height}},
                {{0.5-width/2.0+j, -0.5-height/2.0+i, depth}, {(j+1)/width,i/height}},
                {{-0.5-width/2.0+j, 0.5-height/2.0+i, depth}, {j/width,(i+1)/height}},
                {{0.5-width/2.0+j, 0.5-height/2.0+i, depth}, {(j+1)/width,(i+1)/height}}
            };
            
            memcpy(&indices[6*index], quadIndices, sizeof(quadIndices));
            memcpy(&vertices[quad_index], quad, sizeof(quad));
        }
    }
    
    [self updateVBOs];
    ros_controller_->mtxDepthUnlock();
    
    ros_controller_->mtxRGBLock();
    GLubyte * rgb_data = (GLubyte *) ros_controller_->getRGB();
    
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_d, height_d, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb_data);
    ros_controller_->mtxRGBUnlock();
}

- (void)updateVBOs
{
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertexStruct[4])*width*height, vertices, GL_STATIC_DRAW);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint[6])*width*height, indices, GL_STATIC_DRAW);
}

- (void)generateCamPos
{
    camPos_[0] = zoom * cosf(rotY)*cosf(rotX);
    camPos_[1] = zoom * sinf(rotY);
    camPos_[2] = zoom * cosf(rotY)*sinf(rotX);
}

- (void)setupGeometry
{
    rotX = M_PI/2;
    rotY = 0.0;
    zoom = -1.0;
    [self generateCamPos];
    
    projectionMatrix_ = GLKMatrix4MakePerspective(M_PI/4, 1.0, 0, 6);
}

- (void)setupGL
{
    indices = nil;
    vertices = nil;
    initialized = NO;
    
    [self setupLayer];
    [self setupContext];
    [self setupRenderBuffer];
    [self setupFrameBuffer];
    [self compileShaders];
    [self setupTexture];
    [self setupVBOs];
}

- (void)render
{
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    float scale = 1.0f;
    if ([[UIScreen mainScreen] respondsToSelector:@selector(displayLinkWithTarget:selector:)]) {
        scale = [[UIScreen mainScreen] scale];
    }
    
    float * P = ros_controller_->get_P();
    
    GLKMatrix4 kinectProjection = GLKMatrix4MakeWithArray(P);
    
    glUniformMatrix4fv(projectionRWUniform_, 1, 0, kinectProjection.m);
    
    glUniformMatrix4fv(projectionUniform_, 1, 0, projectionMatrix_.m);
    
    viewMatrix_ = GLKMatrix4MakeLookAt(camPos_[0], camPos_[1], camPos_[2], 0.0, 0.0, 0.0, 0.0, -1.0, 0.0);
    
    glUniformMatrix4fv(modelViewUniform_, 1, 0, viewMatrix_.m);
    
    glViewport(0, 0, self.view.bounds.size.width * scale, self.view.bounds.size.height * scale);
    
    glVertexAttribPointer(positionSlot_, 3, GL_FLOAT, GL_FALSE,
                          sizeof(vertexStruct), (void*)offsetof(vertexStruct,position));
    glVertexAttribPointer(texCoordSlot_, 2, GL_FLOAT, GL_FALSE,
                          sizeof(vertexStruct), (void*)offsetof(vertexStruct,texPosition));
    
    glDrawElements(GL_TRIANGLE_STRIP, 6*width*height, GL_UNSIGNED_INT, (void*)0);
    
    [self.glContext presentRenderbuffer:GL_RENDERBUFFER];
}

- (void)glkView:(GLKView *)view drawInRect:(CGRect)rect
{
    
}

- (void)update
{
    if(ros_controller_->newRGBDataAvailable() && ros_controller_->newDepthDataAvailable())
    {
        [self updatePointCloud];
    }
    
    if(initialized)
    {
        [self render];
    }
}

- (void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event
{
    
}

- (void)touchesMoved:(NSSet *)touches withEvent:(UIEvent *)event
{
    UITouch * touch = [touches anyObject];
    CGPoint location = [touch locationInView:self.view];
    CGPoint lastLoc = [touch previousLocationInView:self.view];
    CGPoint diff = CGPointMake(lastLoc.x - location.x, lastLoc.y - location.y);
    
    rotX += -1 * GLKMathDegreesToRadians(diff.x / 2.0);
    rotY += -1 * GLKMathDegreesToRadians(diff.y / 2.0);
    
    if(rotX > 2*M_PI)
        rotX = -2*M_PI;
    else if (rotX < -2*M_PI)
        rotX = 2*M_PI;
    
    if(rotY > 2*M_PI)
        rotY = -2*M_PI;
    else if (rotY < -2*M_PI)
        rotY = 2*M_PI;
    
    [self generateCamPos];
}

- (IBAction)tapDetected:(UITapGestureRecognizer *)sender
{
    NSLog(@"Double tap!");
    [self setupGeometry];
}

- (IBAction)pinchDetected:(UIPinchGestureRecognizer *)sender
{
    static CGFloat lastScale;
    
    if([(UIPinchGestureRecognizer*)sender state] == UIGestureRecognizerStateEnded)
    {
        lastScale = 1.0;
        return;
	}
    
    CGFloat scale = 1.0 - (lastScale - [(UIPinchGestureRecognizer*)sender scale]);
    CGFloat velocity = [(UIPinchGestureRecognizer *)sender velocity];
    
    NSLog(@"Pinch - scale = %f, velocity = %f", scale, velocity);
    
    zoom *= scale;
    [self generateCamPos];
    
	lastScale = [(UIPinchGestureRecognizer*)sender scale];
}

@end

