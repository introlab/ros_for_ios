//
//  MapViewController.mm
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "MapViewController.h"

typedef struct {
    float Position[3];
    float Color[4];
    float TexCoord[2];
} Vertex;

const Vertex Vertices[] = {
    {{1, -1, 0}, {1, 0}},
    {{1, 1, 0}, {1, 1}},
    {{-1, 1, 0}, {0, 1}},
    {{-1, -1, 0}, {0, 0}}
};

const GLubyte Indices[] = {
    0, 1, 2,
    2, 3, 0
};

@interface MapViewController () {
    
}

@end

@implementation MapViewController

@synthesize glContext = glContext_;
@synthesize glLayer = glLayer_;

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
    
    ros_controller_ = new RosPlanner();
    ros_controller_->view_controller_ = self;
    
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
}

- (void)render
{
    if(ros_controller_->new_map_available())
    {
        [self updateTexture];
    }
    
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    
    float scale = 1.0f;
    if ([[UIScreen mainScreen] respondsToSelector:@selector(displayLinkWithTarget:selector:)]) {
        scale = [[UIScreen mainScreen] scale];
    }
    
    float h = 1 * scale * self.view.bounds.size.height / self.view.bounds.size.width;
    
    GLKMatrix4 projectionMatrix = GLKMatrix4MakeFrustum(-1,1,-h/2,h/2,1,10);
    glUniformMatrix4fv(projectionUniform_, 1, 0, projectionMatrix.m);
    
    GLKMatrix4 rotation = GLKMatrix4MakeWithQuaternion(quat_);
    GLKMatrix4 modelViewMatrix = GLKMatrix4Multiply(transMatrix_, rotation);
    
    glUniformMatrix4fv(modelViewUniform_, 1, 0, modelViewMatrix.m);
    
    glViewport(0, 0, self.view.bounds.size.width * scale, self.view.bounds.size.height * scale);
    
    glVertexAttribPointer(positionSlot_, 3, GL_FLOAT, GL_FALSE,
                          sizeof(Vertex), 0);
    glVertexAttribPointer( texCoordSlot_, 2, GL_FLOAT, GL_FALSE,
                          sizeof(Vertex), (GLvoid*) (sizeof(float) * 3));
    
    glDrawElements(GL_TRIANGLES, sizeof(Indices)/sizeof(Indices[0]),
                   GL_UNSIGNED_BYTE, 0);
    
    [self.glContext presentRenderbuffer:GL_RENDERBUFFER];
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
    
    projectionUniform_ = glGetUniformLocation(programHandle, "Projection");
    modelViewUniform_ = glGetUniformLocation(programHandle, "Modelview");
    
     texCoordSlot_ = glGetAttribLocation(programHandle, "TexCoordIn");
    glEnableVertexAttribArray(texCoordSlot_);
     textureUniform_ = glGetUniformLocation(programHandle, "Texture");
}

- (void)setupVBOs
{
    GLuint vertexBuffer;
    glGenBuffers(1, &vertexBuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertices), Vertices, GL_STATIC_DRAW);
    
    GLuint indexBuffer;
    glGenBuffers(1, &indexBuffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Indices), Indices, GL_STATIC_DRAW);
}

- (void)setupTexture
{
    glBindTexture(GL_TEXTURE_2D, texture_);
    
    // use linear filetring
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    // clamp to edge
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

- (void)updateTexture
{
    size_t width = ros_controller_->get_map_width();
    size_t height = ros_controller_->get_map_height();
    GLubyte * mapData = (GLubyte *) ros_controller_->get_map_data();
    
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, mapData);
}


- (void)setupGL
{
    [self setupLayer];
    [self setupContext];
    [self setupRenderBuffer];
    [self setupFrameBuffer];
    [self compileShaders];
    [self setupTexture];
    [self setupVBOs];
    [self setupGeometry];
}

- (void)setupGeometry
{
    transMatrix_ = GLKMatrix4MakeTranslation(0.0f, 0.0f, -3.0f);
    rotMatrix_ = GLKMatrix4Identity;
    quat_ = GLKQuaternionMake(0, 0, 0, 1);
    quatStart_ = GLKQuaternionMake(0, 0, 0, 1);
}

- (void)glkView:(GLKView *)view drawInRect:(CGRect)rect {
    
}

- (void)update
{
    [self render];
}

- (GLKVector3) projectOntoSurface:(GLKVector3) touchPoint
{
    float radius = self.view.bounds.size.width/3;
    GLKVector3 center = GLKVector3Make(self.view.bounds.size.width/2, self.view.bounds.size.height/2, 0);
    GLKVector3 P = GLKVector3Subtract(touchPoint, center);
    
    // Flip the y-axis because pixel coords increase toward the bottom.
    P = GLKVector3Make(P.x, P.y * -1, P.z);
    
    float radius2 = radius * radius;
    float length2 = P.x*P.x + P.y*P.y;
    
    if (length2 <= radius2)
        P.z = sqrt(radius2 - length2);
    else
    {
        /*
         P.x *= radius / sqrt(length2);
         P.y *= radius / sqrt(length2);
         P.z = 0;
         */
        P.z = radius2 / (2.0 * sqrt(length2));
        float length = sqrt(length2 + P.z * P.z);
        P = GLKVector3DivideScalar(P, length);
    }
    
    return GLKVector3Normalize(P);
}

- (void)computeIncremental
{
    GLKVector3 axis = GLKVector3CrossProduct(anchor_position_, current_position_);
    float dot = GLKVector3DotProduct(anchor_position_, current_position_);
    float angle = acosf(dot);
    
    GLKQuaternion Q_rot = GLKQuaternionMakeWithAngleAndVector3Axis(angle * 2, axis);
    Q_rot = GLKQuaternionNormalize(Q_rot);
    
    // TODO: Do something with Q_rot...
    quat_ = GLKQuaternionMultiply(Q_rot, quatStart_);
}

- (void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event
{    
    UITouch * touch = [touches anyObject];
    CGPoint location = [touch locationInView:self.view];
    
    anchor_position_ = GLKVector3Make(location.x, location.y, 0);
    anchor_position_ = [self projectOntoSurface:anchor_position_];
    
    current_position_ = anchor_position_;
    quatStart_ = quat_;
}

- (void)touchesMoved:(NSSet *)touches withEvent:(UIEvent *)event
{
    UITouch * touch = [touches anyObject];
    CGPoint location = [touch locationInView:self.view];
    CGPoint lastLoc = [touch previousLocationInView:self.view];
    CGPoint diff = CGPointMake(lastLoc.x - location.x, lastLoc.y - location.y);
    
    float rotX = -1 * GLKMathDegreesToRadians(diff.y / 2.0);
    float rotY = -1 * GLKMathDegreesToRadians(diff.x / 2.0);
    
    bool isInvertible;
    GLKVector3 xAxis = GLKMatrix4MultiplyVector3(GLKMatrix4Invert(rotMatrix_, &isInvertible), GLKVector3Make(1, 0, 0));
    rotMatrix_ = GLKMatrix4Rotate(rotMatrix_, rotX, xAxis.x, xAxis.y, xAxis.z);
    GLKVector3 yAxis = GLKMatrix4MultiplyVector3(GLKMatrix4Invert(rotMatrix_, &isInvertible), GLKVector3Make(0, 1, 0));
    rotMatrix_ = GLKMatrix4Rotate(rotMatrix_, rotY, yAxis.x, yAxis.y, yAxis.z);
    
    current_position_ = GLKVector3Make(location.x, location.y, 0);
    current_position_ = [self projectOntoSurface:current_position_];
    
    [self computeIncremental];
}

- (IBAction)buttonGoPressed:(id)sender
{
    CGPoint goal;
    
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

- (IBAction)tapDetected:(UITapGestureRecognizer *)sender
{
    NSLog(@"Double tap!");
    [self setupGeometry];
}

- (IBAction)pinchDetected:(UIPinchGestureRecognizer *)sender
{
    static CGFloat lastScale;
    if([(UIPinchGestureRecognizer*)sender state] == UIGestureRecognizerStateEnded) {
        lastScale = 1.0;
         return;
	}
    
    CGFloat scale = 1.0 - (lastScale - [(UIPinchGestureRecognizer*)sender scale]);
    CGFloat velocity = [(UIPinchGestureRecognizer *)sender velocity];
    
    NSLog(@"Pinch - scale = %f, velocity = %f", scale, velocity);
    
    transMatrix_ = GLKMatrix4Multiply(transMatrix_,GLKMatrix4MakeScale(scale,scale,scale));
	lastScale = [(UIPinchGestureRecognizer*)sender scale];
}

@end
