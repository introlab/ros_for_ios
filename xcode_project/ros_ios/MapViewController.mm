//
//  MapViewController.mm
//  robot_help_me
//
//  Created by Ronan Chauvin on 2013-08-04.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "MapViewController.h"
#import "robot_geometry.h"

typedef struct _vertexStruct
{
    GLfloat position[3];
    GLfloat color[3];
} vertexStruct;

typedef enum
{
    FIXED,
    FOLLOW,
    NB_VIEWS
} viewType;

@interface MapViewController ()
{
    vertexStruct * vertices;
    GLuint * indices;
    
    float rot_alpha;
    float rot_gamma;
    float zoom;
    
    int count;
    viewType view_type;
}

@end

@implementation MapViewController

@synthesize viewTypeSelector, glContext, glLayer;

- (id)initWithNibName:(NSString *)nibNameOrNil bundle:(NSBundle *)nibBundleOrNil
{
    self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
    if (self) {
        // Custom initialization
    }
    return self;
}

- (void)viewDidLoad
{
    [super viewDidLoad];
	// Do any additional setup after loading the view.
    
    NSLog(@"MapViewController : viewDidLoad");
    
    ros_controller_ = new RosPlanner();
    ros_controller_->view_controller_ = self;

    [self setupGL];
    
    //GLKView *view = (GLKView *)self.view;
    //view.context = self.glContext;
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

-(void)dealloc
{
    NSLog(@"MapViewController : dealloc");
    delete ros_controller_;
    
    if ([EAGLContext currentContext] == self.glContext) {
        [EAGLContext setCurrentContext:nil];
    }
    self.glContext = nil;
}

- (void)setupGL
{
    indices = nil;
    vertices = nil;
    count = 0;
    
    [self setupLayer];
    [self setupContext];
    [self setupFrameBuffer];
    [self setupRenderBuffer];
    [self compileShaders];
    [self setupVBOs];
    [self setupVAOs];
    [self setupGeometry];
    
    //General settings
    //glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    
    CGFloat scale = 1.0f;
    if ([[UIScreen mainScreen] respondsToSelector:@selector(displayLinkWithTarget:selector:)])
    {
        scale = [[UIScreen mainScreen] scale];
    }
    glViewport(0, 0, self.view.bounds.size.width * scale, self.view.bounds.size.height * scale);
    
    view_type = FIXED;
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
    
    GLint width;
    GLint height;
    glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_WIDTH, &width);
    glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_HEIGHT, &height);
    
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                              GL_RENDERBUFFER, colorRenderBuffer_);
    
    glGenRenderbuffers(1, &depthRenderBuffer_);
    glBindRenderbuffer(GL_RENDERBUFFER, depthRenderBuffer_);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, width, height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                              GL_RENDERBUFFER, depthRenderBuffer_);
    
    glBindRenderbuffer(GL_RENDERBUFFER, colorRenderBuffer_);
    
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER) ;
    if(status != GL_FRAMEBUFFER_COMPLETE) {
        NSLog(@"failed to make complete framebuffer object %x", status);
    }
}

- (void)setupFrameBuffer
{
    glGenFramebuffers(1, &framebuffer_);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
}

- (void)setupVBOs
{
    glGenBuffers(1, &vertexBufferMap_);
    glGenBuffers(1, &indexBufferMap_);
    glGenBuffers(1, &vertexBufferRobot_);
    glGenBuffers(1, &indexBufferRobot_);
}

- (void)setupVAOs
{
    //Map
    // Create and bind the vertex array object.
    glGenVertexArraysOES(1,&vao1_);
    glBindVertexArrayOES(vao1_);
    // Configure the attributes in the VAO.
    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferMap_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertexStruct[8]) * count, vertices, GL_STATIC_DRAW);
    
    glVertexAttribPointer(positionMapSlot_, 3, GL_FLOAT, GL_FALSE,
                          sizeof(vertexStruct),
                          (void*)offsetof(vertexStruct,position));
    glEnableVertexAttribArray(positionMapSlot_);
    
    glVertexAttribPointer(sourceMapSlot_, 3, GL_FLOAT, GL_FALSE,
                          sizeof(vertexStruct),
                          (void*)offsetof(vertexStruct,color));
    glEnableVertexAttribArray(sourceMapSlot_);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBufferMap_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint[35]) * count, indices, GL_STATIC_DRAW);
    
    //Robot
    // Create and bind the vertex array object.
    glGenVertexArraysOES(1,&vao2_);
    glBindVertexArrayOES(vao2_);
    // Configure the attributes in the VAO.
    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferRobot_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(struct vertex_struct)*VERTEX_COUNT, vertexs, GL_STATIC_DRAW);
    glVertexAttribPointer(positionRobotSlot_, 3, GL_FLOAT, GL_FALSE,
                          sizeof(struct vertex_struct),
                          (void*)offsetof(struct vertex_struct,x));
    glEnableVertexAttribArray(positionRobotSlot_);
    glVertexAttribPointer(sourceRobotSlot_, 3, GL_FLOAT, GL_FALSE,
                          sizeof(struct vertex_struct),
                          (void*)offsetof(struct vertex_struct,nx));
    glEnableVertexAttribArray(sourceRobotSlot_);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBufferRobot_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indexes[0])*FACES_COUNT*3, indexes, GL_STATIC_DRAW);
    
    // Bind back to the default state.
    glBindVertexArrayOES(0);
}

- (void)setupGeometry
{
    rot_alpha = 0.0;
    rot_gamma = M_PI/4.0;
    zoom = -20.0;
    [self generateCamPos];
    
    center_.x = 0.0;
    center_.y = 0.0;
    center_.z = 0.0;
    
    projectionMatrix_ = GLKMatrix4MakePerspective(M_PI/2.0, 1.0, 1.0, 100.0);
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
    GLuint vertexShader = [self compileShader:@"NavigationVertex"
                                     withType:GL_VERTEX_SHADER];
    GLuint mapFragmentShader = [self compileShader:@"MapFragment"
                                          withType:GL_FRAGMENT_SHADER];
    GLuint robotFragmentShader = [self compileShader:@"RobotFragment"
                                            withType:GL_FRAGMENT_SHADER];
    
    //Map
    programHandleMap_ = glCreateProgram();
    glAttachShader(programHandleMap_, vertexShader);
    glAttachShader(programHandleMap_, mapFragmentShader);
    glLinkProgram(programHandleMap_);
    
    GLint linkSuccess;
    glGetProgramiv(programHandleMap_, GL_LINK_STATUS, &linkSuccess);
    if(linkSuccess == GL_FALSE)
    {
        GLchar messages[256];
        glGetProgramInfoLog(programHandleMap_, sizeof(messages), 0, &messages[0]);
        NSString *messageString = [NSString stringWithUTF8String:messages];
        NSLog(@"%@", messageString);
        exit(1);
    }
    
    positionMapSlot_ = glGetAttribLocation(programHandleMap_, "Position");
    sourceMapSlot_ = glGetAttribLocation(programHandleMap_, "Source");
    projectionMaptUniform_ = glGetUniformLocation(programHandleMap_, "Projection");
    modelViewMapUniform_ = glGetUniformLocation(programHandleMap_, "Modelview");
    
    //Robot
    programHandleRobot_ = glCreateProgram();
    glAttachShader(programHandleRobot_, vertexShader);
    glAttachShader(programHandleRobot_, robotFragmentShader);
    glLinkProgram(programHandleRobot_);
    
    glGetProgramiv(programHandleRobot_, GL_LINK_STATUS, &linkSuccess);
    if(linkSuccess == GL_FALSE)
    {
        GLchar messages[256];
        glGetProgramInfoLog(programHandleRobot_, sizeof(messages), 0, &messages[0]);
        NSString *messageString = [NSString stringWithUTF8String:messages];
        NSLog(@"%@", messageString);
        exit(1);
    }
    
    positionRobotSlot_ = glGetAttribLocation(programHandleRobot_, "Position");
    sourceRobotSlot_ = glGetAttribLocation(programHandleRobot_, "Source");
    projectionRobotUniform_ = glGetUniformLocation(programHandleRobot_, "Projection");
    modelViewRobotUniform_ = glGetUniformLocation(programHandleRobot_, "Modelview");
}

- (void)update
{
    [self render];
}

- (void)updateMap
{
    ros_controller_->lockMap();
    
    unsigned int width = ros_controller_->getMapWidth();
    unsigned int height = ros_controller_->getMapHeight();
    signed char * map_data = ros_controller_->getMap();
    float res = ros_controller_->getMapResolution();
    float org_x = ros_controller_->getMapOriginX();
    float org_y = ros_controller_->getMapOriginY();
    float res_2 = res/2.0;
    float real_w = width * res;
    float real_h = width * res;
    
    //Generate the geometry
    //Even if the size is divided by 16, there is plenty space...
    if(vertices == nil)
        vertices = (vertexStruct *) malloc(sizeof(vertexStruct[6])*width*height/16);
    if(indices == nil)
        indices = (GLuint *) malloc(sizeof(GLuint[36])*width*height/16);
    
    count = 0;
    
    for(size_t i = 0; i != height; ++i)
    {
        for(size_t j = 0; j != width; ++j)
        {
            int quad_index = 8 * count;
            
            GLfloat p = map_data[i*width+j];
            
            if(p > 0)
            {
                const vertexStruct quad[] =
                {
                    {   {-res_2-real_w/2.0+(j-org_x)*res,
                        -res_2-real_h/2.0+(i-org_y)*res, 1.0},
                        {0.5, 0.5, 0.5}
                    },
                    {   {res_2-real_w/2.0+(j-org_x)*res,
                        -res_2-real_h/2.0+(i-org_y)*res, 1.0},
                        {0.5, 0.5, 0.5}
                    },
                    {   {res_2-real_w/2.0+(j-org_x)*res,
                        res_2-real_h/2.0+(i-org_y)*res,  1.0},
                        {0.5, 0.5, 0.5}
                    },
                    {   {-res_2-real_w/2.0+(j-org_x)*res,
                        res_2-real_h/2.0+(i-org_y)*res,  1.0},
                        {0.5, 0.5, 0.5}
                    },
                    {   {-res_2-real_w/2.0+(j-org_x)*res,
                        -res_2-real_h/2.0+(i-org_y)*res, 0.0},
                        {0.5, 0.5, 0.5}
                    },
                    {   {res_2-real_w/2.0+(j-org_x)*res,
                        -res_2-real_h/2.0+(i-org_y)*res, 0.0},
                        {0.5, 0.5, 0.5}
                    },
                    {   {res_2-real_w/2.0+(j-org_x)*res,
                        res_2-real_h/2.0+(i-org_y)*res,  0.0},
                        {0.5, 0.5, 0.5}
                    },
                    {   {-res_2-real_w/2.0+(j-org_x)*res,
                        res_2-real_h/2.0+(i-org_y)*res,  0.0},
                        {0.5, 0.5, 0.5}
                    }
                };
                
                const GLuint quadIndices[] =
                {
                    // Top
                    quad_index, quad_index+1, quad_index+2,
                    quad_index, quad_index+2, quad_index+3,
                    // Bottom
                    quad_index+4, quad_index+5, quad_index+6,
                    quad_index+4, quad_index+6, quad_index+7,
                    // Back
                    quad_index, quad_index+1, quad_index+5,
                    quad_index, quad_index+5, quad_index+4,
                    // Front
                    quad_index+3, quad_index+2, quad_index+6,
                    quad_index+3, quad_index+6, quad_index+7,
                    // Left
                    quad_index, quad_index+3, quad_index+7,
                    quad_index, quad_index+7, quad_index+4,
                    // Right
                    quad_index+2, quad_index+1, quad_index+5,
                    quad_index+2, quad_index+5, quad_index+6
                };
                
                memcpy(&vertices[quad_index], quad, sizeof(quad));
                memcpy(&indices[36*count], quadIndices, sizeof(quadIndices));
                count++;
            }
        }
    }
    
    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferMap_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertexStruct[8])*count, vertices, GL_STATIC_DRAW);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBufferMap_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint[35])*count, indices, GL_STATIC_DRAW);
    
    glVertexAttribPointer(positionMapSlot_, 3, GL_FLOAT, GL_FALSE,
                          sizeof(vertexStruct),
                          (void*)offsetof(vertexStruct,position));
    
    glVertexAttribPointer(sourceMapSlot_, 3, GL_FLOAT, GL_FALSE,
                          sizeof(vertexStruct),
                          (void*)offsetof(vertexStruct,color));
    
    ros_controller_->unlockMap();
}

- (void)glkView:(GLKView *)view drawInRect:(CGRect)rect {
    
}

- (void)render
{
    //glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    if(view_type == FIXED)
    {
        viewMatrix_ = GLKMatrix4MakeLookAt(camPos_.x, camPos_.y, camPos_.z,
                                           center_.x, center_.y, center_.z, 0.0, 0.0, 1.0);
    }
    else
    {
        GLKVector3 robot_pose = ros_controller_->getRobotPose();
        GLKVector3 offset_pose = GLKVector3Make(-2.5,0.0,2.5);
        GLKVector3 camera_pose = GLKMatrix4MultiplyVector3WithTranslation(
                                                                          ros_controller_->getRobotTransform(),
                                                                          offset_pose);
        viewMatrix_ = GLKMatrix4MakeLookAt(camera_pose.x,
                                           camera_pose.y,
                                           camera_pose.z,
                                           robot_pose.x,
                                           robot_pose.y,
                                           robot_pose.z, 0.0, 0.0, 1.0);
    }
    
    modelMatrix_ = GLKMatrix4Multiply(ros_controller_->getRobotTransform(),GLKMatrix4MakeYRotation(-M_PI/2));
    
    //Map
    glBindVertexArrayOES(vao1_);
    
    glUniformMatrix4fv(projectionMaptUniform_, 1, 0, projectionMatrix_.m);
    glUniformMatrix4fv(modelViewMapUniform_, 1, 0, viewMatrix_.m);
    
    if(ros_controller_->newMapAvailable())
    {
        [self updateMap];
    }
    
    if(count)
    {
        glDrawElements(GL_TRIANGLES, 35 * count, GL_UNSIGNED_INT, (void*)0);
    }
    
    //Robot
    glUseProgram(programHandleRobot_);
    glBindVertexArrayOES(vao2_);
    
    glUniformMatrix4fv(projectionRobotUniform_, 1, 0, projectionMatrix_.m);
    glUniformMatrix4fv(modelViewRobotUniform_, 1, 0, GLKMatrix4Multiply(viewMatrix_,modelMatrix_).m);
    
    if(count)
    {
        glDrawElements(GL_TRIANGLES, FACES_COUNT * 3, INX_TYPE, (void*)0);
    }
    
    const GLenum discards[]  = {GL_DEPTH_ATTACHMENT};
    glDiscardFramebufferEXT(GL_FRAMEBUFFER, 1, discards);
    [self.glContext presentRenderbuffer:GL_RENDERBUFFER];
}

- (void)generateCamPos
{
    camPos_.x = zoom * cosf(rot_gamma) * cosf(rot_alpha);
    camPos_.y = zoom * cosf(rot_gamma) * sinf(rot_alpha);
    camPos_.z = -1.0 * zoom * sinf(rot_gamma);
}

- (void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event
{
    
}

- (void)touchesMoved:(NSSet *)touches withEvent:(UIEvent *)event
{
    UITouch *touch = [touches anyObject];
    CGPoint location = [touch locationInView:self.view];
    CGPoint lastLocation = [touch previousLocationInView:self.view];
    CGPoint diff = CGPointMake(location.x - lastLocation.x, location.y - lastLocation.y);
    
    NSLog(@"TouchesMoved : %f ,%f.", location.x, location.y);
    
    if(view_type == FIXED)
    {
        rot_alpha += -1 * GLKMathDegreesToRadians(diff.x / 2.0);
        rot_gamma += -1 * GLKMathDegreesToRadians(diff.y / 2.0);
        
        if(rot_alpha > M_PI)
            rot_alpha = M_PI;
        else if (rot_alpha < -M_PI)
            rot_alpha = -M_PI;
        
        if(rot_gamma > M_PI/3.0)
            rot_gamma = M_PI/3.0;
        else if (rot_gamma < -M_PI/3.0)
            rot_gamma = -M_PI/3.0;
        
        [self generateCamPos];
    }
}

- (IBAction)tapDetected:(UITapGestureRecognizer *)sender
{
    if(view_type == FIXED)
    {
        [self setupGeometry];
    }
}

- (IBAction)pinchDetected:(UIPinchGestureRecognizer *)sender
{
    static CGFloat lastScale;
    
    if(view_type == FIXED)
    {
        if([(UIPinchGestureRecognizer*)sender state] == UIGestureRecognizerStateEnded)
        {
            lastScale = 1.0;
            return;
        }
        
        CGFloat scale = 1.0 + (lastScale - [(UIPinchGestureRecognizer*)sender scale]);
        CGFloat velocity = [(UIPinchGestureRecognizer *)sender velocity];
        
        NSLog(@"Pinch - scale = %f, velocity = %f", scale, velocity);
        
        zoom *= scale;
        [self generateCamPos];
        
        lastScale = [(UIPinchGestureRecognizer*)sender scale];
    }
}

- (IBAction)panDetected:(UIPanGestureRecognizer *)sender
{
    if(view_type == FIXED)
    {
        CGPoint translation = [(UIPanGestureRecognizer*)sender translationInView:self.view];
        
        NSLog(@"Pan - translation = %f, %f", translation.x, translation.y);
        
        //center_.x += translation.x;
        //center_.y += translation.y;
    }
}

- (IBAction)viewTypeChanged:(id)sender
{
    view_type = [viewTypeSelector selectedSegmentIndex]?FOLLOW:FIXED;
}

- (IBAction)buttonGoPressed:(id)sender
{
    CGPoint goal;
    
    NSLog(@"%f %f", goal.x, goal.y);
    
    if(!ros_controller_->sendGoal(goal))
    {
        UIAlertView * alert = [[UIAlertView alloc] initWithTitle:@"Wrong Goal" message:@"Move the goal to a valid position" delegate:self cancelButtonTitle:@"Ok" otherButtonTitles:nil];
        [alert show];
    }
}

@end
