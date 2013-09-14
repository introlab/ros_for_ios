//
//  VideoViewController.mm
//  ros_video
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "VideoViewController.h"

@interface VideoViewController ()

@end

@implementation VideoViewController

@synthesize pickerView, videoCamera;

- (void)viewDidLoad
{
    [super viewDidLoad];
    [self.view setMultipleTouchEnabled:NO];
    // Do any additional setup after loading the view, typically from a nib.
    NSLog(@"VideoViewController : viewDidLoad");
    
    ros_controller_ = new RosVideo();
    
    imageTypes = [[NSMutableArray alloc] init];
    [imageTypes addObject:@"RGB"];
    [imageTypes addObject:@"Depth"];
    [imageTypes addObject:@"IR"];
    
    [pickerView selectRow:0 inComponent:0 animated:NO];
    
    self.videoCamera = [[CvVideoCamera alloc] init];
    self.videoCamera.delegate = self;
    self.videoCamera.defaultAVCaptureDevicePosition = AVCaptureDevicePositionBack;
    self.videoCamera.defaultAVCaptureSessionPreset = AVCaptureSessionPreset640x480;
    self.videoCamera.defaultAVCaptureVideoOrientation = AVCaptureVideoOrientationPortrait;
    self.videoCamera.defaultFPS = 1;
    
    [self.videoCamera start];
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
    NSLog(@"didReceiveMemoryWarning");
}

- (void)viewWillAppear:(BOOL)animated
{
    NSLog(@"VideoViewController : viewWillAppear");
    ros_controller_->view_controller_ = self;
}

- (void)viewWillDisappear:(BOOL)animated
{
    NSLog(@"VideoViewController : viewWillDisappear");
    [self.videoCamera stop];
    delete ros_controller_;
}

-(void)dealloc
{
    NSLog(@"VideoViewController : dealloc");
}

-(NSInteger)numberOfComponentsInPickerView:(UIPickerView *)pickerView;
{
    return 1;
}

-(void)pickerView:(UIPickerView *)pickerView didSelectRow:(NSInteger)row inComponent:(NSInteger)component
{
    std::string * type = new std::string([[imageTypes objectAtIndex:row] UTF8String]);
    ros_controller_->subscribeTo(*type);
}

-(NSInteger)pickerView:(UIPickerView *)pickerView numberOfRowsInComponent:(NSInteger)component;
{
    return [imageTypes count];
}

-(NSString *)pickerView:(UIPickerView *)pickerView titleForRow:(NSInteger)row forComponent:(NSInteger)component;
{
    return [imageTypes objectAtIndex:row];
}

#ifdef __cplusplus
- (void)processImage:(cv::Mat&)image;
{
    cv::Mat image_copy;
    cv::cvtColor(image, image_copy, CV_BGRA2BGR);
    ros_controller_->sendImage(image_copy);
}
#endif

@end
