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

@synthesize pickerView;

- (void)viewDidLoad
{
    [super viewDidLoad];
    [self.view setMultipleTouchEnabled:NO];
	// Do any additional setup after loading the view, typically from a nib.
    
    imageTypes  = [[NSMutableArray alloc] init];
    [imageTypes addObject:@"RGB"];
    [imageTypes addObject:@"Depth"];
    [imageTypes addObject:@"IR"];
    
    [pickerView selectRow:0 inComponent:0 animated:NO];
    
    ros_controller_ = new RosVideo();
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
    NSLog(@"didReceiveMemoryWarning");
}

- (void)viewWillAppear:(BOOL)animated
{
    NSLog(@"viewWillAppear");
    ros_controller_->view_controller_ = self;
}

- (void)viewWillDisappear:(BOOL)animated
{
    NSLog(@"viewWillDisappear");
    ros_controller_->view_controller_ = nil;
}

-(void)dealloc
{
    NSLog(@"dealloc");
    delete ros_controller_;
}

-(NSInteger)numberOfComponentsInPickerView:(UIPickerView *)pickerView;
{
    return 1;
}

-(void)pickerView:(UIPickerView *)pickerView didSelectRow:(NSInteger)row inComponent:(NSInteger)component
{
    std::string * type = new std::string([[imageTypes objectAtIndex:row] UTF8String]);
    ros_controller_->subscribe_to(*type);
}

-(NSInteger)pickerView:(UIPickerView *)pickerView numberOfRowsInComponent:(NSInteger)component;
{
    return [imageTypes count];
}

-(NSString *)pickerView:(UIPickerView *)pickerView titleForRow:(NSInteger)row forComponent:(NSInteger)component;
{
    return [imageTypes objectAtIndex:row];
}

@end
