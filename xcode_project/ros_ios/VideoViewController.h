//
//  VideoViewController.h
//  ros_video
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "ros_video.h"
#import <opencv2/highgui/cap_ios.h>
#import <opencv2/imgproc/imgproc.hpp>

@interface VideoViewController : UIViewController<UIPickerViewDelegate, UIPickerViewDataSource, CvVideoCameraDelegate>
{
    NSMutableArray * imageTypes;
    RosVideo * ros_controller_;
}

@property (weak, nonatomic) IBOutlet UIImageView *imageView;
@property (strong, nonatomic) IBOutlet UIPickerView *pickerView;
@property (nonatomic, retain) CvVideoCamera *videoCamera;

@end
