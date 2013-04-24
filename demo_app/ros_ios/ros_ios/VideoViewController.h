//
//  VideoViewController.h
//  ros_video
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "ros_video.h"

@interface VideoViewController :UIViewController<UIPickerViewDelegate, UIPickerViewDataSource>
{
    UIPickerView * pickerView;
    NSMutableArray * imageTypes;
    NSTimer * timer;
    RosVideo * ros_controller_;
}

@property (weak, nonatomic) IBOutlet UIImageView *imageView;
@property (strong, nonatomic) IBOutlet UIPickerView *pickerView;

@end
