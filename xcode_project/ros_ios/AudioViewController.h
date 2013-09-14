//
//  AudioViewController.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-04-11.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>
#import <AudioToolbox/AudioToolbox.h>
#import "ros_audio.h"

@interface AudioViewController : UIViewController
{
    @public
    RosAudio * ros_controller_;
    @private
    BOOL isPaused;
}

@property (readonly) BOOL isPaused;
@property (readonly) AudioComponentInstance audioUnit;
@property(nonatomic, retain) IBOutlet UIBarButtonItem *pause;

- (BOOL)initAudio;
- (void) processAudio: (AudioBufferList*) bufferList;
- (IBAction)pauseOrResume:(id)sender;
- (void) start;
- (void) stop;

@end
