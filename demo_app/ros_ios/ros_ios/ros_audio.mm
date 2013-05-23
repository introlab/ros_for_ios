//
//  ros_audio.mm
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "ros_audio.h"
#import "AudioViewController.h"

RosAudio::RosAudio()
{
    pub_ = n_.advertise<gstreamer_plugins::AudioStream>("/iphone_mic", 1);
    sub_ = n_.subscribe("/audio", 1, &RosAudio::audioCB, this);
    
    ros_thread_ = new boost::thread(&RosAudio::ros_spin, this);
    
    buffer = [[NSMutableArray alloc] init];
}

RosAudio::~RosAudio()
{
    ros::shutdown();
    ros_thread_->join();
    delete ros_thread_;
}

void RosAudio::ros_spin()
{
    ros::spin();
}

void RosAudio::audioCB(const gstreamer_plugins::AudioStreamConstPtr & msg)
{
    NSLog(@"Audio received : encoding %d endianness %d channels %d sample_rate %d size %d", msg->encoding, msg->is_bigendian, msg->channels, msg->sample_rate, (unsigned int)msg->data.size());
    
    [buffer removeAllObjects];
    
    for (int i = 0; i < msg->data.size(); i++)
    {
        [buffer addObject:[NSNumber numberWithInt:msg->data[i]]];
    }

    @autoreleasepool
    {
        if(view_controller_ != nil && !view_controller_.isPaused)
        {
            @synchronized(view_controller_.inputBuffer)
            {
                [view_controller_.inputBuffer performSelectorOnMainThread:@selector(addObjectsFromArray:) withObject:buffer waitUntilDone:YES];
            }
        }
    }
}

void RosAudio::sendAudio(std::vector<unsigned char> data)
{
    gstreamer_plugins::AudioStream msg;
    msg.encoding = msg.SINT_16_PCM;
    msg.is_bigendian = false;
    msg.channels = 1;
    msg.sample_rate = 44100;
    msg.data = data;
    
    pub_.publish(msg);
}