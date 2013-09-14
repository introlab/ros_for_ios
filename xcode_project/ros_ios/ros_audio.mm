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
    pub_ = n_.advertise<rt_audio_ros::AudioStream>("ios_audio", 1);
    sub_ = n_.subscribe("/audio", 1, &RosAudio::audioCB, this);
    
    ros_thread_ = new boost::thread(&RosAudio::rosSpin, this);
}

RosAudio::~RosAudio()
{
    ros::shutdown();
    ros_thread_->join();
    delete ros_thread_;
}

void RosAudio::rosSpin()
{
    ros::spin();
}

void RosAudio::lockAudioBuffer()
{
    mtx_.lock();
}

void RosAudio::unlockAudioBuffer()
{
    mtx_.unlock();
}

std::vector<unsigned char> * RosAudio::getAudioBuffer()
{
    return &buffer_;
}

void RosAudio::sendAudio(std::vector<unsigned char> data)
{
    rt_audio_ros::AudioStream msg;
    msg.encoding = msg.SINT_16_PCM;
    msg.is_bigendian = false;
    msg.channels = CHANNELS_NUMBER;
    msg.sample_rate = SAMPLE_RATE;
    msg.data = data;
    
    pub_.publish(msg);
}

void RosAudio::audioCB(const rt_audio_ros::AudioStreamConstPtr & msg)
{
    //ROS_INFO("Audio received : encoding %d endianness %d channels %d sample_rate %d size %d", msg->encoding, msg->is_bigendian, msg->channels, msg->sample_rate, (unsigned int)msg->data.size());
    
    lockAudioBuffer();
    buffer_.insert(buffer_.end(),msg->data.begin(),msg->data.end());
    unlockAudioBuffer();
}