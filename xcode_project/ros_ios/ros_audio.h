//
//  ros_audio.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#ifndef ros_audio_h
#define ros_audio_h

#import <ros/node_handle.h>
#import <ros/publisher.h>
#import <ros/subscriber.h>
#import <rt_audio_ros/AudioStream.h>
#import <boost/thread/thread.hpp>
#import <boost/thread/mutex.hpp>

#import <vector>

@class AudioViewController;

class RosAudio
{
public:
    static const int SAMPLE_RATE = 44100;
    static const int CHANNELS_NUMBER = 1;
    
    AudioViewController __weak * view_controller_;
    
    RosAudio();
    ~RosAudio();
    void rosSpin();
    
    void lockAudioBuffer();
    void unlockAudioBuffer();
    std::vector<unsigned char> * getAudioBuffer();
    void sendAudio(std::vector<unsigned char> data);
    
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    
    boost::thread * ros_thread_;
    boost::mutex mtx_;
    std::vector<unsigned char> buffer_;
    
    void audioCB(const rt_audio_ros::AudioStreamConstPtr & msg);
};

#endif
