//
//  ros_video.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#ifndef ros_video_h
#define ros_video_h

#import <ros/node_handle.h>
#import <ros/subscriber.h>
#import <sensor_msgs/Image.h>
#import <boost/scoped_ptr.hpp>
#import <boost/thread/thread.hpp>

#include <string>

@class VideoViewController;

class RosVideo
{
public:
    VideoViewController __weak * view_controller_;
    
    RosVideo();
    ~RosVideo();
    void ros_spin();
    void subscribe_to(std::string name);
    
private:
    ros::NodeHandle n_;
    boost::scoped_ptr<ros::Subscriber> sub_;

    boost::thread * ros_thread_;

    void imageCB(const sensor_msgs::ImageConstPtr & msg);
};

#endif
