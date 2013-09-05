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
#import <sensor_msgs/Image.h>
#import <boost/scoped_ptr.hpp>
#import <boost/thread/thread.hpp>

#import <image_transport/image_transport.h>
#import <opencv2/core/core.hpp>
#import <string>

@class VideoViewController;

class RosVideo
{
public:
    VideoViewController __weak * view_controller_;
    
    RosVideo();
    ~RosVideo();
    void rosSpin();
    void subscribeTo(std::string name);
    void sendImage(cv::Mat & image);
    
private:
    ros::NodeHandle n_;
    boost::thread * ros_thread_;
    
    image_transport::ImageTransport * it_;
    image_transport::Subscriber it_sub_;
    image_transport::Publisher it_pub_;
    void imageCB(const sensor_msgs::ImageConstPtr & msg);
};

#endif
