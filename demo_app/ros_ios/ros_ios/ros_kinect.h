//
//  ros_kinect.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#ifndef ros_kinect_h
#define ros_kinect_h

#import <ros/node_handle.h>
#import <ros/subscriber.h>
#import <sensor_msgs/Image.h>
#import <sensor_msgs/CameraInfo.h>
#import <boost/scoped_ptr.hpp>
#import <boost/thread/thread.hpp>
#import <boost/interprocess/sync/interprocess_mutex.hpp>

#import <image_transport/image_transport.h>

@class PointCloudViewController;

class RosKinect
{
public:
    PointCloudViewController __weak * view_controller_;
    
    RosKinect();
    ~RosKinect();
    void ros_spin();
    void mutex_rgb_lock();
    void mutex_rgb_unlock();
    void mutex_depth_lock();
    void mutex_depth_unlock();
    
    unsigned int get_width();
    unsigned int get_height();
    bool new_rgb_data_available();
    bool new_depth_data_available();
    unsigned char * get_rgb();
    unsigned char * get_depth();
    float * get_P();
    
private:
    ros::NodeHandle n_;
    
    boost::thread * ros_thread_;
    boost::interprocess::interprocess_mutex mutex_rgb;
    boost::interprocess::interprocess_mutex mutex_depth;
    
    image_transport::ImageTransport * it_;
    image_transport::Subscriber it_sub_video_;
    image_transport::Subscriber it_sub_depth_;
    
    ros::Subscriber cam_info_sub_;

    sensor_msgs::Image rgb_image;
    sensor_msgs::Image depth_image;
    
    void imageCB(const sensor_msgs::ImageConstPtr & msg);
    void depthCB(const sensor_msgs::ImageConstPtr & msg);
    void camInfoCB(const sensor_msgs::CameraInfoPtr & msg);

    float P[16];
    
    bool new_rgb_data;
    bool new_depth_data;
};

#endif
