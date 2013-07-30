//
//  ros_kinect.mm
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "ros_kinect.h"
#import "PointCloudViewController.h"
#import <sensor_msgs/image_encodings.h>

RosKinect::RosKinect()
{    
    ros_thread_ = new boost::thread(&RosKinect::ros_spin, this);
    it_ = new image_transport::ImageTransport(n_);
    
    image_transport::TransportHints hints_1("x264", ros::TransportHints());
    it_sub_video_ = it_->subscribe("/openni/rgb/image_rect_color", 1, &RosKinect::imageCB, this, hints_1);
    image_transport::TransportHints hints_2("compressed_depth", ros::TransportHints());
    it_sub_depth_ = it_->subscribe("/openni/depth_registered/image_rect", 1, &RosKinect::depthCB, this, hints_2);
    
    cam_info_sub_ = n_.subscribe("/openni/rgb/camera_info", 1, &RosKinect::camInfoCB, this);
    
    rgb_image.width = 0;
    rgb_image.height= 0;
    new_rgb_data = false;
    new_depth_data = false;
}

RosKinect::~RosKinect()
{
    ros::shutdown();
    ros_thread_->join();
    delete ros_thread_;
    delete it_;
}

void RosKinect::ros_spin()
{
    ros::spin();
}

void RosKinect::mutex_rgb_lock()
{
    mutex_rgb.lock();
}

void RosKinect::mutex_rgb_unlock()
{
    mutex_rgb.unlock();
}

void RosKinect::mutex_depth_lock()
{
    mutex_depth.lock();
}

void RosKinect::mutex_depth_unlock()
{
    mutex_depth.unlock();
}

void RosKinect::imageCB(const sensor_msgs::ImageConstPtr & msg)
{
    if(mutex_rgb.try_lock())
    {
        //ROS_INFO("Image Received");
        rgb_image=*msg;
        new_rgb_data = true;
        mutex_rgb.unlock();
    }
}

void RosKinect::depthCB(const sensor_msgs::ImageConstPtr & msg)
{
    if(mutex_depth.try_lock())
    {
        //ROS_INFO("Depth Received");
        depth_image=*msg;
        new_depth_data = true;
        mutex_depth.unlock();
    }
}

void RosKinect::camInfoCB(const sensor_msgs::CameraInfoPtr & msg)
{
    for(int i = 0; i < 12; i++)
        P[i] = msg->P[i];
    
    for(int i = 12; i < 15; i++)
        P[i] = 0.0;
    
    P[15] = 1.0;
    
    cam_info_sub_.shutdown();
}

unsigned int RosKinect::get_width()
{
    return rgb_image.width;
}

unsigned int RosKinect::get_height()
{
    return rgb_image.height;
}

bool RosKinect::new_rgb_data_available()
{
    return new_rgb_data;
}

bool RosKinect::new_depth_data_available()
{
    return new_depth_data;
}

unsigned char * RosKinect::get_rgb()
{
    new_rgb_data = false;
    return &rgb_image.data[0];
}

unsigned char * RosKinect::get_depth()
{
    new_depth_data = false;
    return &depth_image.data[0];
}

float * RosKinect::get_P()
{
    return P;
}