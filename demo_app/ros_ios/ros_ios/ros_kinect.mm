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
    ros_thread_ = new boost::thread(&RosKinect::rosSpin, this);
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

void RosKinect::rosSpin()
{
    ros::spin();
}

void RosKinect::mtxRGBLock()
{
    mtx_rgb_.lock();
}

void RosKinect::mtxRGBUnlock()
{
    mtx_rgb_.unlock();
}

void RosKinect::mtxDepthLock()
{
    mtx_depth_.lock();
}

void RosKinect::mtxDepthUnlock()
{
    mtx_depth_.unlock();
}

void RosKinect::imageCB(const sensor_msgs::ImageConstPtr & msg)
{
    if(mtx_rgb_.try_lock())
    {
        //ROS_INFO("Image Received");
        rgb_image=*msg;
        new_rgb_data = true;
        mtx_rgb_.unlock();
    }
}

void RosKinect::depthCB(const sensor_msgs::ImageConstPtr & msg)
{
    if(mtx_depth_.try_lock())
    {
        //ROS_INFO("Depth Received");
        depth_image=*msg;
        new_depth_data = true;
        mtx_depth_.unlock();
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

unsigned int RosKinect::getWdth()
{
    return rgb_image.width;
}

unsigned int RosKinect::getHeight()
{
    return rgb_image.height;
}

bool RosKinect::newRGBDataAvailable()
{
    return new_rgb_data;
}

bool RosKinect::newDepthDataAvailable()
{
    return new_depth_data;
}

unsigned char * RosKinect::getRGB()
{
    new_rgb_data = false;
    return &rgb_image.data[0];
}

unsigned char * RosKinect::getDepth()
{
    new_depth_data = false;
    return &depth_image.data[0];
}

float * RosKinect::get_P()
{
    return P;
}