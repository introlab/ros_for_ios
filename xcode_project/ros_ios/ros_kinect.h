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
#import <boost/thread/mutex.hpp>

#import <image_transport/image_transport.h>

@class PointCloudViewController;

class RosKinect
{
public:
    PointCloudViewController __weak * view_controller_;
    
    RosKinect();
    ~RosKinect();
    void rosSpin();
    void mtxRGBLock();
    void mtxRGBUnlock();
    void mtxDepthLock();
    void mtxDepthUnlock();
    
    unsigned int getWdth();
    unsigned int getHeight();
    bool newRGBDataAvailable();
    bool newDepthDataAvailable();
    unsigned char * getRGB();
    unsigned char * getDepth();
    float * get_P();
    
private:
    ros::NodeHandle n_;
    boost::thread * ros_thread_;
    boost::mutex mtx_rgb_;
    boost::mutex mtx_depth_;
    
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
