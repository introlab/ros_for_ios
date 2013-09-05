//
//  ros_video.mm
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "ros_video.h"
#import "VideoViewController.h"
#import <sensor_msgs/image_encodings.h>
#import <cv_bridge/cv_bridge.h>

RosVideo::RosVideo()
{    
    ros_thread_ = new boost::thread(&RosVideo::rosSpin, this);
    it_ = new image_transport::ImageTransport(n_);
    this->subscribeTo("RGB");
    
    it_pub_ = it_->advertise("ios_camera", 1);
}

RosVideo::~RosVideo()
{
    ros::shutdown();
    ros_thread_->join();
    delete ros_thread_;
    delete it_;
}

void RosVideo::rosSpin()
{
    ros::spin();
}

void RosVideo::subscribeTo(std::string name)
{
    image_transport::TransportHints hints("x264", ros::TransportHints());
    it_sub_.shutdown();
    
    if(name.compare("RGB") == 0)
    {
        it_sub_ = it_->subscribe("/openni/rgb/image_rect_color", 1, &RosVideo::imageCB, this, hints);
    }
    else if(name.compare("Depth") == 0)
    {
        it_sub_ = it_->subscribe("/openni/depth/image_raw", 1, &RosVideo::imageCB, this, hints);
    }
    else if(name.compare("IR") == 0)
    {
        it_sub_ = it_->subscribe("/openni/ir/image_raw", 1, &RosVideo::imageCB, this, hints);
    }
}

void RosVideo::sendImage(cv::Mat & image)
{
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = ros::Time::now();
    out_msg.header.frame_id = "ios_camera_link";
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = image;
        
    it_pub_.publish(out_msg.toImageMsg());
}

void RosVideo::imageCB(const sensor_msgs::ImageConstPtr & msg)
{
    ROS_INFO("Image Received");
          
    unsigned int width = msg->width;
    unsigned int height = msg->height;
    unsigned char * data = (unsigned char *) msg->data.data();
    
    
    CGDataProviderRef provider = CGDataProviderCreateWithData(NULL,
                                                              data,
                                                              msg->data.size(),
                                                              NULL);
    
    int numChannels = sensor_msgs::image_encodings::numChannels(msg->encoding);
    int bitsPerComponent = sensor_msgs::image_encodings::bitDepth(msg->encoding);
    int bitsPerPixel = numChannels * bitsPerComponent;
    int bytesPerRow = bitsPerComponent/8 * numChannels * width;
    
    CGColorSpaceRef colorSpaceRef;
    
    if(sensor_msgs::image_encodings::isColor(msg->encoding))
    {
        colorSpaceRef = CGColorSpaceCreateDeviceRGB();
    }
    else
    {
        colorSpaceRef = CGColorSpaceCreateDeviceGray();
    }
    
    CGBitmapInfo bitmapInfo = kCGBitmapByteOrderDefault;
    CGColorRenderingIntent renderingIntent = kCGRenderingIntentDefault;
    
    CGImageRef imageRef = CGImageCreate(width,
                                        height,
                                        bitsPerComponent,
                                        bitsPerPixel,
                                        bytesPerRow,
                                        colorSpaceRef,
                                        bitmapInfo,
                                        provider,NULL,NO,renderingIntent);
    
    @autoreleasepool
    {
        UIImage * image = [UIImage imageWithCGImage:imageRef];
    
        if(view_controller_ != nil)
        {
            [view_controller_.imageView performSelectorOnMainThread:@selector(setImage:) withObject:image waitUntilDone:YES];
        }
    }
    
    CGDataProviderRelease(provider);
    CGColorSpaceRelease(colorSpaceRef);
    CGImageRelease(imageRef);
}