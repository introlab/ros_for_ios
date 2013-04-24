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


RosVideo::RosVideo()
{
    this->subscribe_to("RGB");
    
    ros_thread_ = new boost::thread(&RosVideo::ros_spin, this);
}

RosVideo::~RosVideo()
{
    ros::shutdown();
    ros_thread_->join();
    delete ros_thread_;
}

void RosVideo::ros_spin()
{
    ros::spin();
}

void RosVideo::subscribe_to(std::string name)
{
    if(name.compare("RGB") == 0)
        sub_.reset(new ros::Subscriber(n_.subscribe("/openni/rgb/image_rect_color", 10, &RosVideo::imageCB, this)));
    else if(name.compare("Depth") == 0)
        sub_.reset(new ros::Subscriber(n_.subscribe("/openni/depth/image_raw", 10, &RosVideo::imageCB, this)));
    else if(name.compare("IR") == 0)
        sub_.reset(new ros::Subscriber(n_.subscribe("/openni/ir/image_raw", 10, &RosVideo::imageCB, this)));
}

void RosVideo::imageCB(const sensor_msgs::ImageConstPtr & msg)
{
    //ROS_INFO("Image received");
    
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
    
    UIImage * image = [UIImage imageWithCGImage:imageRef];
    
    if(view_controller_ != nil)
    {
        [view_controller_.imageView performSelectorOnMainThread:@selector(setImage:) withObject:image waitUntilDone:YES];
    }
}