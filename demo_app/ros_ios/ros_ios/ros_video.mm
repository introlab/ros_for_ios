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
    ros_thread_ = new boost::thread(&RosVideo::ros_spin, this);
    it = new image_transport::ImageTransport(n_);
    this->subscribe_to("RGB");
}

RosVideo::~RosVideo()
{
    ros::shutdown();
    ros_thread_->join();
    delete ros_thread_;
    delete it;
}

void RosVideo::ros_spin()
{
    ros::spin();
}

void RosVideo::subscribe_to(std::string name)
{
    image_transport::TransportHints hints("x264", ros::TransportHints());
    it_sub_.shutdown();
    
    if(name.compare("RGB") == 0)
    {
        it_sub_ = it->subscribe("/openni/rgb/image_rect_color", 1, &RosVideo::imageCB, this, hints);
    }
    else if(name.compare("Depth") == 0)
    {
        it_sub_ = it->subscribe("/openni/depth/image_raw", 1, &RosVideo::imageCB, this, hints);
    }
    else if(name.compare("IR") == 0)
    {
        it_sub_ = it->subscribe("/openni/ir/image_raw", 1, &RosVideo::imageCB, this, hints);
    }
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