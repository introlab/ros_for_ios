//
//  ros_logger.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#ifndef ros_logger_h
#define ros_logger_h

#undef DEBUG
#import <ros/node_handle.h>
#import <ros/subscriber.h>
#import <ros/service_client.h>
#import <rosgraph_msgs/Log.h>
#import <boost/thread/thread.hpp>

#include <vector>

@class LoggerViewController;

class RosLogger
{
public:
    LoggerViewController __weak * view_controller_;
    std::vector<rosgraph_msgs::Log> logs;
    
    RosLogger();
    ~RosLogger();
    void ros_spin();
    
private:
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    
    boost::thread * ros_thread_;
    
    void loggerCB(const rosgraph_msgs::LogConstPtr & msg);
};

#endif
