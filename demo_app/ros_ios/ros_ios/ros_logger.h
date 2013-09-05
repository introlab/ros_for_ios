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
#import <boost/thread/mutex.hpp>
#import <boost/array.hpp>
#import <vector>

#import "Log.h"

@class LoggerViewController;

class RosLogger
{
public:
    LoggerViewController __weak * view_controller_;
    boost::array<std::vector<rosgraph_msgs::Log>,NB_LEVELS> logs;
    
    RosLogger();
    ~RosLogger();
    void rosSpin();
    
    void lockLogs();
    void unlockLogs();
    bool newLogReceived();
    void newLogReceived(bool v);
    size_t getNumberOfLogs(LogLevel level);
    int getLogStamp(LogLevel level, int index);
    const char * getLogName(LogLevel level, int index);
    const char * getLogMsg(LogLevel level, int index);
    void clearLogs();
    
private:
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    boost::thread * ros_thread_;
    boost::mutex mtx_;
    
    bool new_log_received_;

    void loggerCB(const rosgraph_msgs::LogConstPtr & msg);
};

#endif
