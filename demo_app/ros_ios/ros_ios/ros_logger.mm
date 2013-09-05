//
//  ros_planner.mm
//  ros_planner
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "ros_logger.h"
#import "LoggerViewController.h"

RosLogger::RosLogger()
{
    sub_ = n_.subscribe("/rosout", 1, &RosLogger::loggerCB, this);
    
    ros_thread_ = new boost::thread(&RosLogger::rosSpin, this);
    
    new_log_received_ = false;
}

RosLogger::~RosLogger()
{
    ros::shutdown();
    ros_thread_->join();
    delete ros_thread_;
}

void RosLogger::rosSpin()
{
    ros::spin();
}

void RosLogger::lockLogs()
{
    mtx_.lock();
}

void RosLogger::unlockLogs()
{
    mtx_.unlock();
}

bool RosLogger::newLogReceived()
{
    return new_log_received_;
}

void RosLogger::newLogReceived(bool v)
{
    new_log_received_ = v;
}

size_t RosLogger::getNumberOfLogs(LogLevel level)
{
    return logs[level].size();
}

int RosLogger::getLogStamp(LogLevel level, int index)
{
    return logs[level][index].header.stamp.sec;
}

const char * RosLogger::getLogName(LogLevel level, int index)
{
    return logs[level][index].name.c_str();
}

const char * RosLogger::getLogMsg(LogLevel level, int index)
{
    return logs[level][index].msg.c_str();
}

void RosLogger::clearLogs()
{
    new_log_received_ = false;
    for (LogLevel lvl = DEBUG; lvl < NB_LEVELS; lvl++)
    {
        logs[lvl].clear();
    }
}

void RosLogger::loggerCB(const rosgraph_msgs::LogConstPtr & msg)
{
    ROS_INFO("New Log Received");
    
    //unsigned int seconds = msg->header.stamp.sec%60;
    //unsigned int minutes = (msg->header.stamp.sec/60)%60;
    //unsigned int hours = (msg->header.stamp.sec/60/60)%24;
    
    if (msg->level == msg->DEBUG)
        logs[DEBUG].push_back(*msg);
    else if (msg->level == msg->INFO)
        logs[INFO].push_back(*msg);
    else if (msg->level == msg->WARN)
        logs[WARN].push_back(*msg);
    else if (msg->level == msg->ERROR)
        logs[ERROR].push_back(*msg);
    else if (msg->level == msg->FATAL)
        logs[FATAL].push_back(*msg);
    
    newLogReceived(true);
}