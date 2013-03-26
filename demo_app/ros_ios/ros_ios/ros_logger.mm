//
//  ros_planner.mm
//  ros_planner
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "ros_logger.h"
#import "LoggerViewController.h"

#import "Log.h"

RosLogger::RosLogger()
{
    sub_ = n_.subscribe("/rosout_agg", 5, &RosLogger::loggerCB, this);
    
    ros_thread_ = new boost::thread(&RosLogger::ros_spin, this);
}

RosLogger::~RosLogger()
{
    ros::shutdown();
    ros_thread_->join();
    delete ros_thread_;
}

void RosLogger::ros_spin()
{
    ros::spin();
}

void RosLogger::loggerCB(const rosgraph_msgs::LogConstPtr & msg)
{
    logs.push_back(*msg);
    
    Log * log = [[Log alloc] init];
    
    if (msg->level == msg->DEBUG)
        log.level = DEBUG;
    else if (msg->level == msg->INFO)
        log.level = INFO;
    else if (msg->level == msg->WARN)
        log.level = WARN;
    else if (msg->level == msg->ERROR)
        log.level = ERROR;
    else if (msg->level == msg->FATAL)
        log.level = FATAL;
    
    unsigned int seconds = msg->header.stamp.sec%60;
    unsigned int minutes = (msg->header.stamp.sec/60)%60;
    unsigned int hours = (msg->header.stamp.sec/60/60)%24;
    
    log.stamp = [NSString stringWithFormat:@"%d:%d:%d", hours, minutes, seconds];
    log.message = [NSString stringWithUTF8String:msg->msg.c_str()];
    log.node = [NSString stringWithUTF8String:msg->name.c_str()];
    
    if(view_controller_ != nil)
    {
        [view_controller_ performSelectorOnMainThread:@selector(newLogReceived:) withObject:log waitUntilDone:NO];
    }

}