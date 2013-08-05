//
//  ros_joy.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#ifndef ros_joy_h
#define ros_joy_h

#import <ros/node_handle.h>
#import <ros/publisher.h>
#import <boost/thread/thread.hpp>

class RosJoy
{
public:
    RosJoy();
    ~RosJoy();
    void rosSpin();
    void sendCmds(double lin_x, double lin_y, double ang_z);
    
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    boost::thread * ros_thread_;
};

#endif
