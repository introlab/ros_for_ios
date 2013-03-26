//
//  ros_planner.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#ifndef ros_planner_h
#define ros_planner_h

#import <ros/node_handle.h>
#import <ros/publisher.h>
#import <ros/subscriber.h>
#import <ros/service_client.h>
#import <geometry_msgs/PoseStamped.h>
#import <sensor_msgs/Image.h>
#import <boost/thread/thread.hpp>
#import <vector>

@class MapViewController;

class RosPlanner
{
public:
    MapViewController * view_controller_;
    
    RosPlanner();
    ~RosPlanner();
    void ros_spin();
    std::vector<CGPoint> getPlan(CGPoint goal);
    UIImage * getMapImage();
    bool checkGoal(CGPoint goal);
    void sendGoal(CGPoint goal);
    
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::ServiceClient c_srv_plan_;
    ros::ServiceClient c_srv_goal_;
    ros::ServiceClient c_srv_convert_;
    boost::thread * ros_thread_;
    
    void mapCB(const sensor_msgs::ImageConstPtr & msg);
};

#endif
