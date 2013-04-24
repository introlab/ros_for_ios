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
#import <nav_msgs/OccupancyGrid.h>
#import <boost/thread/thread.hpp>
#import <vector>

@class MapViewController;

class RosPlanner
{
public:
    MapViewController __weak * view_controller_;
    
    RosPlanner();
    ~RosPlanner();
    void ros_spin();
    std::vector<CGPoint> getPlan(CGPoint goal);
    bool checkGoal(CGPoint goal);
    void sendGoal(CGPoint goal);
    
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::ServiceClient c_srv_plan_;
    boost::thread * ros_thread_;
    
    nav_msgs::OccupancyGrid map;
    
    void mapCB(const nav_msgs::OccupancyGridConstPtr & msg);
};

#endif
