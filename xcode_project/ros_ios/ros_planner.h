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
#import <tf/transform_listener.h>
#import <geometry_msgs/PoseStamped.h>
#import <nav_msgs/OccupancyGrid.h>
#import <boost/thread/thread.hpp>
#import <boost/thread/mutex.hpp>
#import <vector>
#import <string>
#import <GLKit/GLKit.h>

@class MapViewController;

class RosPlanner
{
public:
    MapViewController __weak * view_controller_;
    
    RosPlanner();
    ~RosPlanner();
    void rosSpin();
    
    void lockMap();
    void unlockMap();
    bool newMapAvailable();
    size_t getMapWidth();
    size_t getMapHeight();
    signed char * getMap();
    float getMapResolution();
    float getMapOriginX();
    float getMapOriginY();
    GLKVector3 getRobotPose();
    GLKQuaternion getRobotAngle();
    GLKMatrix4 getRobotTransform();
    
    std::vector<CGPoint> getPlan(CGPoint goal);
    bool sendGoal(CGPoint goal);
    
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::ServiceClient c_srv_plan_;
    boost::thread * ros_thread_;
    boost::mutex mtx_;
    
    nav_msgs::OccupancyGrid last_map_;
    bool new_map_available_;
    tf::Point last_robot_pos_;
    tf::Quaternion last_robot_angle_;
    
    ros::Timer timer_;
    tf::TransformListener tf_;
    std::string robot_frame_;
    
    void timerCB(const ros::TimerEvent&);
    void mapCB(const nav_msgs::OccupancyGridConstPtr & msg);
};

#endif
