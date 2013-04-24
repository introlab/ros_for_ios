//
//  ros_planner.mm
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "ros_planner.h"
#import "MapViewController.h"
#import <ros/time.h>
#import <nav_msgs/GetPlan.h>

#include <cmath>

RosPlanner::RosPlanner()
{
    pub_ = n_.advertise<geometry_msgs::PoseStamped>("/planner_goal", 1);
    sub_ = n_.subscribe("/map", 10, &RosPlanner::mapCB, this);
    c_srv_plan_ = n_.serviceClient<nav_msgs::GetPlan>("make_plan");
    
    ros_thread_ = new boost::thread(&RosPlanner::ros_spin, this);
}

RosPlanner::~RosPlanner()
{
    ros::shutdown();
    ros_thread_->join();
    delete ros_thread_;
}

void RosPlanner::ros_spin()
{
    ros::spin();
}

void RosPlanner::mapCB(const nav_msgs::OccupancyGridConstPtr & msg)
{
    map = *msg;
}

std::vector<CGPoint> RosPlanner::getPlan(CGPoint goal)
{
    nav_msgs::GetPlan srv;
    std::vector<CGPoint> plan;
    
    geometry_msgs::PoseStamped start_pose;
    geometry_msgs::PoseStamped goal_pose;
    
    //Start position
    //It's an empty Position (move take the robot position if the start pose is empty)
    start_pose.header.stamp = ros::Time::now();
    start_pose.header.frame_id = "";
    start_pose.header.seq = 0;
    
    //Goal position
    goal_pose.header.stamp = ros::Time::now();
    goal_pose.header.frame_id = "/map";
    goal_pose.header.seq = 0;
    goal_pose.pose.position.x = goal.x;
    goal_pose.pose.position.y = goal.y;
    goal_pose.pose.position.z = 0;
    goal_pose.pose.orientation.w = 1;
    goal_pose.pose.orientation.x = 0;
    goal_pose.pose.orientation.y = 0;
    goal_pose.pose.orientation.z = 0;
    
    srv.request.start = start_pose;
    srv.request.goal = goal_pose;
    
    if (c_srv_plan_.call(srv))
    {
        std::vector<geometry_msgs::PoseStamped> tmp = srv.response.plan.poses;
        
        for(size_t i = 0; i < tmp.size(); i++)
        {
            CGPoint point;
            point.x = tmp[i].pose.position.x;
            point.y = tmp[i].pose.position.y;
            plan.push_back(point);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service make_plan");
    }
    
    return plan;
}

bool RosPlanner::checkGoal(CGPoint goal)
{
    return true;
}

void RosPlanner::sendGoal(CGPoint goal)
{
    geometry_msgs::PoseStamped pose;
    
    //Target position
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "/map";
    pose.header.seq = 0;
    
}
