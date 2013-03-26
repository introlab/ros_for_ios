//
//  ros_planner.mm
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "ros_planner.h"
#import <ros/time.h>
#import "MapViewController.h"
#include <nav_msgs/GetPlan.h>
#include "CheckGoalOnMap.h"
#include "ConvertGoalOnMap.h"

#include <cmath>

RosPlanner::RosPlanner()
{    
    pub_ = n_.advertise<geometry_msgs::PoseStamped>(/*"/planner_goal"*/"/move_base_simple/goal", 1);
    sub_ = n_.subscribe("/map_to_image/map_image", 10, &RosPlanner::mapCB, this);

    c_srv_plan_ = n_.serviceClient<nav_msgs::GetPlan>("make_plan");
    c_srv_goal_ = n_.serviceClient<image_tools::CheckGoalOnMap>("check_goal_on_map");
    c_srv_convert_ = n_.serviceClient<image_tools::ConvertGoalOnMap>("convert_goal_on_map");
    
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

void RosPlanner::mapCB(const sensor_msgs::ImageConstPtr & msg)
{
    unsigned int width = msg->width;
    unsigned int height = msg->height;
    unsigned char * data = (unsigned char *) msg->data.data();
    
    
    CGDataProviderRef provider = CGDataProviderCreateWithData(NULL,
                                                              data,
                                                              msg->data.size(),
                                                              NULL);
    
    int bitsPerComponent = 8;
    int bitsPerPixel = 24;
    int bytesPerRow = 3 * width;
    
    CGColorSpaceRef colorSpaceRef = CGColorSpaceCreateDeviceRGB();
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
    
    UIImage * image = [UIImage imageWithCGImage:imageRef];
    
    [view_controller_.mapView performSelectorOnMainThread:@selector(setImage:) withObject:image waitUntilDone:NO];
    
    //[view_controller_.mapView performSelectorOnMainThread:@selector(sizeToFit:) withObject:nil waitUntilDone:NO];
    
    //[view_controller_.mapView performSelectorOnMainThread:@selector(setNeedsDisplay:) withObject:nil waitUntilDone:NO];
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
    image_tools::CheckGoalOnMap srv;
    srv.request.x = (int)round(goal.x);
    srv.request.y = (int)round(goal.y);
             
    if (c_srv_goal_.call(srv))
    {
        return srv.response.valid != 0;
    }
    else
    {
        ROS_ERROR("Failed to call service check_goal_on_map");
        return false;
    }
}

void RosPlanner::sendGoal(CGPoint goal)
{
    geometry_msgs::PoseStamped pose;
    
    //Target position
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "/map";
    pose.header.seq = 0;
    
    image_tools::ConvertGoalOnMap srv;
    srv.request.x = (int)round(goal.x);
    srv.request.y = (int)round(goal.y);
    
    if (c_srv_convert_.call(srv))
    {
        pose.pose = srv.response.goal;
        pub_.publish(pose);
    }
    else
    {
        ROS_ERROR("Failed to call service convert_goal_on_map");
    }
}
