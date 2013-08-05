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

RosPlanner::RosPlanner()
{
    pub_ = n_.advertise<geometry_msgs::PoseStamped>("/planner_goal", 1);
    sub_ = n_.subscribe("/map", 10, &RosPlanner::mapCB, this);
    c_srv_plan_ = n_.serviceClient<nav_msgs::GetPlan>("make_plan");
    
    ros_thread_ = new boost::thread(&RosPlanner::rosSpin, this);
    
    new_map = false;
}

RosPlanner::~RosPlanner()
{
    ros::shutdown();
    ros_thread_->join();
    delete ros_thread_;
}

void RosPlanner::rosSpin()
{
    ros::spin();
}

void RosPlanner::mapCB(const nav_msgs::OccupancyGridConstPtr & msg)
{
    ros::WallTime start = ros::WallTime::now();
    
    last_map_ = *msg;
    
    const unsigned int & width = last_map_.info.width;
    const unsigned int & height = last_map_.info.height;
    const std::vector<signed char> & map_data = last_map_.data;
    
    // Perform a first scan to find out bounds.
    size_t min_x = width;
    size_t min_y = height;
    size_t max_x = 0;
    size_t max_y = 0;
    
    for (size_t y = 0; y < height; ++y)
    {
        for (size_t x = 0; x < width; ++x)
        {
            const signed char c = map_data[y * width + x];
            if (c >= 0)
            {
                if (x < min_x)
                    min_x = x;
                if (x > max_x)
                    max_x = x;
                if (y < min_y)
                    min_y = y;
                if (y > max_y)
                    max_y = y;
            }
        }
    }
    
    im_width = (max_x - min_x);
    im_height = (max_y - min_y);
   
    //Produce a square
    if(im_width != im_height)
    {
        int delta;
        if(im_width > im_height)
        {
            delta = (im_width - im_height)/2;
            min_y -= delta;
            max_y += delta;
            im_height = im_width;
        }
        else
        {
            delta = (im_height - im_width)/2;
            min_x -= delta;
            max_x += delta;
            im_width = im_height;
        }
    }
    
    image_data.resize(im_width*im_width*4);
    unsigned char * ptr = image_data.data();
    
    for(size_t y = min_y; y != max_y; ++y)
    {
        for(size_t x = min_x; x != max_x; ++x)
        {
            const signed char c = map_data[y * width + x];
            
            if(c == 0)
            {
                ptr[0] = 255;
                ptr[1] = 255;
                ptr[2] = 255;
            }
            else if(c > 0)
            {
                ptr[0] = 255;
                ptr[1] = 0;
                ptr[2] = 0;
            }
            else
            {
                ptr[0] = 0;
                ptr[1] = 0;
                ptr[2] = 0;
            }
            
            ptr[4] = 255;
            ptr += 4;
        }
    }
    
    new_map = true;
    
    ROS_INFO("%ld %ld %ld",(unsigned long)im_width, (unsigned long)im_height, image_data.size());
}

size_t RosPlanner::new_map_available()
{
    return new_map;
}

size_t RosPlanner::get_map_width()
{
    return im_width;
}

size_t RosPlanner::get_map_height()
{
    return im_height;
}

unsigned char * RosPlanner::get_map_data()
{
    new_map = false;
    return &image_data[0];
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
