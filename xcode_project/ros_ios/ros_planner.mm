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
    
    timer_ = n_.createTimer(ros::Duration(0.01), &RosPlanner::timerCB, this);
    robot_frame_ = "base_footprint";
    
    last_map_.header.stamp = ros::Time(0);
    new_map_available_ = false;
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

void RosPlanner::lockMap()
{
    mtx_.lock();
}

void RosPlanner::unlockMap()
{
    mtx_.unlock();
}

void RosPlanner::timerCB(const ros::TimerEvent&)
{
    // A zeroed time in last_map_ indicates we didn't receive a valid map yet.
    // Only warn when an image is requested.
    if (last_map_.header.stamp.isZero())
        return;
    
    tf::StampedTransform t;
    try {
        tf_.lookupTransform(
                            last_map_.header.frame_id,
                            robot_frame_,
                            ros::Time(0),   // Get the latest available.
                            t);
    } catch (tf::TransformException e) {
        ROS_WARN_THROTTLE(
                          1.0,
                          "Cannot get transform, the robot position will be wrong. "
                          "Reason: %s.",
                          e.what());
        return;
    }
    
    last_robot_pos_ = t.getOrigin();
    last_robot_angle_ = t.getRotation();
}

void RosPlanner::mapCB(const nav_msgs::OccupancyGridConstPtr & msg)
{
    ROS_INFO("Map received");
    
    lockMap();
    last_map_ = *msg;
    new_map_available_ = true;
    unlockMap();
}

bool RosPlanner::newMapAvailable()
{
    return new_map_available_;
}

size_t RosPlanner::getMapWidth()
{
    return last_map_.info.width;
}

size_t RosPlanner::getMapHeight()
{
    return last_map_.info.height;
}

signed char * RosPlanner::getMap()
{
    new_map_available_ = false;
    return &(last_map_.data[0]);
}

float RosPlanner::getMapResolution()
{
    return last_map_.info.resolution;
}

float RosPlanner::getMapOriginX()
{
    return last_map_.info.origin.position.x;
}

float RosPlanner::getMapOriginY()
{
    return last_map_.info.origin.position.y;
}

GLKVector3 RosPlanner::getRobotPose()
{
    return GLKVector3Make(last_robot_pos_.x(),
                          last_robot_pos_.y(),
                          last_robot_pos_.z());
}

GLKQuaternion RosPlanner::getRobotAngle()
{
    return GLKQuaternionMake(last_robot_angle_.x(),
                             last_robot_angle_.y(),
                             last_robot_angle_.z(),
                             last_robot_angle_.w());
}

GLKMatrix4 RosPlanner::getRobotTransform()
{
    GLKMatrix4 translation = GLKMatrix4MakeTranslation(last_robot_pos_.x(),
                                                       last_robot_pos_.y(),
                                                       last_robot_pos_.z());
    
    GLKQuaternion quaternion = getRobotAngle();
    
    GLKMatrix4 rotation = GLKMatrix4MakeWithQuaternion(quaternion);
    
    return GLKMatrix4Multiply(translation, rotation);
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

bool RosPlanner::sendGoal(CGPoint goal)
{
//    if (!last_map_.header.stamp.isZero())
//    {
//        if(last_map_.data[goal.y * last_map_.info.width + goal.x] == 0)
//        {
//            geometry_msgs::PoseStamped real_pose;
//            
//            //Target position
//            real_pose.header.stamp = ros::Time::now();
//            real_pose.header.frame_id = "/map";
//            real_pose.header.seq = 0;
//            
//            const float& resolution = last_map_.info.resolution;
//            
//            tf::Pose pose;
//            tf::Point map_org;
//            tf::pointMsgToTF(last_map_.info.origin.position, map_org);
//            
//            double rx = (min_x + goal.x) * resolution + map_org.x();
//            double ry = (min_y + goal.y) * resolution + map_org.y();
//            
//            ROS_INFO("%lf,%lf", rx, ry);
//            
//            pose.setOrigin(tf::Vector3(rx, ry, 0.0));
//            
//            pose.setRotation(tf::createQuaternionFromYaw(
//                                                         atan2(ry-last_robot_pos_.y(),rx-last_robot_pos_.x())));
//            
//            tf::poseTFToMsg(pose,real_pose.pose);
//            pub_.publish(real_pose);
//            return true;
//        }
//        else
//        {
//            ROS_WARN("Wrong goal.");
//            return false;
//        }
//    }
//    else
//    {
//        ROS_WARN("Map not yet ready.");
        return false;
//    }
}
