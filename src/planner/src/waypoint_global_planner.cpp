#include "waypoint_global_planner/waypoint_global_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <base_local_planner/footprint_helper.h>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/footprint.h>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(waypoint_global_planner::WaypointGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace waypoint_global_planner {

WaypointGlobalPlanner::WaypointGlobalPlanner() : 
    initialized_(false),
    allow_unknown_(true),
    costmap_ros_(nullptr),
    costmap_(nullptr),
    dsrv_(nullptr),
    max_vel_x_(0.4),
    max_vel_theta_(0.3),
    acc_lim_x_(0.2),
    acc_lim_theta_(0.3),
    inflation_radius_(0.3),
    obstacle_weight_(10.0),
    resolution_(0.05),
    waypoint_tolerance_(0.2),
    lookahead_distance_(1.0)
{
}

WaypointGlobalPlanner::WaypointGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) : 
    WaypointGlobalPlanner()
{
    initialize(name, costmap_ros);
}

WaypointGlobalPlanner::~WaypointGlobalPlanner()
{
    if(dsrv_)
        delete dsrv_;
}

void WaypointGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    if(initialized_) {
        ROS_WARN("Planner already initialized");
        return;
    }

    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    global_frame_ = costmap_ros_->getGlobalFrameID();
    world_model_ = new base_local_planner::CostmapModel(*costmap_);


    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle public_nh;

    // Initialize TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Setup dynamic reconfigure
    dsrv_ = new dynamic_reconfigure::Server<WaypointPlannerConfig>(private_nh);
    dynamic_reconfigure::Server<WaypointPlannerConfig>::CallbackType cb =
        boost::bind(&WaypointGlobalPlanner::reconfigureCallback, this, _1, _2);
    dsrv_->setCallback(cb);

    // Setup publishers and subscribers
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    marker_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("markers", 1);
    waypoint_sub_ = private_nh.subscribe("/clicked_point", 10, &WaypointGlobalPlanner::waypointCallback, this);
    laser_sub_ = public_nh.subscribe("/scan", 1, &WaypointGlobalPlanner::laserCallback, this);

    initialized_ = true;
    ROS_INFO("Waypoint Global Planner initialized successfully");
}

void WaypointGlobalPlanner::reconfigureCallback(WaypointPlannerConfig &config, uint32_t level) {
    max_vel_x_ = config.max_vel_x;
    max_vel_theta_ = config.max_vel_theta;
    acc_lim_x_ = config.acc_lim_x;
    acc_lim_theta_ = config.acc_lim_theta;
    inflation_radius_ = config.inflation_radius;
    obstacle_weight_ = config.obstacle_weight;
    resolution_ = config.resolution;
}

void WaypointGlobalPlanner::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    obstacle_points_.clear();
    
    // Convert laser scan to obstacle points in the base frame
    for(size_t i = 0; i < scan->ranges.size(); i++) {
        if(scan->ranges[i] < scan->range_max && scan->ranges[i] > scan->range_min) {
            double angle = scan->angle_min + i * scan->angle_increment;
            geometry_msgs::Point pt;
            pt.x = scan->ranges[i] * cos(angle);
            pt.y = scan->ranges[i] * sin(angle);
            obstacle_points_.push_back(pt);
        }
    }
}

void WaypointGlobalPlanner::waypointCallback(const geometry_msgs::PointStamped::ConstPtr& point)
{
    geometry_msgs::PoseStamped pose;
    pose.header = point->header;
    pose.pose.position = point->point;
    pose.pose.orientation.w = 1.0; // Default orientation

    // Calculate orientation if we have previous waypoints
    if(!waypoints_.empty()) {
        const auto& last = waypoints_.back();
        double yaw = atan2(point->point.y - last.pose.position.y, 
                          point->point.x - last.pose.position.x);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation = tf2::toMsg(q);
    }

    waypoints_.push_back(pose);
    visualizePlan(waypoints_);

    // Check if we should trigger planning
    if(waypoints_.size() >= 2) {
        const auto& last = waypoints_.back();
        const auto& prev = waypoints_[waypoints_.size()-2];
        double dist = hypot(last.pose.position.x - prev.pose.position.x,
                          last.pose.position.y - prev.pose.position.y);
        if(dist < waypoint_tolerance_) {
            ROS_INFO("Final waypoint reached threshold distance");
        }
    }
}

bool WaypointGlobalPlanner::checkTrajectory(const base_local_planner::Trajectory& traj) {
    if(!world_model_) {
        ROS_ERROR("World model not initialized");
        return false;
    }

    // Get robot footprint
    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    if(footprint.empty()) {
        ROS_WARN("Robot footprint empty, using default");
        footprint = costmap_2d::makeFootprintFromRadius(0.3); // Default circular footprint
    }

    for(unsigned int i = 0; i < traj.getPointsSize(); i++) {
        double x, y, th;
        traj.getPoint(i, x, y, th);
        
        // Check for collisions at each point
        double cost = world_model_->footprintCost(x, y, th, footprint);
        if(cost < 0) {  // Negative cost indicates collision
            return false;
        }
    }
    return true;
}

void WaypointGlobalPlanner::computeObstacleCosts(const geometry_msgs::PoseStamped& pose, double& cost)
{
    cost = 0;
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    
    for(const auto& obstacle : obstacle_points_) {
        double dx = pose.pose.position.x - obstacle.x;
        double dy = pose.pose.position.y - obstacle.y;
        double dist = sqrt(dx*dx + dy*dy);
        
        if(dist < inflation_radius_) {
            cost += obstacle_weight_ * (1.0 - dist/inflation_radius_);
        }
    }
}

void WaypointGlobalPlanner::optimizeTrajectory(base_local_planner::Trajectory& traj) {
    // Simple optimization - average with previous point
    for(unsigned int i = 1; i < traj.getPointsSize(); i++) {
        double x_curr, y_curr, th_curr;
        double x_prev, y_prev, th_prev;
        
        traj.getPoint(i, x_curr, y_curr, th_curr);
        traj.getPoint(i-1, x_prev, y_prev, th_prev);
        
        // Smooth the path
        x_curr = 0.8 * x_curr + 0.2 * x_prev;
        y_curr = 0.8 * y_curr + 0.2 * y_prev;
        
        // Update the point
        traj.setPoint(i, x_curr, y_curr, th_curr);
    }
}

bool WaypointGlobalPlanner::generateTrajectory(const geometry_msgs::PoseStamped& start,
                                             const geometry_msgs::PoseStamped& goal,
                                             base_local_planner::Trajectory& traj) {
    double dx = goal.pose.position.x - start.pose.position.x;
    double dy = goal.pose.position.y - start.pose.position.y;
    double dist = hypot(dx, dy);
    int steps = dist / resolution_;
    
    traj.resetPoints();
    for(int i = 0; i <= steps; i++) {
        double ratio = static_cast<double>(i) / steps;
        double x = start.pose.position.x + ratio * dx;
        double y = start.pose.position.y + ratio * dy;
        double yaw = atan2(dy, dx);
        traj.addPoint(x, y, yaw);
    }
    
    return true;
}

bool WaypointGlobalPlanner::findBestPath(const geometry_msgs::PoseStamped& start, 
                                       const geometry_msgs::PoseStamped& goal,
                                       std::vector<geometry_msgs::PoseStamped>& plan) {
    // Use NavfnROS as fallback
    navfn::NavfnROS fallback_planner;
    fallback_planner.initialize("fallback_planner", costmap_ros_);
    
    if(!fallback_planner.makePlan(start, goal, plan)) {
        ROS_WARN("Fallback planner failed to find path");
        return false;
    }
    
    // Generate and optimize multiple candidate trajectories
    base_local_planner::Trajectory best_traj;
    double best_cost = std::numeric_limits<double>::max();
    
    for(int i = 0; i < 5; i++) {
        base_local_planner::Trajectory traj;
        if(!generateTrajectory(start, goal, traj)) {
            continue;
        }
        
        optimizeTrajectory(traj);
        
        if(checkTrajectory(traj)) {
            if(traj.cost_ < best_cost) {
                best_cost = traj.cost_;
                best_traj = traj;
            }
        }
    }
    
    if(best_traj.getPointsSize() == 0) {
        ROS_WARN("No valid trajectory found, using fallback path");
        return true; // Use the fallback plan
    }
    
    // Convert trajectory to plan
    plan.clear();
    for(unsigned int i = 0; i < best_traj.getPointsSize(); i++) {
        double x, y, th;
        best_traj.getPoint(i, x, y, th);
        
        geometry_msgs::PoseStamped pose;
        pose.header = goal.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        tf2::Quaternion q;
        q.setRPY(0, 0, th);
        pose.pose.orientation = tf2::toMsg(q);
        plan.push_back(pose);
    }
    
    return true;
}

bool WaypointGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                   const geometry_msgs::PoseStamped& goal,
                                   std::vector<geometry_msgs::PoseStamped>& plan)
{
    if(!initialized_) {
        ROS_ERROR("Planner not initialized");
        return false;
    }
    
    // Clear previous plan
    plan.clear();
    
    // Transform to global frame
    geometry_msgs::PoseStamped global_start, global_goal;
    if(!transformPose(global_frame_, start, global_start) || 
       !transformPose(global_frame_, goal, global_goal)) {
        return false;
    }
    
    // If no waypoints are set, plan directly to goal
    if(waypoints_.empty()) {
        return findBestPath(global_start, global_goal, plan);
    }
    
    // Plan through all waypoints
    geometry_msgs::PoseStamped current_start = global_start;
    std::vector<geometry_msgs::PoseStamped> full_plan;
    
    for(size_t i = 0; i < waypoints_.size(); i++) {
        geometry_msgs::PoseStamped waypoint;
        if(!transformPose(global_frame_, waypoints_[i], waypoint)) {
            continue;
        }
        
        std::vector<geometry_msgs::PoseStamped> segment;
        if(!findBestPath(current_start, waypoint, segment)) {
            ROS_WARN("Failed to plan to waypoint %zu", i);
            continue;
        }
        
        full_plan.insert(full_plan.end(), segment.begin(), segment.end());
        current_start = waypoint;
    }
    
    // Plan from last waypoint to final goal
    std::vector<geometry_msgs::PoseStamped> final_segment;
    if(findBestPath(current_start, global_goal, final_segment)) {
        full_plan.insert(full_plan.end(), final_segment.begin(), final_segment.end());
    }
    
    if(full_plan.empty()) {
        ROS_WARN("Failed to create any plan");
        return false;
    }
    
    plan = full_plan;
    publishPlan(plan);
    return true;
}

bool WaypointGlobalPlanner::transformPose(const std::string& frame, 
                                        const geometry_msgs::PoseStamped& in_pose,
                                        geometry_msgs::PoseStamped& out_pose)
{
    if(in_pose.header.frame_id == frame) {
        out_pose = in_pose;
        return true;
    }

    try {
        if(!tf_buffer_->canTransform(frame, in_pose.header.frame_id, in_pose.header.stamp,
                                    ros::Duration(0.1))) {
            ROS_WARN("Cannot transform pose from %s to %s", 
                    in_pose.header.frame_id.c_str(), frame.c_str());
            return false;
        }
        tf_buffer_->transform(in_pose, out_pose, frame);
        return true;
    } catch (tf2::TransformException& ex) {
        ROS_WARN("Failed to transform pose: %s", ex.what());
        return false;
    }
}

void WaypointGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
    nav_msgs::Path gui_path;
    gui_path.header.frame_id = global_frame_;
    gui_path.header.stamp = ros::Time::now();
    gui_path.poses = path;
    plan_pub_.publish(gui_path);
}

void WaypointGlobalPlanner::visualizePlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;

    // Delete all markers first
    marker.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(marker);
    marker_pub_.publish(markers);
    markers.markers.clear();

    if(path.empty()) return;

    // Create waypoint markers
    marker.header.frame_id = global_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "waypoints";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
    marker.color.r = 1.0;
    marker.color.a = 1.0;

    for(size_t i = 0; i < path.size(); ++i) {
        marker.id = i;
        marker.pose = path[i].pose;
        markers.markers.push_back(marker);
    }

    // Create line strip for path
    marker.ns = "path";
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.id = 0;
    marker.points.clear();
    for(const auto& pose : path) {
        marker.points.push_back(pose.pose.position);
    }
    markers.markers.push_back(marker);

    marker_pub_.publish(markers);
}

} // namespace waypoint_global_planner
