#ifndef WAYPOINT_GLOBAL_PLANNER_H
#define WAYPOINT_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/LaserScan.h>
#include <base_local_planner/trajectory.h>
#include <navfn/navfn_ros.h>
#include <mutex>
#include <base_local_planner/costmap_model.h>
#include <costmap_2d/footprint.h>
#include "waypoint_global_planner/WaypointPlannerConfig.h"

namespace waypoint_global_planner {

class WaypointGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:
    WaypointGlobalPlanner();
    WaypointGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    ~WaypointGlobalPlanner();

    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start, 
                 const geometry_msgs::PoseStamped& goal,
                 std::vector<geometry_msgs::PoseStamped>& plan);

private:
    void reconfigureCallback(WaypointPlannerConfig &config, uint32_t level);
    void waypointCallback(const geometry_msgs::PointStamped::ConstPtr& point);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
    void visualizePlan(const std::vector<geometry_msgs::PoseStamped>& path);
    bool transformPose(const std::string& frame, 
                      const geometry_msgs::PoseStamped& in_pose,
                      geometry_msgs::PoseStamped& out_pose);
    bool checkTrajectory(const base_local_planner::Trajectory& traj);
    void computeObstacleCosts(const geometry_msgs::PoseStamped& pose, double& cost);
    bool findBestPath(const geometry_msgs::PoseStamped& start, 
                     const geometry_msgs::PoseStamped& goal,
                     std::vector<geometry_msgs::PoseStamped>& plan);
    void optimizeTrajectory(base_local_planner::Trajectory& traj);
    bool generateTrajectory(const geometry_msgs::PoseStamped& start,
                          const geometry_msgs::PoseStamped& goal,
                          base_local_planner::Trajectory& traj);

    // ROS components
    ros::Publisher plan_pub_;
    ros::Publisher marker_pub_;
    ros::Subscriber waypoint_sub_;
    ros::Subscriber laser_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    dynamic_reconfigure::Server<WaypointPlannerConfig>* dsrv_;

    // Configuration
    bool initialized_;
    bool allow_unknown_;
    std::string global_frame_;
    double max_vel_x_;
    double max_vel_theta_;
    double acc_lim_x_;
    double acc_lim_theta_;
    double inflation_radius_;
    double obstacle_weight_;
    double resolution_;
    double waypoint_tolerance_;
    double lookahead_distance_;

    // Data storage
    std::vector<geometry_msgs::PoseStamped> waypoints_;
    std::vector<geometry_msgs::Point> obstacle_points_;
    std::mutex obstacle_mutex_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    base_local_planner::WorldModel* world_model_;
};

} // namespace waypoint_global_planner

#endif
