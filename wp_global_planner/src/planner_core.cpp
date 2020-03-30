/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <global_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <global_planner/dijkstra.h>
#include <global_planner/astar.h>
#include <global_planner/grid_path.h>
#include <global_planner/gradient_path.h>
#include <global_planner/quadratic_calculator.h>

#include <stdlib.h>
#include <algorithm>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(wp_global_planner::WPGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace wp_global_planner {

void WPGlobalPlanner::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value) {
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

WPGlobalPlanner::WPGlobalPlanner() :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
}

WPGlobalPlanner::WPGlobalPlanner(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) :
        costmap_(NULL), initialized_(false), allow_unknown_(true) {
    //initialize the planner
    initialize(name, costmap, frame_id);
}

WPGlobalPlanner::~WPGlobalPlanner() {
    if (p_calc_)
        delete p_calc_;
    if (planner_)
        delete planner_;
    if (path_maker_)
        delete path_maker_;
    if (dsrv_)
        delete dsrv_;
}

void WPGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void WPGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id) {
    if (!initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        costmap_ = costmap;
        frame_id_ = frame_id;

        unsigned int cx = costmap->getSizeInCellsX(), cy = costmap->getSizeInCellsY();

        private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);
        if(!old_navfn_behavior_)
            convert_offset_ = 0.5;
        else
            convert_offset_ = 0.0;

        bool use_quadratic;
        private_nh.param("use_quadratic", use_quadratic, true);
        if (use_quadratic)
            p_calc_ = new QuadraticCalculator(cx, cy);
        else
            p_calc_ = new PotentialCalculator(cx, cy);

        bool use_dijkstra;
        private_nh.param("use_dijkstra", use_dijkstra, true);
        if (use_dijkstra)
        {
            DijkstraExpansion* de = new DijkstraExpansion(p_calc_, cx, cy);
            if(!old_navfn_behavior_)
                de->setPreciseStart(true);
            planner_ = de;
        }
        else
            planner_ = new AStarExpansion(p_calc_, cx, cy);

        bool use_grid_path;
        private_nh.param("use_grid_path", use_grid_path, false);
        if (use_grid_path)
            path_maker_ = new GridPath(p_calc_);
        else
            path_maker_ = new GradientPath(p_calc_);

        orientation_filter_ = new OrientationFilter();

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);
        waypoint_sub_ = private_nh.subscribe("/clicked_point", 100, &WPGlobalPlanner::waypointCallback, this);
        path_sub_ = private_nh.subscribe("/path", 100, &WPGlobalPlanner::pathCallback, this);

        private_nh.param("allow_unknown", allow_unknown_, true);
        planner_->setHasUnknown(allow_unknown_);
        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        private_nh.param("default_tolerance", default_tolerance_, 0.0);
        private_nh.param("publish_scale", publish_scale_, 100);

        private_nh.param("fragmentation_size", FRAGMENTATION_SIZE_, 0.25);
        if(FRAGMENTATION_SIZE_ < 0.05){
            ROS_WARN("Parameter fragmentation_size is too little!");
            FRAGMENTATION_SIZE_ = 0.25;
        }
        else if(FRAGMENTATION_SIZE_ > 1.00){
            ROS_WARN("Parameter fragmentation_size is too big!");
            FRAGMENTATION_SIZE_ = 0.25;
        }

        double POINT_RADIUS_;
        private_nh.param("point_radius", POINT_RADIUS_, 1.41);
        if(POINT_RADIUS_ < 0.05){
            ROS_WARN("Parameter point_radius is too little!");
            POINT_RADIUS_ = 1.41;
        }
        else if(POINT_RADIUS_ > 5.00){
            ROS_WARN("Parameter point_radius is too big!");
            POINT_RADIUS_ = 1.41;
        }
        POINT_RADIUS_SQUARE_ = POINT_RADIUS_ * POINT_RADIUS_;

        make_plan_srv_ = private_nh.advertiseService("make_plan", &WPGlobalPlanner::makePlanService, this);

        dsrv_ = new dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>(ros::NodeHandle("~/" + name));
        dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>::CallbackType cb = boost::bind(
                &WPGlobalPlanner::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);

        initialized_ = true;
    } else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
}

void WPGlobalPlanner::reconfigureCB(global_planner::GlobalPlannerConfig& config, uint32_t level) {
    planner_->setLethalCost(config.lethal_cost);
    path_maker_->setLethalCost(config.lethal_cost);
    planner_->setNeutralCost(config.neutral_cost);
    planner_->setFactor(config.cost_factor);
    publish_potential_ = config.publish_potential;
    orientation_filter_->setMode(config.orientation_mode);
    orientation_filter_->setWindowSize(config.orientation_window_size);
}

void WPGlobalPlanner::clearRobotCell(const geometry_msgs::PoseStamped& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool WPGlobalPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = frame_id_;

    return true;
}

void WPGlobalPlanner::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + (mx+convert_offset_) * costmap_->getResolution();
    wy = costmap_->getOriginY() + (my+convert_offset_) * costmap_->getResolution();
}

bool WPGlobalPlanner::worldToMap(double wx, double wy, double& mx, double& my) {
    double origin_x = costmap_->getOriginX(), origin_y = costmap_->getOriginY();
    double resolution = costmap_->getResolution();

    if (wx < origin_x || wy < origin_y)
        return false;

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY())
        return true;

    return false;
}

bool WPGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan) {
    return makePlan(start, goal, default_tolerance_, plan);
}

bool WPGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                      const geometry_msgs::PoseStamped& goal,
                      double tolerance,
                      std::vector<geometry_msgs::PoseStamped>& plan) {
    boost::mutex::scoped_lock lock(mutex_);
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized yet, but it is being \
                   used, please call initialize() before use");
        return false;
    }

    plan.clear();
    if(true == is_path_should_be_updated_){
        createPointPath(start, goal);
        is_path_should_be_updated_ = false;
    }
    deletePassedWaypoints(start);
    if(wp_path_.empty())
        makeDefaultPlan(start, goal, tolerance, plan);
    else
        makeWaypointPlan(start, goal, plan);

    publishPlan(plan);
    return !plan.empty();
}

void WPGlobalPlanner::makeWaypointPlan(const geometry_msgs::PoseStamped& start, 
                      const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan) const{
    plan.push_back(start);
    for(auto iter = wp_path_.begin(); iter != wp_path_.end(); iter++)
        plan.push_back(*iter);
    plan.push_back(goal);
}

void WPGlobalPlanner::makeDefaultPlan(const geometry_msgs::PoseStamped& start,
                      const geometry_msgs::PoseStamped& goal,
                      double tolerance,
                      std::vector<geometry_msgs::PoseStamped>& plan) {
    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if (goal.header.frame_id != global_frame) {
        ROS_ERROR(
                "The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), goal.header.frame_id.c_str());
        return;
    }

    if (start.header.frame_id != global_frame) {
        ROS_ERROR(
                "The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", global_frame.c_str(), start.header.frame_id.c_str());
        return;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    unsigned int start_x_i, start_y_i, goal_x_i, goal_y_i;
    double start_x, start_y, goal_x, goal_y;

    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i)) {
        ROS_WARN(
                "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return;
    }
    if(old_navfn_behavior_){
        start_x = start_x_i;
        start_y = start_y_i;
    }else{
        worldToMap(wx, wy, start_x, start_y);
    }

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i)) {
        ROS_WARN_THROTTLE(1.0,
                "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
        return;
    }
    if(old_navfn_behavior_){
        goal_x = goal_x_i;
        goal_y = goal_y_i;
    }else{
        worldToMap(wx, wy, goal_x, goal_y);
    }

    //clear the starting cell within the costmap because we know it can't be an obstacle
    clearRobotCell(start, start_x_i, start_y_i);

    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();

    //make sure to resize the underlying array that Navfn uses
    p_calc_->setSize(nx, ny);
    planner_->setSize(nx, ny);
    path_maker_->setSize(nx, ny);
    potential_array_ = new float[nx * ny];

    outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

    bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
                                                    nx * ny * 2, potential_array_);
    if(!old_navfn_behavior_)
        planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);
    if(publish_potential_)
        publishPotential(potential_array_);

    if (found_legal) {
        //extract the plan
        if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, goal, plan)) {
            //make sure the goal we push on has the same timestamp as the rest of the plan
            geometry_msgs::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = ros::Time::now();
            plan.push_back(goal_copy);
        } else {
            ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
        }
    }else{
        ROS_ERROR("Failed to get a plan.");
    }

    // add orientations if needed
    orientation_filter_->processPath(start, plan);

    delete potential_array_;
}

void WPGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = frame_id_;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

bool WPGlobalPlanner::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                      const geometry_msgs::PoseStamped& goal,
                                       std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    std::string global_frame = frame_id_;

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;

    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
        ROS_ERROR("NO PATH!");
        return false;
    }

    ros::Time plan_time = ros::Time::now();
    for (int i = path.size() -1; i>=0; i--) {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
        mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }
    if(old_navfn_behavior_){
            plan.push_back(goal);
    }
    return !plan.empty();
}

void WPGlobalPlanner::publishPotential(float* potential)
{
    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
    double resolution = costmap_->getResolution();
    nav_msgs::OccupancyGrid grid;
    // Publish Whole Grid
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx, wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++) {
        float potential = potential_array_[i];
        if (potential < POT_HIGH) {
            if (potential > max) {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++) {
        if (potential_array_[i] >= POT_HIGH) {
            grid.data[i] = -1;
        } else
            grid.data[i] = potential_array_[i] * publish_scale_ / max;
    }
    potential_pub_.publish(grid);
}

//
void WPGlobalPlanner::deletePassedWaypoints(const geometry_msgs::PoseStamped& start){
    while(!wp_path_.empty()){
        double Px = start.pose.position.x;
        double Py = start.pose.position.y;
        double Ax = wp_path_.begin()->pose.position.x;
        double Ay = wp_path_.begin()->pose.position.y;
        double PAsquare = (Px - Ax) * (Px - Ax) + (Py - Ay) * (Py - Ay);

        if(PAsquare < POINT_RADIUS_SQUARE_){
            wp_path_.pop_front();
            ROS_DEBUG("Waypoint was removed: start is [%f, %f], deleted is [%f, %f]",
                     Px, Py, Ax, Ay);
        }
        else{
            break;
        }
    }
}

//
void WPGlobalPlanner::waypointCallback(const geometry_msgs::PointStamped::ConstPtr& waypoint){
    ROS_DEBUG("Point detected!");
    is_path_should_be_updated_ = true;
    if(waypoints_.empty()){
        waypoints_.push_back(geometry_msgs::PoseStamped());
    }
    waypoints_.back().header = waypoint->header;
    waypoints_.back().pose.position = waypoint->point;
}

//
void WPGlobalPlanner::createPointPath(const geometry_msgs::PoseStamped& start,
                                      const geometry_msgs::PoseStamped& goal){
    wp_path_.clear();

    if(!waypoints_.empty()){
        wp_path_.push_back(geometry_msgs::PoseStamped());
        wp_path_.back().header = start.header;
        wp_path_.back().pose.position = start.pose.position;
        fragmentWaypoints(*wp_path_.begin(), *waypoints_.begin());
    }

    for(auto iter = waypoints_.begin(); iter != waypoints_.end(); iter++){
        wp_path_.push_back(geometry_msgs::PoseStamped());
        wp_path_.back().header = iter->header;
        wp_path_.back().pose.position = iter->pose.position;
        if(std::next(iter, 1) != waypoints_.end()){
            fragmentWaypoints(*iter, *std::next(iter, 1));
        }
        else{
            fragmentWaypoints(waypoints_.back(), goal);
        }
    }
}

//
void WPGlobalPlanner::fragmentWaypoints(const geometry_msgs::PoseStamped& current_wp,
                                        const geometry_msgs::PoseStamped& next_wp){
    const size_t MAX_NUMBER_OF_POINT_BETWEEN_WP = 1000;
    double lengthX = next_wp.pose.position.x - current_wp.pose.position.x;
    double lengthY = next_wp.pose.position.y - current_wp.pose.position.y;
    uint16_t parts = 1 + std::max(abs(lengthX), abs(lengthY)) / FRAGMENTATION_SIZE_;
    double dx = lengthX/parts;
    double dy = lengthY/parts;
    double newX, newY;
    if(parts > MAX_NUMBER_OF_POINT_BETWEEN_WP){
        ROS_WARN("The distance between waypoints is too big! \
                  There are greater then 1000 points between them.");
    }
    while(parts > 1){
        parts--;
        newX = wp_path_.back().pose.position.x + dx;
        newY = wp_path_.back().pose.position.y + dy;
        wp_path_.push_back(geometry_msgs::PoseStamped());
        wp_path_.back().header = current_wp.header;
        wp_path_.back().pose.position.x = newX;
        wp_path_.back().pose.position.y = newY;
    }
}

//
void WPGlobalPlanner::pathCallback(const nav_msgs::Path::ConstPtr& path){
    const size_t MAX_NUMBER_OF_WP = 100;
    ROS_DEBUG("Path detected!");
    is_path_should_be_updated_ = true;
    waypoints_.clear();
    if(path->poses.size() > MAX_NUMBER_OF_WP){
        ROS_WARN("Number of waypoints have been given from path topic is too big!");
    }
    for(auto iter = path->poses.begin(); iter != path->poses.end(); iter++){
        waypoints_.push_back(geometry_msgs::PoseStamped());
        waypoints_.back().header = iter->header;
        waypoints_.back().pose.position = iter->pose.position;
        ROS_DEBUG("wp: %f / %f", waypoints_.back().pose.position.x, waypoints_.back().pose.position.y);
    }
}

} //end namespace wp_global_planner
