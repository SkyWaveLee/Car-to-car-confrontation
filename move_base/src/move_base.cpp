/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 *         Mike Phillips (put the planner in its own thread)
 *********************************************************************/
#include <move_base/move_base.h>
#include <move_base_msgs/RecoveryStatus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace move_base
{
    MoveBase::MoveBase(tf2_ros::Buffer &tf) : tf_(tf),
                                              as_(NULL),
                                              planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
                                              bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
                                              blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
                                              recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
                                              planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
                                              runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false)
    {
        as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", [this](auto &goal) { executeCb(goal); }, false);

        ros::NodeHandle private_nh("~");
        ros::NodeHandle nh;

        recovery_trigger_ = PLANNING_R;

        // Get some parameters that will be global to the move base node
        std::string global_planner, local_planner;
        private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
        private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
        private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
        private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
        private_nh.param("planner_frequency", planner_frequency_, 0.0);
        private_nh.param("controller_frequency", controller_frequency_, 20.0);
        private_nh.param("planner_patience", planner_patience_, 5.0);
        private_nh.param("controller_patience", controller_patience_, 15.0);
        private_nh.param("max_planning_retries", max_planning_retries_, -1); // disabled by default

        private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
        private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

        // Set up plan triple buffer
        planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

        // Set up the planner's thread
        planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

        // For commanding the base
        vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0);

        ros::NodeHandle action_nh("move_base");
        action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
        recovery_status_pub_ = action_nh.advertise<move_base_msgs::RecoveryStatus>("recovery_status", 1);

        // We'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
        ros::NodeHandle simple_nh("move_base_simple");
        goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, [this](auto &goal) { goalCB(goal); });

        // We'll assume the radius of the robot to be consistent with what's specified for the costmaps
        private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
        private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
        private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
        private_nh.param("conservative_reset_dist", conservative_reset_dist_, 3.0);

        private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
        private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
        private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);

        // Create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
        planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
        planner_costmap_ros_->pause();

        // Initialize the global planner
        try
        {
            planner_ = bgp_loader_.createInstance(global_planner);
            planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
        }
        catch (const pluginlib::PluginlibException &ex)
        {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
            exit(1);
        }

        // Create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
        controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
        controller_costmap_ros_->pause();

        // Create a local planner
        try
        {
            tc_ = blp_loader_.createInstance(local_planner);
            ROS_INFO("Created local_planner %s", local_planner.c_str());
            tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
        }
        catch (const pluginlib::PluginlibException &ex)
        {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
            exit(1);
        }

        // Start actively updating costmaps based on sensor data
        planner_costmap_ros_->start();
        controller_costmap_ros_->start();

        // Advertise a service for getting a plan
        make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

        // Advertise a service for clearing the costmaps
        clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

        // If we shutdown our costmaps when we're deactivated... we'll do that now
        if (shutdown_costmaps_)
        {
            ROS_DEBUG_NAMED("move_base", "Stopping costmaps initially");
            planner_costmap_ros_->stop();
            controller_costmap_ros_->stop();
        }

        // Load any user specified recovery behaviors, and if that fails load the defaults
        if (!loadRecoveryBehaviors(private_nh))
        {
            loadDefaultRecoveryBehaviors();
        }

        // Initially, we'll need to make a plan
        state_ = PLANNING;

        // We'll start executing recovery behaviors at the beginning of our list
        recovery_index_ = 0;

        // We're all set up now so we can start the action server
        as_->start();

        dsrv_ = new dynamic_reconfigure::Server<move_base::MoveBaseConfig>(ros
