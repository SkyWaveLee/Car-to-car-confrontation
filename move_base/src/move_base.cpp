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
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

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

    // as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    recovery_trigger_ = PLANNING_R;

    // get some parameters that will be global to the move base node
    std::string local_planner;
    //    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    private_nh.param("move_base/saveMapping", saveMapping, false);
    private_nh.param("savePath", savePath, std::string("/home/duzhong/dzacs/src/dzglobalplanner/mapping.txt"));
    private_nh.param("base_local_planner", local_planner, std::string("dwa_local_planner/DWAPlannerROS"));
    //    private_nh.param("base_local_planner", local_planner, std::string("teb_local_planner/TebLocalPlannerROS"));
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
    private_nh.param("planner_frequency", planner_frequency_, 1.0);
    private_nh.param("controller_frequency", controller_frequency_, 3.0);
    private_nh.param("planner_patience", planner_patience_, 5.0);
    private_nh.param("controller_patience", controller_patience_, 15.0);
    private_nh.param("max_planning_retries", max_planning_retries_, -1); // disabled by default
    printf("savePath----move_base-->-=%s\n\n\n\n", savePath.c_str());
    private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
    private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

    // set up plan triple buffer
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    // set up the planner's thread
    planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

    // for commanding the base
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0);
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    all_plan_pub_ = private_nh.advertise<nav_msgs::Path>("allplan", 1);

    ros::NodeHandle action_nh("move_base");
    action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);

    // we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
    // they won't get any useful information back about its status, but this is useful for tools
    // like nav_view and rviz
    ros::NodeHandle simple_nh("move_base_simple");
    goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

    // we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    private_nh.param("local_costmap/inscribed_radius", inscribed_radius_, 0.325);
    private_nh.param("local_costmap/circumscribed_radius", circumscribed_radius_, 0.46);
    private_nh.param("clearing_radius", clearing_radius_, circumscribed_radius_);
    private_nh.param("conservative_reset_dist", conservative_reset_dist_, 0.21);

    private_nh.param("shutdown_costmaps", shutdown_costmaps_, false);
    private_nh.param("clearing_rotation_allowed", clearing_rotation_allowed_, true);
    private_nh.param("recovery_behavior_enabled", recovery_behavior_enabled_, true);
    private_nh.param("road_paths_savePath", road_paths_savePath, std::string("/home/duzhong/dzacs/src/resource/road_paths"));
    private_nh.param("stop_points_savePath", stop_points_savePath, std::string("/home/duzhong/dzacs/src/resource/stop_points"));

    if (saveMapping)
    {
      printf("move_base--> saveMapping = true\n");
    }
    else if (!saveMapping)
    {
      printf("move_base--> saveMapping = false\n");
    }
    // create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    // initialize the global planner
    /* try
    {
      planner_ = bgp_loader_.createInstance(global_planner);
      planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    }
    catch (const pluginlib::PluginlibException &ex)
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    } */

    // create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    // create a local planner
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

    // advertise a service for getting a plan
    make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveBase::planService, this);

    // advertise a service for clearing the costmaps
    clear_costmaps_srv_ = private_nh.advertiseService("clear_costmaps", &MoveBase::clearCostmapsService, this);

    // if we shutdown our costmaps when we're deactivated... we'll do that now
    if (shutdown_costmaps_)
    {
      ROS_DEBUG_NAMED("move_base", "Stopping costmaps initially");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }

    // load any user specified recovery behaviors, and if that fails load the defaults
    if (!loadRecoveryBehaviors(private_nh))
    {
      loadDefaultRecoveryBehaviors();
    }

    // initially, we'll need to make a plan
    state_ = PLANNING;

    // we'll start executing recovery behaviors at the beginning of our list
    recovery_index_ = 0;

    foundObstacleFlag = 0;

    pursuitSpeed = 0.6;
    pubtargetSpeed = 0.6;
    recvScanFlag = 0;

    saveMappingMsg.data = 0;
    road_points_index = 0;
    stop_point_signal_msg.data = 0;

    sub_lasersScan = nh.subscribe("scan", 1, &MoveBase::callback_lasersScan, this);
    sub_saveMapping = nh.subscribe("savemapping", 1, &MoveBase::callback_saveMapping, this);

    hgglobalplannerpub = private_nh.advertise<move_base_msgs::hgpathplanner>("hgglobalplanner", 10);
    hglocationpub = private_nh.advertise<move_base_msgs::hglocation>("hglocation", 10);
    angle_pub = nh.advertise<geometry_msgs::Twist>("pursuitAngle", 1);

    vis_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    stop_point_signal = private_nh.advertise<std_msgs::UInt8>("stop_signal",1);


    std::vector<std::string> mapFolders = {
        road_paths_savePath,
        stop_points_savePath};
    loadAllMaps(mapFolders);

    roadPoints = road_paths[road_points_index];
    stopPoints = stop_Points[0];
    doplanner();
  }

  void MoveBase::loadAllMaps(const std::vector<std::string> &mapFolders)
  {
    loadMapsFromFolder(mapFolders[0], road_paths);
    loadMapsFromFolder(mapFolders[1], stop_Points);
  }
  void MoveBase::loadMapsFromFolder(const std::string &mapname, std::vector<std::vector<geometry_msgs::PoseStamped>> &paths)
  {
    std::vector<std::string> files;
    std::vector<int> nameNum;
    getAllFiles(mapname, files, nameNum);
    std::cout << "sss" << (int)files.size() << "mapname= " << mapname << std::endl;
    std::vector<geometry_msgs::PoseStamped> path;
#if 0
    // printf("----------------(int)files.size() %d\n\n",(int)files.size());
    for(int i;i<(int)files.size();i++){
        // printf("files[%d]=%s,nameNum[%d]%d\n",i,files[i].c_str(),i,nameNum[i]);
    }
#endif
    for (int i = 0; i < files.size(); i++)
    {
      std::string routemap = files[i].c_str();
      loadMap(routemap, path);
      paths.push_back(path);
    }
  }

  void MoveBase::loadMap(std::string routelinemap, std::vector<geometry_msgs::PoseStamped> &path)
  {
    path.clear();
    std::ifstream fin(routelinemap.c_str());
    std::string line, temp;
    geometry_msgs::PoseStamped rp;

    while (getline(fin, line))
    {
      temp = "";
      int strnum = 0;
      for (unsigned int i = 0; i < line.size(); ++i)
      {
        if (line[i] == ',')
        {
          std::stringstream stream;
          stream << temp;
          switch (strnum)
          {
          case 0:
            stream >> rp.pose.position.x;
            stream.str("");
            temp = "";
            break;
          case 1:
            stream >> rp.pose.position.y;
            stream.str("");
            temp = "";
            break;
          case 2:
            stream >> rp.pose.position.z;
            stream.str("");
            temp = "";
            break;
          case 3:
            stream >> rp.pose.orientation.x;
            stream.str("");
            temp = "";
            break;
          case 4:
            stream >> rp.pose.orientation.y;
            stream.str("");
            temp = "";
            break;
          case 5:
            stream >> rp.pose.orientation.z;
            stream.str("");
            temp = "";
            break;
          case 6:
            stream >> rp.pose.orientation.w;
            stream.str("");
            temp = "";
            break;
          default:
            stream.str("");
            temp = "";
            break;
          }
          strnum++;
        }
        else
        {
          temp += line[i];
        }
      }
      path.push_back(rp);
    }

    // printf("waypoints_.size() =%d\n\n",(int)waypoints_.size());
    fin.close();
  }
  void MoveBase::getAllFiles(std::string path, std::vector<std::string> &files, std::vector<int> &nameNum)
  {
    int tempnamenum;
    std::string tempname;
    std::vector<int> tempnameNums;
    if (path[path.length() - 1] != '/')
      path = path + "/";
    DIR *dir;
    struct dirent *ptr;
    char base[1000];
    if ((dir = opendir(path.c_str())) == NULL)
    {
      perror("Open dir error...");
      std::cout << "Check: " << path << std::endl;
      exit(1);
    }

    while ((ptr = readdir(dir)) != NULL)
    {
      if (ptr->d_type == 8) // 文件
      {
        std::string name = ptr->d_name;
        int length = name.length();
        if (name.substr(length - 3, length - 1) == "txt" || name.substr(length - 3, length - 1) == "TXT")
        {

          tempname.clear();
          for (int j = 0; j < name.size() - 4; j++)
          {
            tempname += name[j];
          }
          tempnamenum = atoi(tempname.c_str());
          tempnameNums.push_back(tempnamenum);
        }
      }
    }
    closedir(dir);

    int compareNum;
    int indexI;
    while (tempnameNums.size() != 0)
    {
      compareNum = 99999;
      for (int i = 0; i < (int)tempnameNums.size(); i++)
      {
        if (compareNum > tempnameNums[i])
        {
          compareNum = tempnameNums[i];
          indexI = i;
        }
      }
      nameNum.push_back(compareNum);
      tempnameNums.erase(tempnameNums.begin() + indexI);
    }

    for (int i = 0; i < (int)nameNum.size(); i++)
    {
      tempname = std::to_string(nameNum[i]);
      files.push_back(path + tempname + ".txt");
    }

    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr current_pcl_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  void MoveBase::callback_lasersScan(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    currentScan = *msg;
    //       laserprojector.
    scancloud.header = msg->header;
    //      tf2_ros::Buffer* tf_;
    // project the scan into a point cloud
    try
    {
      laserprojector.transformLaserScanToPointCloud(msg->header.frame_id, *msg, scancloud, tf_);
    }
    catch (tf::TransformException &ex)
    {
      ROS_WARN("High fidelity enabled, but TF returned a transform exception to frame %s: %s", global_frame_.c_str(),
               ex.what());
      laserprojector.projectLaser(*msg, scancloud);
    }
    pcl::fromROSMsg(scancloud, *current_pcl_ptr);
    recvScanFlag = 1;
  }

  void MoveBase::callback_saveMapping(const std_msgs::UInt8::ConstPtr &msg)
  {
    saveMappingMsg = *msg;
    if (saveMappingMsg.data == 1)
    {
      saveMapping = true;
    }
    else if (saveMappingMsg.data == 2)
    {
      saveMapping = false;
    }
    if (saveMapping)
    {
      printf("move_base callback_saveMapping--> saveMapping = true\n");
    }
    else if (!saveMapping)
    {
      printf("move_base callback_saveMapping--> saveMapping = false\n");
    }
  }

  void MoveBase::pub_stopP_marks()
  {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "stopPoints";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.position.x = rp.pose.position.x;
    // marker.pose.position.y = rp.pose.position.y;
    // marker.pose.position.z = rp.pose.position.z;
    // marker.pose.orientation.x = rp.pose.orientation.x;
    // marker.pose.orientation.y = rp.pose.orientation.y;
    // marker.pose.orientation.z = rp.pose.orientation.z;
    // marker.pose.orientation.w = rp.pose.orientation.w;
    marker.scale.x = 1.5;
    marker.scale.y = 1.5;
    marker.scale.z = 1.5;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 254;
    marker.color.g = 0;
    marker.color.b = 0.0;

    markers.markers.clear();
    for (size_t i = 0; i < (int)stopPoints.size(); i++)
    {
      marker.id = i;
      marker.pose.position.x = stopPoints[i].pose.position.x;
      marker.pose.position.y = stopPoints[i].pose.position.y;
      marker.pose.position.z = stopPoints[i].pose.position.z;
      marker.pose.orientation.x = stopPoints[i].pose.orientation.x;
      marker.pose.orientation.y = stopPoints[i].pose.orientation.y;
      marker.pose.orientation.z = stopPoints[i].pose.orientation.z;
      marker.pose.orientation.w = stopPoints[i].pose.orientation.w;
      markers.markers.push_back(marker);
    }
    markers.markers.push_back(marker);
    // printf("\nmarkers.markers.size()=%d\n",(int)markers.markers.size());
    vis_pub.publish(markers);
  }

  double getYaw(geometry_msgs::PoseStamped pose)
  {
    return tf2::getYaw(pose.pose.orientation);
  }

  double getDis(geometry_msgs::PoseStamped currentpose, geometry_msgs::PoseStamped roadpose)
  {
    return sqrt((currentpose.pose.position.x - roadpose.pose.position.x) * (currentpose.pose.position.x - roadpose.pose.position.x) + (currentpose.pose.position.y - roadpose.pose.position.y) * (currentpose.pose.position.y - roadpose.pose.position.y));
  }

  int MoveBase::detectReachedGoal(const geometry_msgs::PoseStamped &currentPose, geometry_msgs::PoseStamped &stopPose)
  {
    double diffDistance = getDis(currentPose, stopPose);
    if (diffDistance < STOP_DIS)
    {
      double currentpose_yaw = tf2::getYaw(currentPose.pose.orientation);
      double targetpose_yaw = tf2::getYaw(stopPose.pose.orientation);
      double diffangle = fabs(currentpose_yaw - targetpose_yaw);
      // printf("diffangle = %.2f,diffDistance = %.2f\n",diffangle,diffDistance);
      if (diffangle > 3.1415926)
        diffangle = 6.283 - diffangle;
      if (diffangle > 1.0)
      {
        return 0;
      }
      else
        return 1;
    }
    else
      return 0;
  }

  float MoveBase::foundTargetAngle(float targetDis, float speed, geometry_msgs::PoseStamped current_position, std::vector<geometry_msgs::PoseStamped> temp_plan, float *pubSpeed)
  {
    float minDis = 8888, temp_dis = 9999;
    float targetX = 0.0, targetY = 0.0;
    float resultSpeed = 0.5;
    geometry_msgs::PoseStamped targetPose;
    for (int i = 0; i < (int)temp_plan.size(); i++)
    {
      temp_dis = getDis(current_position, temp_plan[i]);
      temp_dis = abs(temp_dis - targetDis);
      if (temp_dis < minDis)
      {
        minDis = temp_dis;
        targetPose = temp_plan[i];
      }
    }
    std::string base_frame = robot_base_frame_;
    geometry_msgs::PoseStamped baselink_target_pose;

    // just get the latest available transform... for accuracy they should send
    // goals in the frame of the planner
    targetPose.header.stamp = ros::Time();

    try
    {
      tf_.transform(targetPose, baselink_target_pose, base_frame);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
               baselink_target_pose.header.frame_id.c_str(), base_frame.c_str(), ex.what());
    }
    double currentpose_yaw = tf2::getYaw(current_position.pose.orientation);
    double targetpose_yaw = tf2::getYaw(baselink_target_pose.pose.orientation);
    double diffangle = fabs(currentpose_yaw - targetpose_yaw);
    // 	printf("%d = %.2f,%.2f\n",i,roadPoint_yaw,diffangle);
    if (diffangle > 3.1415926)
      diffangle = 6.283 - diffangle;
    if (diffangle > 0.1)
    {
      resultSpeed = speed / (1 + 0.15 * (diffangle - 0.1));
    }

    *pubSpeed = resultSpeed;
    float alpha = lineDirection(baselink_target_pose.pose.position.x, baselink_target_pose.pose.position.y);
    float targetAngle = atan(2.0 * 0.16 * sin(alpha) / targetDis) * 57.3;
    //         printf("targetDis = %.2f,x = %.2f;y = %.2f,alpha = %.2f,diffangle = %.2f,pubSpeed = %.2f,angle = %.2f\n",targetDis,baselink_target_pose.pose.position.x, baselink_target_pose.pose.position.y,alpha*57.3,diffangle,resultSpeed,targetAngle);
    return targetAngle;
  }
  float MoveBase::foundNearestPoint(float aimDis, geometry_msgs::PoseStamped current_position, std::vector<geometry_msgs::PoseStamped> temp_plan)
  {
    float minDis = 8888;
    float temp_dis = 9999;
    for (int i = 0; i < (int)temp_plan.size(); i++)
    {
      temp_dis = getDis(current_position, temp_plan[i]);
      if (temp_dis < minDis)
      {
        minDis = temp_dis;
      }
    }
    return minDis;
  }

  float MoveBase::lineDirection(float xsecond, float ysecond)
  {
    float alpha = 0.0;
    if ((xsecond > 0) && (ysecond > 0))
    {
      alpha = atan(fabs(ysecond - 0) / fabs(xsecond - 0));
    }
    else if ((xsecond > 0) && (ysecond <= 0))
    {
      //       alpha=2*M_PI-atan(fabs(ysecond-0)/fabs(xsecond-0));
      alpha = -atan(fabs(ysecond - 0) / fabs(xsecond - 0));
    }

    return alpha;
  }

  float MoveBase::sLineControl(geometry_msgs::PoseStamped current_position, std::vector<geometry_msgs::PoseStamped> temp_plan, float *pubSpeed)
  {
    float tempspeed = pubtargetSpeed;
    float NearestDis = foundNearestPoint(450, current_position, temp_plan);
    if (NearestDis > 0.03)
    {                                                                // if > 3cm
      tempspeed = pubtargetSpeed / (1.0 + 10.0 * (NearestDis - 0.03)); // if NearestDis = 0.04m, 100*NearestDis = 4
    }
    float targetDis = 0.5 * tempspeed + 0.2;
    float targetAngle = foundTargetAngle(targetDis, tempspeed, current_position, temp_plan, pubSpeed);
    return targetAngle;
  }

  float MoveBase::obstacleDetect(geometry_msgs::PoseStamped current_position, std::vector<geometry_msgs::PoseStamped> temp_plan, float *distance)
  {
    geometry_msgs::PoseStamped targetPose;
    std::string base_frame = robot_base_frame_;
    geometry_msgs::PoseStamped baselink_local_planner_pose;

    double minObstacleDis = 8888;
    int planSize = (int)temp_plan.size();
    if (planSize > 20)
    {
      for (int i = 0; i < (int)temp_plan.size(); i++)
      {
        targetPose = temp_plan[i];
        targetPose.header.stamp = ros::Time();
        try
        {
          tf_.transform(targetPose, baselink_local_planner_pose, base_frame);
        }
        catch (tf2::TransformException &ex)
        {
          ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
                   baselink_local_planner_pose.header.frame_id.c_str(), base_frame.c_str(), ex.what());
        }
        double minDis = 8888, temp_dis = 9999, obstacleDis = 9999;
        int minindex = -1;
        for (int j = 0; j < (int)current_pcl_ptr->points.size(); j++)
        {
          double px = -current_pcl_ptr->points[j].x, py = current_pcl_ptr->points[j].y, pz = current_pcl_ptr->points[j].z;
          if (px > 0.10 && px < 1.0 && py > -0.105 && py < 0.105)
          {
            temp_dis = sqrt((px - baselink_local_planner_pose.pose.position.x) * (px - baselink_local_planner_pose.pose.position.x) + (py - baselink_local_planner_pose.pose.position.y) * (py - baselink_local_planner_pose.pose.position.y));
            if (temp_dis < minDis)
            {
              minDis = temp_dis;
              minindex = j;
            }
            //       if(temp_dis < 0.15){
            //           minindex = j;
            //       }
          }
        }
        if (minindex != -1 && minDis < 0.15)
        {
          obstacleDis = sqrt((current_pcl_ptr->points[minindex].x) * (current_pcl_ptr->points[minindex].x) + (current_pcl_ptr->points[minindex].y) * (current_pcl_ptr->points[minindex].y));
        }
        if (obstacleDis < minObstacleDis)
        {
          minObstacleDis = obstacleDis;
        }
      }

      if (minObstacleDis > 0.10 && minObstacleDis < 0.4)
      {
        foundObstacleFlag = 1;
      }

      // printf("move_base obstacleDetect-->minObstacleDis = %.2f,foundObstacleFlag is %d\n",minObstacleDis,foundObstacleFlag);
    }
  }
  void MoveBase::doplanner()
  {
    ros::NodeHandle n;
    ros::Rate rate(20);
    geometry_msgs::PoseStamped goal;
    std::vector<geometry_msgs::PoseStamped> global_plan;
    geometry_msgs::PoseStamped global_pose;
    geometry_msgs::PoseStamped current_position;
    geometry_msgs::Twist cmd_vel;
    state_ = CONTROLLING;
    int traffic_flag = 0;
    int SP_deleted_flag = 0;
    while (n.ok())
    {
      // 	recovery_behaviors_[0]->runBehavior();
      ros::spinOnce();
      getRobotPose(global_pose, planner_costmap_ros_);
      current_position = global_pose;
      // 	saveMapping = false;

      if (saveMapping)
      {
        printf("start save mapping\n");
        if (distance(current_position, last_pose) >= 0.02)
        {
          fp = fopen(savePath.c_str(), "a");
          fprintf(fp, "%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,\n",
                  current_position.pose.position.x,
                  current_position.pose.position.y,
                  current_position.pose.position.z,
                  current_position.pose.orientation.x,
                  current_position.pose.orientation.y,
                  current_position.pose.orientation.z,
                  current_position.pose.orientation.w);
          fclose(fp);
          last_pose = current_position;
        }
      }
      else
      {
#if 1
        currentlocation.x = current_position.pose.position.x;
        currentlocation.y = current_position.pose.position.y;
        double currentpose_yaw = tf2::getYaw(current_position.pose.orientation);
        currentlocation.heading = currentpose_yaw;
        hglocationpub.publish(currentlocation);
        global_plan.clear();
        if (!makePlan(current_position, goal, global_plan) || global_plan.empty())
        {
          ROS_DEBUG_NAMED("move_base", "Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
          continue;
        }
        else
        {
          if (state_ == CONTROLLING)
          {

            std::vector<geometry_msgs::PoseStamped> *temp_plan = &global_plan;

            foundObstacleFlag = 0;
            float distance = 0.8;
            //障碍物检测，当检测到障碍物，foundObstacleFlag = 1;
            obstacleDetect(current_position, global_plan, &distance);


            if (road_points_index > stopPoints.size() || road_points_index > road_paths.size())
            {
              ROS_ERROR("move_base-->tc_->stopPoints or  road_paths number error.");
              publishZeroVelocity();
              continue;
            }
            /***********定点停车**********/
            // 到达停车点，停固定时间
            if (detectReachedGoal(current_position, stopPoints[road_points_index]))
            {
              publishZeroVelocity();
              printf("move_base-->reached stoppoint!!! %d\n",road_points_index);
              if (!(road_points_index == stopPoints.size() - 1)) {
                printf("move_base-->stop_point_signal_msg.data =1; \n");
                stop_point_signal_msg.data =1;
                stop_point_signal.publish(stop_point_signal_msg);              
              }
              ros::Duration(10).sleep();
              SP_deleted_flag = 1;
              road_points_index++;
              roadPoints = road_paths[road_points_index];
              new_plan = true;
              stop_point_signal_msg.data =0;
              stop_point_signal.publish(stop_point_signal_msg);
              continue;
            }

            /***********定点停车  end **********/
            //----------------start for pure pursuit----------------
            //此处令foundObstacleFlag = 2，则始终使用纯追踪算法，
            // 若想遇到障碍物时使用避障算法，屏蔽此行
            foundObstacleFlag = 2;


            //当 foundObstacleFlag ！= 1;启用纯追踪算法
            if (foundObstacleFlag != 1)
            {
              // printf("foundObstacleFlag == %d \n",foundObstacleFlag);
              float tagetAngle = sLineControl(current_position, global_plan, &pursuitSpeed);
              cmd_vel.linear.x = pursuitSpeed / 2;
              cmd_vel.angular.z = tagetAngle;
              if (foundObstacleFlag == 1)
              {
                cmd_vel.linear.x = 0.0;
                //  cmd_vel.angular.z = 0;
              }
              //发布线速度和角度指令
              angle_pub.publish(cmd_vel);
              rate.sleep();
              continue;
            }
            //----------------end for pure pursuit----------------
            //当 foundObstacleFlag = 1;启用避障算法
            else
            {
              latest_plan_ = temp_plan;
              last_valid_plan_ = ros::Time::now();
              planning_retries_ = 0;
              new_global_plan_ = true;
              if (!tc_->setPlan(*latest_plan_))
              {
                // ABORT and SHUTDOWN COSTMAPS
                ROS_ERROR("move_base-->tc_->setPlan error.");
                publishZeroVelocity();
                ros::Duration(0.2).sleep();
                publishZeroVelocity();
                ros::Duration(0.2).sleep();
                publishZeroVelocity();
                ros::Duration(0.2).sleep();
                publishZeroVelocity();
                ros::Duration(0.2).sleep();
                continue;
              }
              if (tc_->computeVelocityCommands(cmd_vel))
              {
                ROS_DEBUG_NAMED("move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                                cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
                last_valid_control_ = ros::Time::now();
                //发布线速度和角速度指令
                vel_pub_.publish(cmd_vel);
                if (recovery_trigger_ == CONTROLLING_R)
                  recovery_index_ = 0;
              }
              else
              {
                printf("The local planner could not find a valid plan.\n");
                ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

                // check if we've tried to find a valid control for longer than our time limit
                if (ros::Time::now() > attempt_end)
                {
                  // we'll move into our obstacle clearing mode
                  publishZeroVelocity();
                  printf("zero vel because timeout2\n");
                  recovery_trigger_ = CONTROLLING_R;
                }
              }
            }
          }
        }
#endif
      }
      rate.sleep();
      if (rate.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds",
                 controller_frequency_, rate.cycleTime().toSec());
    }
  }

  void MoveBase::reconfigureCB(move_base::MoveBaseConfig &config, uint32_t level)
  {
    boost::recursive_mutex::scoped_lock l(configuration_mutex_);

    // The first time we're called, we just want to make sure we have the
    // original configuration
    if (!setup_)
    {
      last_config_ = config;
      default_config_ = config;
      setup_ = true;
      return;
    }

    if (config.restore_defaults)
    {
      config = default_config_;
      // if someone sets restore defaults on the parameter server, prevent looping
      config.restore_defaults = false;
    }

    if (planner_frequency_ != config.planner_frequency)
    {
      planner_frequency_ = config.planner_frequency;
      p_freq_change_ = true;
    }

    if (controller_frequency_ != config.controller_frequency)
    {
      controller_frequency_ = config.controller_frequency;
      c_freq_change_ = true;
    }

    planner_patience_ = config.planner_patience;
    controller_patience_ = config.controller_patience;
    max_planning_retries_ = config.max_planning_retries;
    conservative_reset_dist_ = config.conservative_reset_dist;

    recovery_behavior_enabled_ = config.recovery_behavior_enabled;
    clearing_rotation_allowed_ = config.clearing_rotation_allowed;
    shutdown_costmaps_ = config.shutdown_costmaps;

    oscillation_timeout_ = config.oscillation_timeout;
    oscillation_distance_ = config.oscillation_distance;
    if (config.base_global_planner != last_config_.base_global_planner)
    {
      boost::shared_ptr<nav_core::BaseGlobalPlanner> old_planner = planner_;
      // initialize the global planner
      ROS_INFO("Loading global planner %s", config.base_global_planner.c_str());
      try
      {
        planner_ = bgp_loader_.createInstance(config.base_global_planner);

        // wait for the current planner to finish planning
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);

        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        planner_->initialize(bgp_loader_.getName(config.base_global_planner), planner_costmap_ros_);

        lock.unlock();
      }
      catch (const pluginlib::PluginlibException &ex)
      {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s",
                  config.base_global_planner.c_str(), ex.what());
        planner_ = old_planner;
        config.base_global_planner = last_config_.base_global_planner;
      }
    }

    if (config.base_local_planner != last_config_.base_local_planner)
    {
      boost::shared_ptr<nav_core::BaseLocalPlanner> old_planner = tc_;
      // create a local planner
      try
      {
        tc_ = blp_loader_.createInstance(config.base_local_planner);
        // Clean up before initializing the new planner
        planner_plan_->clear();
        latest_plan_->clear();
        controller_plan_->clear();
        resetState();
        tc_->initialize(blp_loader_.getName(config.base_local_planner), &tf_, controller_costmap_ros_);
      }
      catch (const pluginlib::PluginlibException &ex)
      {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the \
                   containing library is built? Exception: %s",
                  config.base_local_planner.c_str(), ex.what());
        tc_ = old_planner;
        config.base_local_planner = last_config_.base_local_planner;
      }
    }

    last_config_ = config;
  }

  void MoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr &goal)
  {
    ROS_DEBUG_NAMED("move_base", "In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    move_base_msgs::MoveBaseActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    action_goal_pub_.publish(action_goal);
  }

  void MoveBase::clearCostmapWindows(double size_x, double size_y)
  {
    geometry_msgs::PoseStamped global_pose;

    // clear the planner's costmap
    getRobotPose(global_pose, planner_costmap_ros_);

    std::vector<geometry_msgs::Point> clear_poly;
    double x = global_pose.pose.position.x;
    double y = global_pose.pose.position.y;
    geometry_msgs::Point pt;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

    // clear the controller's costmap
    getRobotPose(global_pose, controller_costmap_ros_);

    clear_poly.clear();
    x = global_pose.pose.position.x;
    y = global_pose.pose.position.y;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
  }

  bool MoveBase::clearCostmapsService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp)
  {
    // clear the costmaps
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_controller(*(controller_costmap_ros_->getCostmap()->getMutex()));
    controller_costmap_ros_->resetLayers();

    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock_planner(*(planner_costmap_ros_->getCostmap()->getMutex()));
    planner_costmap_ros_->resetLayers();
    return true;
  }

  bool MoveBase::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp)
  {
    if (as_->isActive())
    {
      ROS_ERROR("move_base must be in an inactive state to make a plan for an external user");
      return false;
    }
    // make sure we have a costmap for our planner
    if (planner_costmap_ros_ == NULL)
    {
      ROS_ERROR("move_base cannot make a plan for you because it doesn't have a costmap");
      return false;
    }

    geometry_msgs::PoseStamped start;
    // if the user does not specify a start pose, identified by an empty frame id, then use the robot's pose
    if (req.start.header.frame_id.empty())
    {
      geometry_msgs::PoseStamped global_pose;
      if (!getRobotPose(global_pose, planner_costmap_ros_))
      {
        ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
        return false;
      }
      start = global_pose;
    }
    else
    {
      start = req.start;
    }

    // update the copy of the costmap the planner uses
    clearCostmapWindows(2 * clearing_radius_, 2 * clearing_radius_);

    // first try to make a plan to the exact desired goal
    std::vector<geometry_msgs::PoseStamped> global_plan;
    if (!planner_->makePlan(start, req.goal, global_plan) || global_plan.empty())
    {
      ROS_DEBUG_NAMED("move_base", "Failed to find a plan to exact goal of (%.2f, %.2f), searching for a feasible goal within tolerance",
                      req.goal.pose.position.x, req.goal.pose.position.y);

      // search outwards for a feasible goal within the specified tolerance
      geometry_msgs::PoseStamped p;
      p = req.goal;
      bool found_legal = false;
      float resolution = planner_costmap_ros_->getCostmap()->getResolution();
      float search_increment = resolution * 3.0;
      if (req.tolerance > 0.0 && req.tolerance < search_increment)
        search_increment = req.tolerance;
      for (float max_offset = search_increment; max_offset <= req.tolerance && !found_legal; max_offset += search_increment)
      {
        for (float y_offset = 0; y_offset <= max_offset && !found_legal; y_offset += search_increment)
        {
          for (float x_offset = 0; x_offset <= max_offset && !found_legal; x_offset += search_increment)
          {

            // don't search again inside the current outer layer
            if (x_offset < max_offset - 1e-9 && y_offset < max_offset - 1e-9)
              continue;

            // search to both sides of the desired goal
            for (float y_mult = -1.0; y_mult <= 1.0 + 1e-9 && !found_legal; y_mult += 2.0)
            {

              // if one of the offsets is 0, -1*0 is still 0 (so get rid of one of the two)
              if (y_offset < 1e-9 && y_mult < -1.0 + 1e-9)
                continue;

              for (float x_mult = -1.0; x_mult <= 1.0 + 1e-9 && !found_legal; x_mult += 2.0)
              {
                if (x_offset < 1e-9 && x_mult < -1.0 + 1e-9)
                  continue;

                p.pose.position.y = req.goal.pose.position.y + y_offset * y_mult;
                p.pose.position.x = req.goal.pose.position.x + x_offset * x_mult;

                if (planner_->makePlan(start, p, global_plan))
                {
                  if (!global_plan.empty())
                  {

                    // adding the (unreachable) original goal to the end of the global plan, in case the local planner can get you there
                    //(the reachable goal should have been added by the global planner)
                    global_plan.push_back(req.goal);

                    found_legal = true;
                    ROS_DEBUG_NAMED("move_base", "Found a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                    break;
                  }
                }
                else
                {
                  ROS_DEBUG_NAMED("move_base", "Failed to find a plan to point (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
                }
              }
            }
          }
        }
      }
    }

    // copy the plan into a message to send out
    resp.plan.poses.resize(global_plan.size());
    for (unsigned int i = 0; i < global_plan.size(); ++i)
    {
      resp.plan.poses[i] = global_plan[i];
    }

    return true;
  }

  MoveBase::~MoveBase()
  {
    recovery_behaviors_.clear();

    delete dsrv_;

    if (as_ != NULL)
      delete as_;

    if (planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if (controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;

    planner_thread_->interrupt();
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;

    planner_.reset();
    tc_.reset();
  }
  bool MoveBase::makePlan(const geometry_msgs::PoseStamped &start,
                          const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
  {
    // printf("---------------\n");
    // printf("minIndex 1-= %d,%d ,",minIndex,LastIndex);
    if (new_plan)
    {
      LastIndex = 0;
      minIndex = 0;
      new_plan = false;
    }
    // printf("2-= %d,%d ,",minIndex,LastIndex);

    int plan_front = 150;
    int plan_bear = 2;
    double Min_cartoplan_dis = 3;
    double Max_dis_Toplan = 0.5;
    double current_yaw = 0, roadPoint_yaw = 0;

    // printf("Got a start: %.2f, %.2f\n", start.pose.position.x, start.pose.position.y);
    plan.clear();
    std::vector<geometry_msgs::PoseStamped> visualplan;
    visualplan.clear();
    double point_heading = 0.0;
    minDis = 10000;
    double diffangle = 0.0;
    int j_num = 0;
    int endpoint_flag = 0;
    // printf("LastIndex = %d,roadPoints.size = %d\n",LastIndex,(int)roadPoints.size());
    pub_Num++;
    if (pub_Num >= 10)
    {
      pub_Num = 0;
      nav_msgs::Path gui_path;
      gui_path.poses.resize((int)roadPoints.size());

      gui_path.header.frame_id = "map";
      gui_path.header.stamp = ros::Time::now();

      // Extract the plan in world co-ordinates, we assume the path is all in the same frame
      for (unsigned int i = 0; i < (int)roadPoints.size(); i++)
      {
        gui_path.poses[i] = roadPoints[i];
      }
      all_plan_pub_.publish(gui_path);
    }
    if (minIndex > (int)roadPoints.size())
      minIndex = 0;

    if (abs((int)roadPoints.size() - LastIndex) < 5)
    {
      LastIndex = 0;
      minIndex = 0;
      endpoint_flag = 1;
      // printf("success1--abs((int)roadPoints.size() - LastIndex) < 5\n");
    }
    else
      endpoint_flag = 0;
    //   maxIndex = (int)roadPoints.size();
    // printf("3 -= %d,%d ,",minIndex,LastIndex);
    for (int j = minIndex; j < (int)roadPoints.size(); j++)
    {
      double diffDis = getDis(start, roadPoints[j]);
      if (diffDis < minDis)
      {
        minDis = diffDis;
        LastIndex = j;
        // printf("success-- 2");
      }

      j_num++;
      if ((j_num % 10) == 0 && minDis < Max_dis_Toplan)
      {
        // printf("success-- j_num =%d ,",j_num);
        break;
      }
    }
    // printf("4-= %d,%d ,",minIndex,LastIndex);
    if (minDis > Min_cartoplan_dis)
    {
      LastIndex = 0;
      Last_lastIndex = -999;
    }
    if (fabs(LastIndex - Last_lastIndex) > 50 && endpoint_flag == 0)
    {
      {
        double tem_mindis = Min_cartoplan_dis;
        double diffDis;
        int flag_findminpoint = 0;
        for (int k = 0; k < (int)roadPoints.size(); k++)
        {
          diffDis = getDis(start, roadPoints[k]);
          if (diffDis < tem_mindis)
          {
            current_yaw = getYaw(start),
            roadPoint_yaw = getYaw(roadPoints[k]);
            diffangle = fabs(current_yaw - roadPoint_yaw);

            if (diffangle > 3.1415926)
              diffangle = 6.283 - diffangle;
            if (diffangle < 2.57)
            {
              tem_mindis = diffDis;
              LastIndex = k;
              flag_findminpoint = 1;
            }
          }
        }
        if (flag_findminpoint == 0)
        {
          LastIndex = 0;
          Last_lastIndex = -999;
          publishPlan(plan);
          return !plan.empty();
        }
      }
    }

    if (LastIndex >= plan_bear)
      minIndex = LastIndex - plan_bear;
    else
      minIndex = 0;

    // printf("5-= %d,%d ,\n",minIndex,LastIndex);
    maxIndex = LastIndex + plan_front;

    if (maxIndex >= (int)roadPoints.size())
    {
      maxIndex = (int)roadPoints.size();
    }

    ros::Time plan_time = ros::Time::now();

    for (unsigned int i = minIndex; i < maxIndex; i++)
    {
      geometry_msgs::PoseStamped visualpose;
      geometry_msgs::PoseStamped carCoordinatePose;
      visualpose.header.stamp = plan_time;
      visualpose.header.frame_id = "map";

      if (minDis < Min_cartoplan_dis)
      {
        double diffDistance = getDis(start, roadPoints[i]);
        if (diffDistance < 1.2) // 局部路径取 3米 内得点
        {
          visualpose.header.frame_id = "map";
          visualpose.header.stamp = ros::Time::now();
          visualpose.pose.position.x = roadPoints[i].pose.position.x;
          visualpose.pose.position.y = roadPoints[i].pose.position.y;
          visualpose.pose.position.z = 0.1;
          visualpose.pose.orientation.x = roadPoints[i].pose.orientation.x;
          visualpose.pose.orientation.y = roadPoints[i].pose.orientation.y;
          visualpose.pose.orientation.z = roadPoints[i].pose.orientation.z;
          visualpose.pose.orientation.w = roadPoints[i].pose.orientation.w;
          //if (i % 4 == 0)
          {
            plan.push_back(visualpose);
          }
          if ((int)plan.size() >= 40)
          {
            break;
          }
        }
        else
          break;
      }
    }

    if (abs((int)roadPoints.size() - LastIndex) < 2 && (int)plan.size() < 2 || (int)plan.size() < 1)
    {
      LastIndex = 0;
      minIndex = 0;
      maxIndex = LastIndex + plan_front;
    }
    // printf("6-= %d ,",minIndex);
    Last_lastIndex = LastIndex;

    publishPlan(plan);

    return !plan.empty();
  }
  void MoveBase::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path)
  {

    nav_msgs::Path all_gui_path;
    all_gui_path.poses.resize((int)roadPoints.size());

    all_gui_path.header.frame_id = "map";
    all_gui_path.header.stamp = ros::Time::now();

    for (unsigned int i = 0; i < (int)roadPoints.size(); i++)
    {
      all_gui_path.poses[i] = roadPoints[i];
    }
    all_plan_pub_.publish(all_gui_path);

    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = "map";
    gui_path.header.stamp = ros::Time::now();

    for (unsigned int i = 0; i < path.size(); i++)
    {
      gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
  }

  bool MoveBase::makePlan(const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
  {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    // make sure to set the plan to be empty initially
    plan.clear();

    // since this gets called on handle activate
    if (planner_costmap_ros_ == NULL)
    {
      ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
      return false;
    }

    // get the starting pose of the robot
    geometry_msgs::PoseStamped global_pose;
    if (!getRobotPose(global_pose, planner_costmap_ros_))
    {
      ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
      return false;
    }

    const geometry_msgs::PoseStamped &start = global_pose;

    // if the planner fails or returns a zero length plan, planning failed
    if (!planner_->makePlan(start, goal, plan) || plan.empty())
    {
      ROS_DEBUG_NAMED("move_base", "Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    return true;
  }

  void MoveBase::publishZeroVelocity()
  {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    // cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

  bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion &q)
  {
    // first we need to check if the quaternion has nan's or infs
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
    {
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    // next, we need to check if the length of the quaternion is close to zero
    if (tf_q.length2() < 1e-6)
    {
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    // next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf2::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if (fabs(dot - 1) > 1e-3)
    {
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }

  geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped &goal_pose_msg)
  {
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    geometry_msgs::PoseStamped goal_pose, global_pose;
    goal_pose = goal_pose_msg;

    // just get the latest available transform... for accuracy they should send
    // goals in the frame of the planner
    goal_pose.header.stamp = ros::Time();

    try
    {
      tf_.transform(goal_pose_msg, global_pose, global_frame);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
               goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
      return goal_pose_msg;
    }

    return global_pose;
  }

  void MoveBase::wakePlanner(const ros::TimerEvent &event)
  {
    // we have slept long enough for rate
    planner_cond_.notify_one();
  }

  void MoveBase::planThread()
  {
    ROS_DEBUG_NAMED("move_base_plan_thread", "Starting planner thread...");
    ros::NodeHandle n;
    ros::Timer timer;
    bool wait_for_wake = false;
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    while (n.ok())
    {
      // check if we should run the planner (the mutex is locked)
      while (wait_for_wake || !runPlanner_)
      {
        // if we should not be running the planner then suspend this thread
        ROS_DEBUG_NAMED("move_base_plan_thread", "Planner thread is suspending");
        planner_cond_.wait(lock);
        wait_for_wake = false;
      }
      ros::Time start_time = ros::Time::now();

      // time to plan! get a copy of the goal and unlock the mutex
      geometry_msgs::PoseStamped temp_goal = planner_goal_;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base_plan_thread", "Planning...");

      // run planner
      planner_plan_->clear();
      bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

      if (gotPlan)
      {
        ROS_DEBUG_NAMED("move_base_plan_thread", "Got Plan with %zu points!", planner_plan_->size());
        // pointer swap the plans under mutex (the controller will pull from latest_plan_)
        std::vector<geometry_msgs::PoseStamped> *temp_plan = planner_plan_;

        lock.lock();
        planner_plan_ = latest_plan_;
        latest_plan_ = temp_plan;
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;
        new_global_plan_ = true;

        ROS_DEBUG_NAMED("move_base_plan_thread", "Generated a plan from the base_global_planner");

        // make sure we only start the controller if we still haven't reached the goal
        if (runPlanner_)
          state_ = CONTROLLING;
        if (planner_frequency_ <= 0)
          runPlanner_ = false;
        lock.unlock();
      }
      // if we didn't get a plan and we are in the planning state (the robot isn't moving)
      else if (state_ == PLANNING)
      {
        ROS_DEBUG_NAMED("move_base_plan_thread", "No Plan...");
        ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

        // check if we've tried to make a plan for over our time limit or our maximum number of retries
        // issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
        // is negative (the default), it is just ignored and we have the same behavior as ever
        lock.lock();
        planning_retries_++;
        if (runPlanner_ &&
            (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_)))
        {
          // we'll move into our obstacle clearing mode
          state_ = CLEARING;
          runPlanner_ = false; // proper solution for issue #523
          publishZeroVelocity();
          recovery_trigger_ = PLANNING_R;
        }

        lock.unlock();
      }

      // take the mutex for the next iteration
      lock.lock();

      // setup sleep interface if needed
      if (planner_frequency_ > 0)
      {
        ros::Duration sleep_time = (start_time + ros::Duration(1.0 / planner_frequency_)) - ros::Time::now();
        if (sleep_time > ros::Duration(0.0))
        {
          wait_for_wake = true;
          timer = n.createTimer(sleep_time, &MoveBase::wakePlanner, this);
        }
      }
    }
  }

  void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr &move_base_goal)
  {
    if (!isQuaternionValid(move_base_goal->target_pose.pose.orientation))
    {
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_base_goal->target_pose);

    publishZeroVelocity();
    // we have a goal so start the planner
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    planner_goal_ = goal;
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    current_goal_pub_.publish(goal);
    std::vector<geometry_msgs::PoseStamped> global_plan;

    ros::Rate r(controller_frequency_);
    if (shutdown_costmaps_)
    {
      ROS_DEBUG_NAMED("move_base", "Starting up costmaps that were shut down previously");
      planner_costmap_ros_->start();
      controller_costmap_ros_->start();
    }

    // we want to make sure that we reset the last time we had a valid plan and control
    last_valid_control_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();
    planning_retries_ = 0;

    ros::NodeHandle n;
    while (n.ok())
    {
      if (c_freq_change_)
      {
        ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
        r = ros::Rate(controller_frequency_);
        c_freq_change_ = false;
      }

      if (as_->isPreemptRequested())
      {
        if (as_->isNewGoalAvailable())
        {
          // if we're active and a new goal is available, we'll accept it, but we won't shut anything down
          move_base_msgs::MoveBaseGoal new_goal = *as_->acceptNewGoal();

          if (!isQuaternionValid(new_goal.target_pose.pose.orientation))
          {
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on goal because it was sent with an invalid quaternion");
            return;
          }

          goal = goalToGlobalFrame(new_goal.target_pose);

          // we'll make sure that we reset our state for the next execution cycle
          recovery_index_ = 0;
          state_ = PLANNING;

          // we have a new goal so make sure the planner is awake
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();

          // publish the goal point to the visualizer
          ROS_DEBUG_NAMED("move_base", "move_base has received a goal of x: %.2f, y: %.2f", goal.pose.position.x, goal.pose.position.y);
          current_goal_pub_.publish(goal);

          // make sure to reset our timeouts and counters
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();
          planning_retries_ = 0;
        }
        else
        {
          // if we've been preempted explicitly we need to shut things down
          resetState();

          // notify the ActionServer that we've successfully preempted
          ROS_DEBUG_NAMED("move_base", "Move base preempting the current goal");
          as_->setPreempted();

          // we'll actually return from execute after preempting
          return;
        }
      }

      // we also want to check if we've changed global frames because we need to transform our goal pose
      if (goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID())
      {
        goal = goalToGlobalFrame(goal);

        // we want to go back to the planning state for the next execution cycle
        recovery_index_ = 0;
        state_ = PLANNING;

        // we have a new goal so make sure the planner is awake
        lock.lock();
        planner_goal_ = goal;
        runPlanner_ = true;
        planner_cond_.notify_one();
        lock.unlock();

        // publish the goal point to the visualizer
        ROS_DEBUG_NAMED("move_base", "The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
        current_goal_pub_.publish(goal);

        // make sure to reset our timeouts and counters
        last_valid_control_ = ros::Time::now();
        last_valid_plan_ = ros::Time::now();
        last_oscillation_reset_ = ros::Time::now();
        planning_retries_ = 0;
      }

      // for timing that gives real time even in simulation
      ros::WallTime start = ros::WallTime::now();

      // the real work on pursuing a goal is done here
      bool done = executeCycle(goal, global_plan);

      // if we're done, then we'll return from execute
      if (done)
        return;

      // check if execution of the goal has completed in some way

      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG_NAMED("move_base", "Full control cycle time: %.9f\n", t_diff.toSec());

      r.sleep();
      // make sure to sleep for the remainder of our cycle time
      if (r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
        ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }

    // wake up the planner thread so that it can exit cleanly
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    // if the node is killed then we'll abort and return
    as_->setAborted(move_base_msgs::MoveBaseResult(), "Aborting on the goal because the node has been killed");
    return;
  }

  double MoveBase::distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

  bool MoveBase::executeCycle(geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &global_plan)
  {
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    // we need to be able to publish velocity commands
    geometry_msgs::Twist cmd_vel;

    // update feedback to correspond to our curent position
    geometry_msgs::PoseStamped global_pose;
    getRobotPose(global_pose, planner_costmap_ros_);
    const geometry_msgs::PoseStamped &current_position = global_pose;

    // push the feedback out
    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position = current_position;
    as_->publishFeedback(feedback);

    // check to see if we've moved far enough to reset our oscillation timeout
    if (distance(current_position, oscillation_pose_) >= oscillation_distance_)
    {
      last_oscillation_reset_ = ros::Time::now();
      oscillation_pose_ = current_position;

      // if our last recovery was caused by oscillation, we want to reset the recovery index
      if (recovery_trigger_ == OSCILLATION_R)
        recovery_index_ = 0;
    }

    // check that the observation buffers for the costmap are current, we don't want to drive blind
    if (!controller_costmap_ros_->isCurrent())
    {
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety", ros::this_node::getName().c_str());
      publishZeroVelocity();
      return false;
    }

    // if we have a new plan then grab it and give it to the controller
    if (new_global_plan_)
    {
      // make sure to set the new plan flag to false
      new_global_plan_ = false;

      ROS_DEBUG_NAMED("move_base", "Got a new plan...swap pointers");

      // do a pointer swap under mutex
      std::vector<geometry_msgs::PoseStamped> *temp_plan = controller_plan_;

      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      controller_plan_ = latest_plan_;
      latest_plan_ = temp_plan;
      lock.unlock();
      ROS_DEBUG_NAMED("move_base", "pointers swapped!");

      if (!tc_->setPlan(*controller_plan_))
      {
        // ABORT and SHUTDOWN COSTMAPS
        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        resetState();

        // disable the planner thread
        lock.lock();
        runPlanner_ = false;
        lock.unlock();

        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
        return true;
      }

      // make sure to reset recovery_index_ since we were able to find a valid plan
      if (recovery_trigger_ == PLANNING_R)
        recovery_index_ = 0;
    }

    // the move_base state machine, handles the control logic for navigation
    switch (state_)
    {
    // if we are in a planning state, then we'll attempt to make a plan
    case PLANNING:
    {
      boost::recursive_mutex::scoped_lock lock(planner_mutex_);
      runPlanner_ = true;
      planner_cond_.notify_one();
    }
      ROS_DEBUG_NAMED("move_base", "Waiting for plan, in the planning state.");
      break;

    // if we're controlling, we'll attempt to find valid velocity commands
    case CONTROLLING:
      ROS_DEBUG_NAMED("move_base", "In controlling state.");

      // check to see if we've reached our goal
      if (tc_->isGoalReached())
      {
        ROS_DEBUG_NAMED("move_base", "Goal reached!");
        resetState();

        // disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();

        as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
        return true;
      }

      // check for an oscillation condition
      if (oscillation_timeout_ > 0.0 &&
          last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
      {
        publishZeroVelocity();
        state_ = CLEARING;
        recovery_trigger_ = OSCILLATION_R;
      }

      {
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

        if (tc_->computeVelocityCommands(cmd_vel))
        {
          ROS_DEBUG_NAMED("move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                          cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
          last_valid_control_ = ros::Time::now();
          // make sure that we send the velocity command to the base
          vel_pub_.publish(cmd_vel);
          if (recovery_trigger_ == CONTROLLING_R)
            recovery_index_ = 0;
        }
        else
        {
          ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
          ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

          // check if we've tried to find a valid control for longer than our time limit
          if (ros::Time::now() > attempt_end)
          {
            // we'll move into our obstacle clearing mode
            publishZeroVelocity();
            state_ = CLEARING;
            recovery_trigger_ = CONTROLLING_R;
          }
          else
          {
            // otherwise, if we can't find a valid control, we'll go back to planning
            last_valid_plan_ = ros::Time::now();
            planning_retries_ = 0;
            state_ = PLANNING;
            publishZeroVelocity();

            // enable the planner thread in case it isn't running on a clock
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            runPlanner_ = true;
            planner_cond_.notify_one();
            lock.unlock();
          }
        }
      }

      break;

    // we'll try to clear out space with any user-provided recovery behaviors
    case CLEARING:
      ROS_DEBUG_NAMED("move_base", "In clearing/recovery state");
      // we'll invoke whatever recovery behavior we're currently on if they're enabled
      if (recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size())
      {
        ROS_DEBUG_NAMED("move_base_recovery", "Executing behavior %u of %zu", recovery_index_, recovery_behaviors_.size());
        recovery_behaviors_[recovery_index_]->runBehavior();

        // we at least want to give the robot some time to stop oscillating after executing the behavior
        last_oscillation_reset_ = ros::Time::now();

        // we'll check if the recovery behavior actually worked
        ROS_DEBUG_NAMED("move_base_recovery", "Going back to planning state");
        last_valid_plan_ = ros::Time::now();
        planning_retries_ = 0;
        state_ = PLANNING;

        // update the index of the next recovery behavior that we'll try
        recovery_index_++;
      }
      else
      {
        ROS_DEBUG_NAMED("move_base_recovery", "All recovery behaviors have failed, locking the planner and disabling it.");
        // disable the planner thread
        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        runPlanner_ = false;
        lock.unlock();

        ROS_DEBUG_NAMED("move_base_recovery", "Something should abort after this.");

        if (recovery_trigger_ == CONTROLLING_R)
        {
          ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
          as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
        }
        else if (recovery_trigger_ == PLANNING_R)
        {
          ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
          as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
        }
        else if (recovery_trigger_ == OSCILLATION_R)
        {
          ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
          as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
        }
        resetState();
        return true;
      }
      break;
    default:
      ROS_ERROR("This case should never be reached, something is wrong, aborting");
      resetState();
      // disable the planner thread
      boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
      runPlanner_ = false;
      lock.unlock();
      as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
      return true;
    }

    // we aren't done yet
    return false;
  }

  bool MoveBase::loadRecoveryBehaviors(ros::NodeHandle node)
  {
    XmlRpc::XmlRpcValue behavior_list;
    if (node.getParam("recovery_behaviors", behavior_list))
    {
      if (behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        for (int i = 0; i < behavior_list.size(); ++i)
        {
          if (behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
          {
            if (behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type"))
            {
              // check for recovery behaviors with the same name
              for (int j = i + 1; j < behavior_list.size(); j++)
              {
                if (behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct)
                {
                  if (behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type"))
                  {
                    std::string name_i = behavior_list[i]["name"];
                    std::string name_j = behavior_list[j]["name"];
                    if (name_i == name_j)
                    {
                      ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                                name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }
            else
            {
              ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
              return false;
            }
          }
          else
          {
            ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                      behavior_list[i].getType());
            return false;
          }
        }

        // if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
        for (int i = 0; i < behavior_list.size(); ++i)
        {
          try
          {
            // check if a non fully qualified name has potentially been passed in
            if (!recovery_loader_.isClassAvailable(behavior_list[i]["type"]))
            {
              std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
              for (unsigned int i = 0; i < classes.size(); ++i)
              {
                if (behavior_list[i]["type"] == recovery_loader_.getName(classes[i]))
                {
                  // if we've found a match... we'll get the fully qualified name and break out of the loop
                  ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                           std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                  behavior_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

            // shouldn't be possible, but it won't hurt to check
            if (behavior.get() == NULL)
            {
              ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            // initialize the recovery behavior with its name
            behavior->initialize(behavior_list[i]["name"], &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behaviors_.push_back(behavior);
          }
          catch (pluginlib::PluginlibException &ex)
          {
            ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
            return false;
          }
        }
      }
      else
      {
        ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
                  behavior_list.getType());
        return false;
      }
    }
    else
    {
      // if no recovery_behaviors are specified, we'll just load the defaults
      return false;
    }

    // if we've made it here... we've constructed a recovery behavior list successfully
    return true;
  }

  // we'll load our default recovery behaviors here
  void MoveBase::loadDefaultRecoveryBehaviors()
  {
    recovery_behaviors_.clear();
    try
    {
      // we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
      ros::NodeHandle n("~");
      n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
      n.setParam("aggressive_reset/reset_distance", circumscribed_radius_ * 4);

      // first, we'll load a recovery behavior to clear the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(cons_clear);

      // next, we'll load a recovery behavior to rotate in place
      boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
      if (clearing_rotation_allowed_)
      {
        rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behaviors_.push_back(rotate);
      }

      // next, we'll load a recovery behavior that will do an aggressive reset of the costmap
      boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
      ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(ags_clear);

      // we'll rotate in-place one more time
      if (clearing_rotation_allowed_)
        recovery_behaviors_.push_back(rotate);
    }
    catch (pluginlib::PluginlibException &ex)
    {
      ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }

    return;
  }

  void MoveBase::resetState()
  {
    // Disable the planner thread
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    // if we shutdown our costmaps when we're deactivated... we'll do that now
    if (shutdown_costmaps_)
    {
      ROS_DEBUG_NAMED("move_base", "Stopping costmaps");
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
    }
  }

  bool MoveBase::getRobotPose(geometry_msgs::PoseStamped &global_pose, costmap_2d::Costmap2DROS *costmap)
  {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = ros::Time();     // latest available
    ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

    // get robot pose on the given costmap frame
    try
    {
      tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    }
    catch (tf2::LookupException &ex)
    {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException &ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException &ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

    // check if global_pose time stamp is within costmap transform tolerance
    if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
    {
      ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. "
                             "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f",
                        costmap->getName().c_str(),
                        current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
      return false;
    }

    return true;
  }

};
