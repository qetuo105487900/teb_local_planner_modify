/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017.
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
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
 *   * Neither the name of the institute nor the names of its
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
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/teb_local_planner_ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <campusrover_msgs/TEBStartEnd.h> //srv
#include <campusrover_msgs/TEBVias.h> //srv
#include <campusrover_msgs/TEBStartEndViasMsg.h> //msg
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace teb_local_planner; // it is ok here to import everything for testing purposes

// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.
PlannerInterfacePtr planner;
TebVisualizationPtr visual;
std::vector<ObstaclePtr> obst_vector;
ViaPointContainer via_points;
TebConfig config;
boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg;

ros::Subscriber custom_obst_sub;
ros::Subscriber via_points_sub;
ros::Subscriber clicked_points_sub;
ros::Subscriber teb_start_end_sub, teb_stop_sub, stop_teb_sub, teb_start_end_vias_sub, mpc_turn_on_sub;

std::vector<ros::Subscriber> obst_vel_subs;
unsigned int no_fixed_obstacles;

int mpc_turn_on_count = 0 ;
bool mpc_turn_on_flag = false ;

bool valid_start_goal = false;
PoseSE2 start_pose(0, 0, 0);
PoseSE2 goal_pose(0, 0, 0);

// =========== Function declarations =============
void CB_mainCycle(const ros::TimerEvent& e);
// void CB_publishCycle(const ros::TimerEvent& e);
void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level);
void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg);
void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb);
void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg);
// void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg);
void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr& twist_msg, const unsigned int id);

void StopTEBCallback(const std_msgs::Bool &flag);
void MPCTurnOnCallback(const std_msgs::Bool &flag);
// void Start_End_Callback(const geometry_msgs::PoseArray::ConstPtr& msg);
// void StartEndBoolCallback(const std_msgs::Bool::ConstPtr& msg);
void StartEndViasCallback(const campusrover_msgs::TEBStartEndViasMsg::ConstPtr& msg);

bool StopTEBServiceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
bool TEBStartEndCallback(campusrover_msgs::TEBStartEnd::Request &req, campusrover_msgs::TEBStartEnd::Response &res);
bool TEBViasCallback(campusrover_msgs::TEBVias::Request &req, campusrover_msgs::TEBVias::Response &res);

// =============== Main function =================
int main( int argc, char** argv )
{
  ros::init(argc, argv, "test_optim_node");
  ros::NodeHandle n("~");
 
  
  // load ros parameters from node handle
  config.loadRosParamFromNodeHandle(n);
 
  ros::Timer cycle_timer = n.createTimer(ros::Duration(0.025), CB_mainCycle);
  // ros::Timer publish_timer = n.createTimer(ros::Duration(0.1), CB_publishCycle);
  
  // setup dynamic reconfigure
  dynamic_recfg = boost::make_shared< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> >(n);
  dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(CB_reconfigure, _1, _2);
  dynamic_recfg->setCallback(cb);
  
  
  // setup callback for custom obstacles
  custom_obst_sub = n.subscribe("/teb_obstacles", 10, CB_customObstacle);
  
  // setup callback for clicked points (in rviz) that are considered as via-points
  clicked_points_sub = n.subscribe("/clicked_point", 10, CB_clicked_points);
  
  teb_start_end_vias_sub = n.subscribe("/teb_start_end_vias", 10, StartEndViasCallback);
  // teb_start_end_sub = n.subscribe("/teb_start_end", 10, Start_End_Callback);
  ros::ServiceServer teb_start_end_service = n.advertiseService("/teb_start_end", TEBStartEndCallback);
  ros::ServiceServer teb_stop_service = n.advertiseService("/teb_stop", StopTEBServiceCallback);
  
  mpc_turn_on_sub = n.subscribe("/teb_mpc_turn_on", 10, MPCTurnOnCallback);
  stop_teb_sub = n.subscribe("/teb_mpc_finish", 10, StopTEBCallback);
  // teb_stop_sub = n.subscribe("/teb_stop", 10, StartEndBoolCallback);
  // setup callback for via-points (callback overwrites previously set via-points)
  // via_points_sub = n.subscribe("/teb_via_points", 10, CB_via_points);
  ros::ServiceServer via_service = n.advertiseService("/teb_via_points", TEBViasCallback);
  // interactive marker server for simulated dynamic obstacles
  interactive_markers::InteractiveMarkerServer marker_server("marker_obstacles");

  // obst_vector.push_back( boost::make_shared<PointObstacle>(-3,1) );
  // obst_vector.push_back( boost::make_shared<PointObstacle>(3,2) );
  // obst_vector.push_back( boost::make_shared<PointObstacle>(0,0.1) );
  // obst_vector.push_back( boost::make_shared<LineObstacle>(1,1.5,1,-1.5) ); //90 deg
  // obst_vector.push_back( boost::make_shared<LineObstacle>(1,0,-1,0) ); //180 deg
  // obst_vector.push_back( boost::make_shared<PointObstacle>(-1.5,-0.5) );

  // Dynamic obstacles
  // Eigen::Vector2d vel (0.1, -0.3);
  // obst_vector.at(0)->setCentroidVelocity(vel);
  // vel = Eigen::Vector2d(-0.3, -0.2);
  // obst_vector.at(1)->setCentroidVelocity(vel);

  
  // PolygonObstacle* polyobst = new PolygonObstacle;
  // polyobst->pushBackVertex(1, -1);
  // polyobst->pushBackVertex(0, 1);
  // polyobst->pushBackVertex(1, 1);
  // polyobst->pushBackVertex(2, 1);
 
  // polyobst->finalizePolygon();
  // obst_vector.emplace_back(polyobst);
  
  
  for (unsigned int i=0; i<obst_vector.size(); ++i)
  {
    // setup callbacks for setting obstacle velocities
    std::string topic = "/test_optim_node/obstacle_" + std::to_string(i) + "/cmd_vel";
    obst_vel_subs.push_back(n.subscribe<geometry_msgs::Twist>(topic, 1, boost::bind(&CB_setObstacleVelocity, _1, i)));

    //CreateInteractiveMarker(obst_vector.at(i)[0],obst_vector.at(i)[1],i,&marker_server, &CB_obstacle_marker);  
    // Add interactive markers for all point obstacles
    boost::shared_ptr<PointObstacle> pobst = boost::dynamic_pointer_cast<PointObstacle>(obst_vector.at(i));
    if (pobst)
    {
      CreateInteractiveMarker(pobst->x(),pobst->y(),i, config.map_frame, &marker_server, &CB_obstacle_marker);  
    }
  }
  marker_server.applyChanges();
  
  // Setup visualization
  visual = TebVisualizationPtr(new TebVisualization(n, config));
  
  // Setup robot shape model
  config.robot_model = TebLocalPlannerROS::getRobotFootprintFromParamServer(n, config);
  
  // Setup planner (homotopy class planning or just the local teb planner)
  if (config.hcp.enable_homotopy_class_planning)
    planner = PlannerInterfacePtr(new HomotopyClassPlanner(config, &obst_vector, visual, &via_points));
  else
    planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, visual, &via_points));
  
  no_fixed_obstacles = obst_vector.size();
  ros::spin();

  return 0;
}

// Planning loop
void CB_mainCycle(const ros::TimerEvent& e)
{
  if (valid_start_goal)
  {
    // 使用接收到的起點與終點
    planner->plan(start_pose, goal_pose);
    planner->visualize();
    visual->publishObstacles(obst_vector);
    visual->publishViaPoints(via_points);
  }
  //起點終點在這 基於odom
  // planner->plan(PoseSE2(-4,0,-1), PoseSE2(4,0,0.57)); // hardcoded start and goal for testing purposes
}

// Visualization loop
// void CB_publishCycle(const ros::TimerEvent& e)
// {
//   planner->visualize();
//   visual->publishObstacles(obst_vector);
//   visual->publishViaPoints(via_points);
// }

void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level)
{
  config.reconfigure(reconfig);
}

void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb)
{
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker i_marker;
  i_marker.header.frame_id = frame;
  i_marker.header.stamp = ros::Time::now();
  std::ostringstream oss;
  //oss << "obstacle" << id;
  oss << id;
  i_marker.name = oss.str();
  i_marker.description = "Obstacle";
  i_marker.pose.position.x = init_x;
  i_marker.pose.position.y = init_y;
  i_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.id = id;
  box_marker.scale.x = 0.2;
  box_marker.scale.y = 0.2;
  box_marker.scale.z = 0.2;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;
  box_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  i_marker.controls.push_back( box_control );

  // create a control which will move the box, rviz will insert 2 arrows
  visualization_msgs::InteractiveMarkerControl move_control;
  move_control.name = "move_x";
  move_control.orientation.w = 0.707107f;
  move_control.orientation.x = 0;
  move_control.orientation.y = 0.707107f;
  move_control.orientation.z = 0;
  move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

  // add the control to the interactive marker
  i_marker.controls.push_back(move_control);

  // add the interactive marker to our collection
  marker_server->insert(i_marker);
  marker_server->setCallback(i_marker.name,feedback_cb);
}

void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::stringstream ss(feedback->marker_name);
  unsigned int index;
  ss >> index;
  
  if (index>=no_fixed_obstacles) 
    return;
  PointObstacle* pobst = static_cast<PointObstacle*>(obst_vector.at(index).get());
  pobst->position() = Eigen::Vector2d(feedback->pose.position.x,feedback->pose.position.y);	  
}

void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
  ROS_INFO_ONCE("Custom obstacle message received with frame_id: %s", obst_msg->header.frame_id.c_str());
  // resize such that the vector contains only the fixed obstacles specified inside the main function
  obst_vector.resize(no_fixed_obstacles);
  
  // Add custom obstacles obtained via message (assume that all obstacles coordiantes are specified in the default planning frame)  
  for (size_t i = 0; i < obst_msg->obstacles.size(); ++i)
  {
    if (obst_msg->obstacles.at(i).polygon.points.size() == 1 )
    {
      if (obst_msg->obstacles.at(i).radius == 0) 
      {
        obst_vector.push_back(ObstaclePtr(new PointObstacle( obst_msg->obstacles.at(i).polygon.points.front().x,
                                                           obst_msg->obstacles.at(i).polygon.points.front().y )));
      }
      else
      {
        obst_vector.push_back(ObstaclePtr(new CircularObstacle( obst_msg->obstacles.at(i).polygon.points.front().x,
                                                            obst_msg->obstacles.at(i).polygon.points.front().y,
                                                            obst_msg->obstacles.at(i).radius )));
      }
    }
    else if (obst_msg->obstacles.at(i).polygon.points.empty())
    {
      ROS_WARN("Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
      continue;
    }
    else
    {
      PolygonObstacle* polyobst = new PolygonObstacle;
      for (size_t j=0; j<obst_msg->obstacles.at(i).polygon.points.size(); ++j)
      {
        polyobst->pushBackVertex( obst_msg->obstacles.at(i).polygon.points[j].x,
                                  obst_msg->obstacles.at(i).polygon.points[j].y );
      }
      polyobst->finalizePolygon();
      obst_vector.push_back(ObstaclePtr(polyobst));
    }

    if(!obst_vector.empty())
      obst_vector.back()->setCentroidVelocity(obst_msg->obstacles.at(i).velocities, obst_msg->obstacles.at(i).orientation);
  }
  // ROS_INFO("Current number of obstacles: %lu", obst_msg->obstacles.size());
}

void StartEndViasCallback(const campusrover_msgs::TEBStartEndViasMsg::ConstPtr& msg)
{
  via_points.clear();
  if (mpc_turn_on_flag && mpc_turn_on_count >= 1)
  {
    ROS_INFO_ONCE("Via-points received. This message is printed once.");
    for (const geometry_msgs::PoseStamped& pose : msg->vias.poses)
    {
      via_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }

    // 將第一個pose當作起點，第二個pose當作終點
    geometry_msgs::Pose start, end;

    start = msg->start.pose;
    end = msg->end.pose;

    double yaw_start = tf::getYaw(start.orientation);
    double yaw_end = tf::getYaw(end.orientation);

    // 檢查起點與終點之間的距離
    double dx = end.position.x - start.position.x;
    double dy = end.position.y - start.position.y;

    double distance = std::sqrt(dx * dx + dy * dy);

    start_pose = PoseSE2(start.position.x, start.position.y, yaw_start);
    goal_pose  = PoseSE2(end.position.x,  end.position.y,  yaw_end);
    
    valid_start_goal = true;

    ROS_INFO("[test_optim_node] Received valid_start_goal = true");
      
  }

}

// void Start_End_Callback(const geometry_msgs::PoseArray::ConstPtr& msg)
// {
//   // 將第一個pose當作起點，第二個pose當作終點
//   geometry_msgs::Pose start, goal;

//   start = msg->poses[0];
//   goal = msg->poses[1];

//   double yaw_start = tf::getYaw(start.orientation);
//   double yaw_goal = tf::getYaw(goal.orientation);

//   // 檢查起點與終點之間的距離
//   double dx = goal.position.x - start.position.x;
//   double dy = goal.position.y - start.position.y;

//   double distance = std::sqrt(dx * dx + dy * dy);

//   if (distance > 0.3)
//   {
//     start_pose = PoseSE2(start.position.x, start.position.y, yaw_start);
//     goal_pose  = PoseSE2(goal.position.x,  goal.position.y,  yaw_goal);
//     ROS_INFO("[test_optim_node] legal start and end, dis = %f", distance);
//     valid_start_goal = true;
//   }
//   else
//   {
//     valid_start_goal = false;
//     ROS_WARN("[test_optim_node] illegal start and end, dis = %f", distance);
//   }  
// }

bool TEBStartEndCallback(campusrover_msgs::TEBStartEnd::Request &req, campusrover_msgs::TEBStartEnd::Response &res)
{

  // 將請求的 start 與 end 取出，注意它們為 PoseStamped
  geometry_msgs::Pose start = req.start.pose;
  geometry_msgs::Pose goal = req.end.pose;

  double yaw_start = tf::getYaw(start.orientation);
  double yaw_goal = tf::getYaw(goal.orientation);

  // 檢查起點與終點之間的距離
  double dx = goal.position.x - start.position.x;
  double dy = goal.position.y - start.position.y;
  double distance = std::sqrt(dx * dx + dy * dy);

  if (distance > 0.2)
  {
    start_pose = PoseSE2(start.position.x, start.position.y, yaw_start);
    goal_pose  = PoseSE2(goal.position.x,  goal.position.y,  yaw_goal);
    ROS_INFO("[test_optim_node] legal start and end, dis = %f", distance);
    valid_start_goal = true;
    res.success = true;
  }
  else
  {
    valid_start_goal = false;
    ROS_WARN("[test_optim_node] illegal start and end, dis = %f", distance);
    res.success = false;
  }
  
  return true;
}

// void StartEndBoolCallback(const std_msgs::Bool::ConstPtr& msg)
// {
//   if (msg->data)
//   {
//     valid_start_goal = false;
//     ROS_WARN("[test_optim_node] Received true: setting valid_start_goal = false");
//   }
//   else
//   {
//     ROS_INFO("[test_optim_node] Received false: valid_start_goal remains unchanged");
//   }
// }

bool StopTEBServiceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  if (req.data)
  {
    valid_start_goal = false;
    ROS_WARN("[test_optim_node] Received true: setting valid_start_goal = false");
    res.success = true;
    res.message = "valid_start_goal set to false.";
  }
  else
  {
    ROS_INFO("[test_optim_node] Received false: valid_start_goal remains unchanged");
    res.success = true;
    res.message = "valid_start_goal unchanged.";
  }
  return true;
}

void StopTEBCallback(const std_msgs::Bool &flag)
{
  if (flag.data)
  {
    ROS_INFO("[test_optim_node] valid_start_goal = false");
    valid_start_goal = false;
  }
}

void MPCTurnOnCallback(const std_msgs::Bool &flag) //接收mpc start了沒
{
  if (flag.data)
  {
    mpc_turn_on_flag = true ;
    mpc_turn_on_count++ ; //第一次變true發布 其餘不發布
    ROS_INFO("[test optim node] MPC is turned on. Count: %d", mpc_turn_on_count);
  }
  else
  {
    mpc_turn_on_flag = false ;
    mpc_turn_on_count = 0 ;
  }
}

void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg)
{
  // we assume for simplicity that the fixed frame is already the map/planning frame
  // consider clicked points as via-points
  via_points.push_back( Eigen::Vector2d(point_msg->point.x, point_msg->point.y) );
  ROS_INFO_STREAM("Via-point (" << point_msg->point.x << "," << point_msg->point.y << ") added.");
  if (config.optim.weight_viapoint<=0)
    ROS_WARN("Note, via-points are deactivated, since 'weight_via_point' <= 0");
}

// void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg)
// {
//   ROS_INFO_ONCE("Via-points received. This message is printed once.");
//   via_points.clear();
//   for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
//   {
//     via_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
//   }
// }

bool TEBViasCallback(campusrover_msgs::TEBVias::Request &req, campusrover_msgs::TEBVias::Response &res)
{
  ROS_INFO("TEBVias service called.");
  
  // 清除之前儲存的 via points
  via_points.clear();
  
  // 從請求的 path 中取出所有點的 x 與 y 座標，存入 via_points
  for (const geometry_msgs::PoseStamped &pose : req.path.poses)
  {
    via_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  
  res.success = true;
  return true;
}

void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr& twist_msg, const unsigned int id)
{
  if (id >= obst_vector.size())
  {
    ROS_WARN("Cannot set velocity: unknown obstacle id.");
    return;
  }

  Eigen::Vector2d vel (twist_msg->linear.x, twist_msg->linear.y);
  obst_vector.at(id)->setCentroidVelocity(vel);
}
