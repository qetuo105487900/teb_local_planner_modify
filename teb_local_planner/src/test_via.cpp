#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <utility>

#include <campusrover_msgs/TEBVias.h>  // 假設該服務定義：Request 包含 nav_msgs::Path, Response 包含 bool success, string message, nav_msgs::Path path
#include <std_srvs/Trigger.h>

// 透過 via_client 呼叫 "/teb_via_points" 服務
ros::ServiceClient via_client;

// 定義三組點位 (基於 odom 坐標)
std::vector<std::vector<std::pair<double, double>>> point_sets = {
    { {3.0, 3.0}
 },   // 第1組
    {  },                                           // 第2組 (空集合)
    { {3.0, 3.0}, {-2.5, 0.0}, {-1.75, 4.0} }        // 第3組
};

int set_index = 0; // 當前使用哪一組點位

// Trigger 服務 callback：切換至第1組點位，並呼叫 via_client 傳送對應的點組
bool TriggerSet1Callback(std_srvs::Trigger::Request &req,
                           std_srvs::Trigger::Response &res)
{
  set_index = 0;

  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "odom";  // 基於 odom 坐標系

  // 將第1組點位逐一加入 path_msg
  for (const auto &pt : point_sets[set_index])
  {
    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = pt.first;
    pose.pose.position.y = pt.second;
    pose.pose.position.z = 0.0;
    // 預設無旋轉
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }

  campusrover_msgs::TEBVias via_srv;
  via_srv.request.path = path_msg;

  // 呼叫 via 服務
  if(via_client.call(via_srv))
  {
    ROS_INFO("TEBVias service call succeeded for point set 1.");
  }
  else
  {
    ROS_WARN("TEBVias service call failed for point set 1.");
  }

  res.success = true;
  res.message = "Point set 1 activated.";
  ROS_INFO("Point set 1 activated.");
  return true;
}

// Trigger 服務 callback：切換至第2組點位，並呼叫 via_client 傳送對應的點組
bool TriggerSet2Callback(std_srvs::Trigger::Request &req,
                           std_srvs::Trigger::Response &res)
{
  set_index = 1;

  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "odom";

  // 將第2組點位逐一加入 path_msg (這裡可能為空集合)
  for (const auto &pt : point_sets[set_index])
  {
    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = pt.first;
    pose.pose.position.y = pt.second;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }

  campusrover_msgs::TEBVias via_srv;
  via_srv.request.path = path_msg;

  if(via_client.call(via_srv))
  {
    ROS_INFO("TEBVias service call succeeded for point set 2.");
  }
  else
  {
    ROS_WARN("TEBVias service call failed for point set 2.");
  }

  res.success = true;
  res.message = "Point set 2 activated.";
  ROS_INFO("Point set 2 activated.");
  return true;
}

// Trigger 服務 callback：切換至第3組點位，並呼叫 via_client 傳送對應的點組
bool TriggerSet3Callback(std_srvs::Trigger::Request &req,
                           std_srvs::Trigger::Response &res)
{
  set_index = 2;

  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "odom";

  // 將第3組點位逐一加入 path_msg
  for (const auto &pt : point_sets[set_index])
  {
    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = pt.first;
    pose.pose.position.y = pt.second;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }

  campusrover_msgs::TEBVias via_srv;
  via_srv.request.path = path_msg;

  if(via_client.call(via_srv))
  {
    ROS_INFO("TEBVias service call succeeded for point set 3.");
  }
  else
  {
    ROS_WARN("TEBVias service call failed for point set 3.");
  }

  res.success = true;
  res.message = "Point set 3 activated.";
  ROS_INFO("Point set 3 activated.");
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_via");
  ros::NodeHandle nh;

  // 透過 via_client 呼叫服務 "/teb_via_points"
  via_client = nh.serviceClient<campusrover_msgs::TEBVias>("/teb_via_points");

  // 建立三個 Trigger 服務，分別用來切換不同的點位組
  ros::ServiceServer trigger_set1 = nh.advertiseService("test_via1", TriggerSet1Callback);
  ros::ServiceServer trigger_set2 = nh.advertiseService("test_via2", TriggerSet2Callback);
  ros::ServiceServer trigger_set3 = nh.advertiseService("test_via3", TriggerSet3Callback);

  ROS_INFO("Path service server and trigger services are ready.");
  ros::spin();
  return 0;
}
