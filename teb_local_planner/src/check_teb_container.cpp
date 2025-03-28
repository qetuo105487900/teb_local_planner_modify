#include <ros/ros.h>
#include <campusrover_msgs/PathArray.h>  // 自定義訊息，裡面包含 nav_msgs/Path[] path
#include <algorithm>  // std::min

// 回呼函數：處理 teb_container 主題的訊息
void tebContainerCallback(const campusrover_msgs::PathArray::ConstPtr& msg)
{
  // 計算收到的路徑數量
  int num_paths = msg->path.size();
  ROS_INFO("received teb_container, has %d paths", num_paths);
  
  // 遍歷每一條路徑
  for (size_t i = 0; i < msg->path.size(); ++i)
  {
    const nav_msgs::Path &path = msg->path[i];
    int num_points = path.poses.size();
    ROS_INFO("Path %zu has %d pose", i, num_points);
    
    // 取前 10 個點 (若不足 10 個則全部顯示)
    int points_to_print = std::min(num_points, 10);
    for (int j = 0; j < points_to_print; ++j)
    {
      const geometry_msgs::PoseStamped &pose_stamped = path.poses[j];
      ROS_INFO("Path %zu, pose %d : x = %.2f, y = %.2f, z = %.2f",
               i, j,
               pose_stamped.pose.position.x,
               pose_stamped.pose.position.y,
               pose_stamped.pose.position.z);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teb_container_subscriber");
  ros::NodeHandle nh;
  
  // 訂閱 "teb_container" 主題
  ros::Subscriber sub = nh.subscribe("/test_optim_node/local_planner", 10, tebContainerCallback);
  
  ros::spin();
  return 0;
}