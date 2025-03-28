#include <ros/ros.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_obstacle");
  ros::NodeHandle nh;
  
  ros::Publisher obst_pub = nh.advertise<costmap_converter::ObstacleArrayMsg>("/teb_obstacles", 10);
  
  // 使用10 Hz的更新頻率
  ros::Rate rate(10);
  
  // 初始位置與參數設定
  double current_x = 6.0;  // 每次從 (6,4) 開始
  const double y = 4.0;
  const double radius = 1.5;
  const double speed = 0.5;  // m/s
  // dt 根據更新頻率計算，20 Hz 下 dt = 1/20 = 0.05秒
  const double dt = 1.0 / 20.0;
  const double target_left = 2.0;
  const double start_x = 6.0;  // 重置位置
  
  int counter = 0; // 用來記錄循環次數
  
  while (ros::ok())
  {
    // 每次更新 obstacle 往左移動
    current_x -= speed * dt;
    
    // 當障礙物移動到 (2,4) 時，重置回 (6,4)
    if (current_x <= target_left)
    {
      current_x = start_x;
    }
    
    // 建立障礙物訊息
    costmap_converter::ObstacleArrayMsg obst_array_msg;
    obst_array_msg.header.stamp = ros::Time::now();
    obst_array_msg.header.frame_id = "odom";

    costmap_converter::ObstacleMsg obst_msg;
    geometry_msgs::Point32 pt;
    pt.x = current_x;
    pt.y = y;
    pt.z = 0.0;
    obst_msg.polygon.points.push_back(pt);
    
    obst_msg.radius = radius;
    
    // 設定障礙物的速度，永遠往左移動
    obst_msg.velocities.twist.linear.x = -speed;
    obst_msg.velocities.twist.linear.y = 0.0;
    obst_msg.velocities.twist.linear.z = 0.0;
    obst_msg.velocities.twist.angular.x = 0.0;
    obst_msg.velocities.twist.angular.y = 0.0;
    obst_msg.velocities.twist.angular.z = 0.0;
    
    // 設定障礙物的朝向（無旋轉）
    obst_msg.orientation.w = 1.0;
    obst_msg.orientation.x = 0.0;
    obst_msg.orientation.y = 0.0;
    obst_msg.orientation.z = 0.0;
    
    obst_array_msg.obstacles.push_back(obst_msg);
    
    // 發佈障礙物訊息
    obst_pub.publish(obst_array_msg);
    
    // 每5次循環打印一次障礙物位置
    counter++;
    if (counter % 10 == 0)
    {
      ROS_INFO("Published obstacle at (%.2f, %.2f)", current_x, y);
    }
    
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
