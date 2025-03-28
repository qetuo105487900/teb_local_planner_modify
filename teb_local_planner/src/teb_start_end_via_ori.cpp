#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>

#include <campusrover_msgs/TEBVias.h>
#include <campusrover_msgs/TEBStartEnd.h>
#include <std_msgs/Bool.h>

ros::ServiceClient via_client, start_end_client;

geometry_msgs::PoseStamped robot_tf_poseStamped_;

int mpc_turn_on_count = 0 ;
bool mpc_turn_on_flag = false ;

void MPCTurnOnCallback(const std_msgs::Bool &flag) //接收mpc start了沒
{
    if (flag.data)
    {
        mpc_turn_on_flag = true ;
        mpc_turn_on_count++ ; //第一次變true發布 其餘不發布
        ROS_INFO("[teb_start_end_via] MPC is turned on. Count: %d", mpc_turn_on_count);
    }
    else
    {
        mpc_turn_on_flag = false ;
        mpc_turn_on_count = 0 ;
        // ROS_INFO("[teb_start_end_via] MPC is turned off.");
    }
}

void TebGoalCallback(const nav_msgs::Path::ConstPtr& msg)
{
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);

    if(msg->poses.size() > 0)
    {
        // 先建立 new_path，在前端加入目前機器人的位置
        nav_msgs::Path new_path;
        new_path.header = msg->header;
        
        geometry_msgs::PoseStamped current_pose_stamped;
        current_pose_stamped.header.stamp = ros::Time::now();
        current_pose_stamped.header.frame_id = "world"; // 假設 TF 在 world 座標系
        current_pose_stamped.pose = robot_tf_poseStamped_.pose;
        new_path.poses.push_back(current_pose_stamped);
        
        // 接著將原本收到的路徑依序接上
        new_path.poses.insert(new_path.poses.end(), msg->poses.begin(), msg->poses.end());

        geometry_msgs::TransformStamped tf_world_to_odom;
        try
        {
            // 將 "world" 座標系轉換到 "odom" 座標系 (以 ros::Time(0) 取得最新的轉換)
            tf_world_to_odom = tfBuffer.lookupTransform("odom", new_path.header.frame_id, ros::Time(0), ros::Duration(2.0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("[teb_start_end_via] Transform failed: %s", ex.what());
            return;
        }

        nav_msgs::Path transformed_path;
        transformed_path.header.frame_id = "odom";
        transformed_path.header.stamp = ros::Time::now();
        for (const auto &pose_stamped : new_path.poses)
        {
            geometry_msgs::PoseStamped transformed_pose;
            tf2::doTransform(pose_stamped, transformed_pose, tf_world_to_odom);
            transformed_path.poses.push_back(transformed_pose);
        }

        // 依照 transformed_path 的點數決定：
        // 如果路徑點數大於 2，則第一點為起點、最後一點為終點，中間點以 nav_msgs::Path 儲存
        // 如果只有 2 點，則中間點為空
        geometry_msgs::PoseStamped start_pose, end_pose;
        nav_msgs::Path middle_path;
        middle_path.header = transformed_path.header;
        
        if(transformed_path.poses.size() > 2)
        {
            start_pose = transformed_path.poses.front();
            end_pose = transformed_path.poses.back();
            // 儲存介於起點與終點之間的中間路徑點
            for (size_t i = 1; i < transformed_path.poses.size()-1; i++)
            {
                middle_path.poses.push_back(transformed_path.poses[i]);
            }
        }
        else if(transformed_path.poses.size() == 2)
        {
            start_pose = transformed_path.poses.front();
            end_pose = transformed_path.poses.back();
            // middle_path 留空
        }
        else
        {
            ROS_WARN("[teb_start_end_via] Path does not contain enough poses.");
            return;
        }
        
        // 建立 via service 的請求訊息，將中間的 via points 存入
        campusrover_msgs::TEBVias via_srv;
        via_srv.request.path = middle_path;
        
        // 建立 start/end service 的請求訊息
        campusrover_msgs::TEBStartEnd start_end_srv;
        start_end_srv.request.start = start_pose;
        start_end_srv.request.end = end_pose;
        
        // 呼叫 via 服務
        if(via_client.call(via_srv))
        {
            if(via_srv.response.success)
            {
                ROS_INFO("[teb_start_end_via] TEB vias service call succeeded.");

                while (!(mpc_turn_on_flag)) 
                {
                    ROS_INFO("[teb_start_end_via] mpc_turn_on_flag waiting 0.1 sec");
                    ros::Duration(0.1).sleep();
                }

                if (mpc_turn_on_count >= 1) //第一次變true發布 其餘不發布
                {
                    if(start_end_client.call(start_end_srv))
                    {
                        if(start_end_srv.response.success)
                        {
                            ROS_INFO("[teb_start_end_via] TEB start/end service call succeeded.");
                        }
                        else
                        {
                            ROS_WARN("[teb_start_end_via] TEB start/end service call returned failure. Retrying...");
                        }
                    }
                    else
                    {
                        ROS_WARN("[teb_start_end_via] TEB start/end service call failed. Retrying...");
                    }
                }
            }
            else
            {
                ROS_WARN("[teb_start_end_via] TEB vias service call returned failure.");
            }
        }
        else
        {
            ROS_ERROR("[teb_start_end_via] TEB vias service call failed.");
        }
    }
}

void CurrentPoseTimer(const ros::TimerEvent &event)
{
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);
    static geometry_msgs::TransformStamped transformStamped;

    try
    {
        transformStamped = tfBuffer.lookupTransform("world", "base_link", ros::Time(0), ros::Duration(2));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("[teb_start_end_via]  %s. Can't update pose from TF, using latest available.", ex.what());
    }

    robot_tf_poseStamped_.header.frame_id = "world";
    robot_tf_poseStamped_.header.stamp = ros::Time::now();
    robot_tf_poseStamped_.pose.position.x = transformStamped.transform.translation.x;
    robot_tf_poseStamped_.pose.position.y = transformStamped.transform.translation.y;
    robot_tf_poseStamped_.pose.position.z = transformStamped.transform.translation.z;
    robot_tf_poseStamped_.pose.orientation.x = transformStamped.transform.rotation.x;
    robot_tf_poseStamped_.pose.orientation.y = transformStamped.transform.rotation.y;
    robot_tf_poseStamped_.pose.orientation.z = transformStamped.transform.rotation.z;
    robot_tf_poseStamped_.pose.orientation.w = transformStamped.transform.rotation.w;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teb_start_end_via_node");
  ros::NodeHandle nh;
  
  ros::Subscriber teb_goal_sub = nh.subscribe("/teb_global_path", 10, TebGoalCallback);
  ros::Subscriber mpc_turn_on_sub = nh.subscribe("/mpc_turn_on", 10, MPCTurnOnCallback);  

  via_client = nh.serviceClient<campusrover_msgs::TEBVias>("/teb_via_points");
  start_end_client = nh.serviceClient<campusrover_msgs::TEBStartEnd>("/teb_start_end");

  ros::Timer current_pose_timer = nh.createTimer(ros::Duration(0.1), CurrentPoseTimer);
  ros::Timer start_end_via_timer = nh.createTimer(ros::Duration(0.1), StartEndViaTimer);

  ros::AsyncSpinner spinner(4); // 使用4個線程的異步轉輪
  spinner.start();
  ros::waitForShutdown(); // 等待程序被手動終止
  
  return 0;
}
