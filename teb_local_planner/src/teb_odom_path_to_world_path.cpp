#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <campusrover_msgs/PathArray.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>

ros::Publisher teb_path_world_pub;

geometry_msgs::PoseStamped robot_tf_poseStamped_;

std::string target_frame = "world";  
int mpc_turn_on_count = 0 ;
bool mpc_turn_on_flag = false ;

bool start_lock_flag = false;

geometry_msgs::PoseStamped last_teb_path_pose;
bool last_teb_pose_valid = false;  // 用來確認是否已經更新過

double calculateYaw(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2) 
{
    return atan2((p2.position.y - p1.position.y) , (p2.position.x - p1.position.x)); 
}

double calculateDistance(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) 
{
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    return sqrt(dx * dx + dy * dy);
}

void SamePoseFilter(const nav_msgs::Path& input_path, double distance_threshold, nav_msgs::Path& output_path) 
{
    output_path.header = input_path.header;

    // 清空輸出路徑的點
    output_path.poses.clear();

    for (const auto& pose : input_path.poses) 
    {
        bool is_duplicate = false;

        // 檢查是否與 output_path 最後一個點過近
        if (!output_path.poses.empty()) 
        {
            const auto& last_pose = output_path.poses.back();
            if (calculateDistance(last_pose.pose, pose.pose) <= distance_threshold)
            {
                is_duplicate = true;

                // 如果是重複點，替換成後面的點
                output_path.poses.back() = pose;
            }
        }

        // 如果不是重複點，則直接加入
        if (!is_duplicate) 
        {
            output_path.poses.push_back(pose);
        }
    }
}

void UpdateOrientationFilter(const nav_msgs::Path& same_pose_filter_path, nav_msgs::Path& update_orientation_filter_path) 
{
    if (same_pose_filter_path.poses.empty()) 
        return;

    // 初始化輸出路徑
    update_orientation_filter_path.header = same_pose_filter_path.header;
    update_orientation_filter_path.poses.clear();

    // 計算第一個點的角度
    if (same_pose_filter_path.poses.size() == 1)
    {
        // 如果只有一個點，直接加入並返回
        update_orientation_filter_path.poses.push_back(same_pose_filter_path.poses.front());
        return;
    }

    // 計算第一個點的角度（根據第一點與第二點之間的方向）
    double orientation0 = calculateYaw(same_pose_filter_path.poses[0].pose, same_pose_filter_path.poses[1].pose);
    geometry_msgs::PoseStamped first_pose = same_pose_filter_path.poses.front();
    {
        tf2::Quaternion q;
        q.setRPY(0, 0, orientation0);
        first_pose.pose.orientation = tf2::toMsg(q);
    }
    update_orientation_filter_path.poses.push_back(first_pose);

    // 更新中間點的朝向
    for (size_t i = 1; i < same_pose_filter_path.poses.size() - 1; ++i) 
    {
        double orientation_i = calculateYaw(same_pose_filter_path.poses[i].pose, same_pose_filter_path.poses[i + 1].pose);
        geometry_msgs::PoseStamped modified_pose = same_pose_filter_path.poses[i];
        {
            tf2::Quaternion q;
            q.setRPY(0, 0, orientation_i);
            modified_pose.pose.orientation = tf2::toMsg(q);
        }
        update_orientation_filter_path.poses.push_back(modified_pose);
    }

    // 計算最後一個點的方向，並記錄其索引
    geometry_msgs::PoseStamped end_pose = same_pose_filter_path.poses.back();
    update_orientation_filter_path.poses.push_back(end_pose);
}

void StartLockCallback(const std_msgs::Bool &msg) 
{
    if (msg.data)
    {
        start_lock_flag = true ;
    }
    else
    {
        start_lock_flag = false ;
    }
}

void MPCTurnOnCallback(const std_msgs::Bool &flag) //接收mpc start了沒
{
    if (flag.data)
    {
        mpc_turn_on_flag = true ;
        mpc_turn_on_count++ ; //第一次變true發布 其餘不發布
        ROS_INFO("[TebPathCallback] MPC is turned on. Count: %d", mpc_turn_on_count);
    }
    else
    {
        mpc_turn_on_flag = false ;
        mpc_turn_on_count = 0 ;
        start_lock_flag = false;
        last_teb_pose_valid = false;
        // ROS_INFO("[mpc_navigation_node] MPC is turned off.");
    }
}

void TebPathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    // 建立靜態的 TF Buffer 與 TransformListener（只初始化一次）
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);

    if (!msg->poses.empty())
    {
        last_teb_path_pose = msg->poses.back(); //world
        last_teb_pose_valid = true;
    }

    if(!start_lock_flag)
    {
        nav_msgs::Path world_origin_teb_path, same_pose_filter_path, update_orientation_filter_path, world_filtered_teb_path;             // 儲存轉換後的 path
        geometry_msgs::PoseStamped pose_out;

        // 如果 path 為空就直接返回
        if (msg->poses.empty()) 
        {
            return;
        }

        // 準備要發布的 teb_path，其 header 設定為 target_frame 及當前時間

        ROS_INFO("[TebPathCallback] Transforming teb path from origin frame_id (%s) to target frame_id (%s)",
                msg->header.frame_id.c_str(), target_frame.c_str());

        // 嘗試取得從原始 frame 到 target frame 的轉換
        geometry_msgs::TransformStamped tf_odom_to_world;
        try
        {
            tf_odom_to_world = tfBuffer.lookupTransform(target_frame, msg->header.frame_id, ros::Time(0), ros::Duration(2.0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("[TebPathCallback] teb path Transform warning: %s", ex.what());
            return;
        }

        // 對每個點做轉換
        for (size_t i = 0; i < msg->poses.size(); i++)
        {
            try
            {
                tf2::doTransform(msg->poses[i], pose_out, tf_odom_to_world);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_ERROR("[TebPathCallback] teb path do transform: %s", ex.what());
                ros::Duration(0.1).sleep();
                return;
            }
            world_origin_teb_path.poses.push_back(pose_out);
        }

        // SamePoseFilter
        double distance_threshold = 0.05; // 5 cm threshold
        SamePoseFilter(world_origin_teb_path, distance_threshold, same_pose_filter_path); 
        // same_pose_filter_path_pub.publish(same_pose_filter_path);
        ROS_INFO("[TebPathCallback] After SamePoseFilter Global path contains %lu poses.", same_pose_filter_path.poses.size());

        // UpdateOrientationFilter
        //有用use orientation的global path經過SamePoseFilter後角度就是對的了 可以不用過updateorientationfilter
        ROS_INFO("[TebPathCallback] UpdateOrientationFilter ");
        UpdateOrientationFilter(same_pose_filter_path, update_orientation_filter_path);
        // update_orientation_filter_path_pub.publish(update_orientation_filter_path);

        world_filtered_teb_path.poses.clear();
        world_filtered_teb_path = update_orientation_filter_path;

        // 設置 target_path 的 header
        world_filtered_teb_path.header.frame_id = target_frame;  // 替換為正確的座標框架名稱
        world_filtered_teb_path.header.stamp = ros::Time::now();

        for (size_t i = 0; i < world_filtered_teb_path.poses.size(); i++) 
        {
            world_filtered_teb_path.poses[i].header.frame_id = world_filtered_teb_path.header.frame_id;
            world_filtered_teb_path.poses[i].header.stamp = ros::Time::now();             
        }

        while (!(mpc_turn_on_flag)) 
        {
            ROS_INFO("[TebPathCallback] mpc_turn_on_flag waiting 0.1 sec");
            ros::Duration(0.1).sleep();
        }

        if (mpc_turn_on_count >= 1) //第一次變true發布 其餘不發布
        {
            teb_path_world_pub.publish(world_filtered_teb_path);
        }

        ROS_INFO("teb path has %lu poses", world_filtered_teb_path.poses.size());
    }

}

void TebContainerCallback(const campusrover_msgs::PathArray::ConstPtr& msg)
{
    // 建立靜態的 TF Buffer 與 TransformListener（只初始化一次）
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);

    if(start_lock_flag)
    {
        nav_msgs::Path world_origin_teb_path, same_pose_filter_path, update_orientation_filter_path, world_filtered_teb_path;             // 儲存轉換後的 path
        geometry_msgs::PoseStamped pose_out;

        // 如果 path 為空就直接返回
        if (msg->path.size() == 0) 
        {
            return;
        }

        int best_index = -1;
        double min_distance = std::numeric_limits<double>::max();

        for (size_t i = 0; i < msg->path.size(); i++)
        {
            if (msg->path[i].poses.empty())
            {
                continue;
            }
            double path_min_distance = std::numeric_limits<double>::max();
            // 遍歷每個點，找出該路徑上與機器人最近的點
            for (size_t j = 0; j < msg->path[i].poses.size(); j++)
            {
                double dx = msg->path[i].poses[j].pose.position.x - robot_tf_poseStamped_.pose.position.x;
                double dy = msg->path[i].poses[j].pose.position.y - robot_tf_poseStamped_.pose.position.y;
                double distance = std::sqrt(dx*dx + dy*dy);
                if(distance < path_min_distance)
                {
                    path_min_distance = distance;
                }
            }
            
            if(path_min_distance < min_distance)
            {
                min_distance = path_min_distance;
                best_index = i;
            }
        }

        if(best_index == -1)
        {
            ROS_WARN("[TebContainerCallback] No valid path found near current position.");
            return;
        }
        
        // 使用挑選到的路徑進行後續處理
        nav_msgs::Path selected_path = msg->path[best_index];

        if (!selected_path.poses.empty() && last_teb_pose_valid)
        {
            selected_path.poses.back().pose.orientation = last_teb_path_pose.pose.orientation;
            // ROS_INFO("[TebContainerCallback] Replaced last point orientation with TebPathCallback's last point orientation.");
        }

        // 準備要發布的 teb_path，其 header 設定為 target_frame 及當前時間

        ROS_INFO("[TebContainerCallback] Transforming teb path from origin frame_id (%s) to target frame_id (%s)",
                msg->path[0].header.frame_id.c_str(), target_frame.c_str());

        // 嘗試取得從原始 frame 到 target frame 的轉換
        geometry_msgs::TransformStamped tf_odom_to_world;
        try
        {
            tf_odom_to_world = tfBuffer.lookupTransform(target_frame, msg->path[0].header.frame_id, ros::Time(0), ros::Duration(2.0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR("[TebContainerCallback] teb path Transform warning: %s", ex.what());
            return;
        }

        // 對 selected_path 中的每個點做轉換
        for (size_t i = 0; i < selected_path.poses.size(); i++)
        {
            try
            {
                tf2::doTransform(selected_path.poses[i], pose_out, tf_odom_to_world);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_ERROR("[TebContainerCallback] teb path do transform: %s", ex.what());
                ros::Duration(0.1).sleep();
                return;
            }
            world_origin_teb_path.poses.push_back(pose_out);
        }

        // SamePoseFilter
        double distance_threshold = 0.05; // 5 cm threshold
        SamePoseFilter(world_origin_teb_path, distance_threshold, same_pose_filter_path); 
        // same_pose_filter_path_pub.publish(same_pose_filter_path);
        ROS_INFO("[TebContainerCallback] After SamePoseFilter Global path contains %lu poses.", same_pose_filter_path.poses.size());

        // UpdateOrientationFilter
        //有用use orientation的global path經過SamePoseFilter後角度就是對的了 可以不用過updateorientationfilter
        ROS_INFO("[TebContainerCallback] UpdateOrientationFilter ");
        UpdateOrientationFilter(same_pose_filter_path, update_orientation_filter_path);
        // update_orientation_filter_path_pub.publish(update_orientation_filter_path);

        world_filtered_teb_path.poses.clear();
        world_filtered_teb_path = update_orientation_filter_path;

        // 設置 target_path 的 header
        world_filtered_teb_path.header.frame_id = target_frame;  // 替換為正確的座標框架名稱
        world_filtered_teb_path.header.stamp = ros::Time::now();

        for (size_t i = 0; i < world_filtered_teb_path.poses.size(); i++) 
        {
            world_filtered_teb_path.poses[i].header.frame_id = world_filtered_teb_path.header.frame_id;
            world_filtered_teb_path.poses[i].header.stamp = ros::Time::now();             
        }

        while (!(mpc_turn_on_flag)) 
        {
            ROS_INFO("[TebContainerCallback] mpc_turn_on_flag waiting 0.1 sec");
            ros::Duration(0.1).sleep();
        }

        if (mpc_turn_on_count >= 1) //第一次變true發布 其餘不發布
        {
            teb_path_world_pub.publish(world_filtered_teb_path);
        }

        ROS_INFO("teb path has %lu poses", world_filtered_teb_path.poses.size());
    }

}

void CurrentPoseTimer(const ros::TimerEvent &event)
{
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);
    static geometry_msgs::TransformStamped transformStamped;

    try
    {
        transformStamped = tfBuffer.lookupTransform(target_frame, "base_link", ros::Time(0), ros::Duration(2.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN(" %s. Can't update pose from TF, for that will be use the latest source point.", ex.what());
    }

    robot_tf_poseStamped_.header.frame_id = target_frame;
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
    ros::init(argc, argv, "teb_odom_path_to_world_path");
    ros::NodeHandle nh;

    ros::Subscriber start_lock_sub = nh.subscribe("/start_lock", 10, StartLockCallback);
    ros::Subscriber teb_container_sub = nh.subscribe("/test_optim_node/teb_container", 10, TebContainerCallback);
    ros::Subscriber teb_path_sub = nh.subscribe("/test_optim_node/local_plan", 10, TebPathCallback);
    ros::Subscriber mpc_turn_on_sub = nh.subscribe("/teb_mpc_turn_on", 10, MPCTurnOnCallback);

    teb_path_world_pub = nh.advertise<nav_msgs::Path>("/teb_path_world", 10);

    ros::Timer current_pose_timer = nh.createTimer(ros::Duration(0.1), CurrentPoseTimer);

    ros::AsyncSpinner spinner(4); // 使用4個線程的異步轉輪
    spinner.start();
    ros::waitForShutdown(); // 等待程序被手動終止

    return 0;
}
