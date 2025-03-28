#define NEW_PATH 1
#define ARRIVED_PATH 2

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Bool.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <campusrover_msgs/TEBVias.h> //srv
#include <campusrover_msgs/TEBStartEnd.h> //srv
#include <campusrover_msgs/TEBStartEndViasMsg.h> //msg
#include <cmath>
#include <limits>
#include <vector>

// Global variables
ros::ServiceClient via_client, start_end_client;
ros::Publisher teb_start_end_vias_pub, global_path_end_pub, start_lock_pub;

std::string child_frame = "base_link";
std::string target_frame = "world";

double car_radius = 0.35;
double extension_step = 0.05;
double max_extension = 8.0; // 上限，例如8.0m

double dis_threshold = 0.05;
double total_teb_filter_path_length_threshold = 0.2 ;
double local_start_threshold = 2.0 ;
double candidate_local_end_threshold = 8.0;

int patience = 3;

geometry_msgs::PoseStamped robot_tf_poseStamped_, initial_robot_poseStamped;
nav_msgs::Path teb_filter_path;
double total_teb_filter_path_length;
int total_teb_index;

int path_mode = ARRIVED_PATH ;

costmap_converter::ObstacleArrayMsg obstacles_;

int patience_front = 0, patience_back = 0, patience_left = 0, patience_right = 0;
bool local_start_locked = false;
bool local_end_locked = false;
geometry_msgs::PoseStamped locked_local_start;
geometry_msgs::PoseStamped locked_local_end;
int via_start_idx = -1;
int via_end_idx = -1;
/*---------------------------
  以下為障礙物檢查與調整功能
---------------------------*/

void get_parameters(ros::NodeHandle n_private)
{
    n_private.param<std::string>("target_frame", target_frame, "world");
    n_private.param<std::string>("child_frame", child_frame, "base_link");
    n_private.param<double>("car_radius", car_radius, 0.35);
    n_private.param<double>("max_extension", max_extension, 8.0);
    n_private.param<double>("extension_step", extension_step, 0.05);
    n_private.param<double>("dis_threshold", dis_threshold, 0.05);
    n_private.param<double>("total_teb_filter_path_length_threshold", total_teb_filter_path_length_threshold, 0.2);
    n_private.param<double>("local_start_threshold", local_start_threshold, 2.0);
    n_private.param<double>("candidate_local_end_threshold", candidate_local_end_threshold, 8.0);
    n_private.param<int>("patience", patience, 3);
}

double calculateDistance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    return std::sqrt(dx*dx + dy*dy);
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
            if (calculateDistance(last_pose, pose) <= distance_threshold)
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

// 檢查給定 candidate_pose 位置是否與最近障礙物重疊
// 條件：若 candidate_pose 與最近障礙物中心的距離 <= (obstacle.radius + car_radius)，則視為被障礙物阻擋
bool checkObstacle(const geometry_msgs::PoseStamped &candidate_pose)
{
    if (obstacles_.obstacles.empty())
    {
        return false;
    }

    double min_obs_dist = std::numeric_limits<double>::max();
    int nearest_obs_idx = -1;
    // 假設 obstacles_.obstacles 為一個障礙物列表，每個障礙物的 polygon.points[0] 為障礙物中心
    for (size_t i = 0; i < obstacles_.obstacles.size(); ++i)
    {
        const geometry_msgs::Point32 &obs_center = obstacles_.obstacles[i].polygon.points[0];
        double dx = candidate_pose.pose.position.x - obs_center.x;
        double dy = candidate_pose.pose.position.y - obs_center.y;
        double d = std::sqrt(dx * dx + dy * dy);
        if (d < min_obs_dist)
        {
            min_obs_dist = d;
            nearest_obs_idx = i;
        }
    }
    if (nearest_obs_idx < 0)
    {
        return false;
    }

    double req_dist = obstacles_.obstacles[nearest_obs_idx].radius + car_radius;
    return (min_obs_dist <= req_dist);
}

// 輔助函式：沿著給定方向延伸 candidate_pose 點，直到該點與 obs_center 距離大於 (obs_radius + car_radius)
// 若延伸超過 max_extension (例如 2.0m) 仍未滿足，則回傳 false
bool extendCandidate(const geometry_msgs::PoseStamped &candidate_pose, const geometry_msgs::Vector3 &direction,
                    double max_extension, geometry_msgs::PoseStamped &result_pose)
{
    double accum = 0.0;
    result_pose = candidate_pose;

    while (accum < max_extension)
    {
        // 沿指定方向延伸
        result_pose.pose.position.x += direction.x * extension_step;
        result_pose.pose.position.y += direction.y * extension_step;
        accum += extension_step;
        
        // 重新計算延伸後這個點與所有障礙物的距離，找出最近的障礙物
        double min_dist = std::numeric_limits<double>::max();
        double current_obs_radius = 0.0;
        bool foundObstacle = false;
        for (const auto &obs : obstacles_.obstacles)
        {
            const geometry_msgs::Point32 &obs_center = obs.polygon.points[0];
            double dx = result_pose.pose.position.x - obs_center.x;
            double dy = result_pose.pose.position.y - obs_center.y;
            double d = std::sqrt(dx * dx + dy * dy);
            if (d < min_dist)
            {
                min_dist = d;
                current_obs_radius = obs.radius;
                foundObstacle = true;
            }
        }
        
        // 如果有障礙物，判斷候選點是否滿足安全距離
        if (foundObstacle)
        {
            if (min_dist > (current_obs_radius + car_radius))
            {
                return true;  // 此位置安全
            }
        }
        else
        {
            // 沒有障礙物，直接認定為安全
            return true;
        }
    }

    return false; // 延伸超過最大距離仍不滿足
}

// 根據 front 與 back 計算 4 個方向的 candidate 點，並延伸至無障礙區域
// 最後選擇與 robot_tf_poseStamped_ 直線距離最短的 candidate 作為調整後的位置
geometry_msgs::PoseStamped adjustCandidate(const geometry_msgs::PoseStamped &front, const geometry_msgs::PoseStamped &back, 
                                            const std::string &mode, std::string &patience_orientation)
{
    geometry_msgs::PoseStamped adjusted_pose;

    if (mode == "front_mode")
    {
        adjusted_pose = front;
    }
    else if (mode == "back_mode")
    {
        adjusted_pose = back;
    }

    if (obstacles_.obstacles.empty())
    {
        patience_orientation = "all";
        return adjusted_pose;
    }

    // 利用 front 與 back 的座標差，得到從 back 指向 front 的向量 (vx, vy)。
    double vx = front.pose.position.x - back.pose.position.x;
    double vy = front.pose.position.y - back.pose.position.y;
    double norm = std::sqrt(vx * vx + vy * vy);
    if (norm == 0)
    {
        norm = 1.0;
    }

    //將這個向量正規化（除以其長度），得到單位向量，代表主要行進方向。
    vx /= norm;
    vy /= norm;
    // 垂直方向 (逆時針90度)
    double px = -vy;
    double py = vx;

    // 定義四個方向：前、後、左、右
    std::vector<std::pair<geometry_msgs::Vector3, std::string>> directions;
    
    geometry_msgs::Vector3 dir_front; 
    dir_front.x = vx; 
    dir_front.y = vy; 
    dir_front.z = 0;
    directions.push_back(std::make_pair(dir_front, "front"));
    
    geometry_msgs::Vector3 dir_back; 
    dir_back.x = -vx; 
    dir_back.y = -vy; 
    dir_back.z = 0;
    directions.push_back(std::make_pair(dir_back, "back"));
    
    geometry_msgs::Vector3 dir_left; 
    dir_left.x = px; 
    dir_left.y = py; 
    dir_left.z = 0;
    directions.push_back(std::make_pair(dir_left, "left"));
    
    geometry_msgs::Vector3 dir_right; 
    dir_right.x = -px; 
    dir_right.y = -py; 
    dir_right.z = 0;
    directions.push_back(std::make_pair(dir_right, "right"));

    // 從 adjusted_pose 開始, 以固定的 step（例如 0.05m）逐步延伸
    double extension = car_radius;
    std::vector<std::pair<geometry_msgs::PoseStamped, std::string>> candidates;

    // 嘗試延伸直到至少有一個方向回傳 ok = true，或延伸距離超過上限
    while (candidates.empty() && extension <= max_extension)
    {
        candidates.clear();
        for (size_t i = 0; i < directions.size(); ++i)
        {
            geometry_msgs::PoseStamped candidate = adjusted_pose;
            geometry_msgs::PoseStamped newCandidate;
            bool ok = extendCandidate(candidate, directions[i].first, extension, newCandidate);
            if (ok)
            {
                candidates.push_back(std::make_pair(newCandidate, directions[i].second));
            }
        }
        if (candidates.empty())
        {
            extension += extension_step;
        }
    }

    // 如果都找不到則回傳 front，並設空方向
    if (candidates.empty())
    {
        patience_orientation = "no";
        return adjusted_pose;
    }

    // 從有效候選中選出與 robot_tf_poseStamped_ 直線距離最短的
    double best_dist = std::numeric_limits<double>::max();
    geometry_msgs::PoseStamped best_candidate;
    std::string best_orientation = "";
    for (auto &item : candidates)
    {
        double dx = item.first.pose.position.x - robot_tf_poseStamped_.pose.position.x;
        double dy = item.first.pose.position.y - robot_tf_poseStamped_.pose.position.y;
        double d = std::sqrt(dx * dx + dy * dy);
        if (d < best_dist)
        {
            best_dist = d;
            best_candidate = item.first;
            best_orientation = item.second;
        }
    }
    patience_orientation = best_orientation;
    return best_candidate;
}

/*---------------------------
  End of 障礙物檢查與調整功能
---------------------------*/

void InitialCallback(const std_msgs::Bool &flag)
{
  if (flag.data)
  {
    path_mode = ARRIVED_PATH ;

    patience_front = 0;
    patience_back = 0;
    patience_left = 0;
    patience_right = 0;

    local_start_locked = false;
    local_end_locked = false;
    via_start_idx = -1;
    via_end_idx = -1;

    locked_local_start.header.seq = 0;
    locked_local_start.header.frame_id = "";
    locked_local_start.header.stamp = ros::Time::now();
    locked_local_start.pose.position.x = 0.0;
    locked_local_start.pose.position.y = 0.0;
    locked_local_start.pose.position.z = 0.0;
    locked_local_start.pose.orientation.x = 0.0;
    locked_local_start.pose.orientation.y = 0.0;
    locked_local_start.pose.orientation.z = 0.0;
    locked_local_start.pose.orientation.w = 0.0;

    locked_local_end.header.seq = 0;
    locked_local_end.header.frame_id = "";
    locked_local_end.header.stamp = ros::Time::now();
    locked_local_end.pose.position.x = 0.0;
    locked_local_end.pose.position.y = 0.0;
    locked_local_end.pose.position.z = 0.0;
    locked_local_end.pose.orientation.x = 0.0;
    locked_local_end.pose.orientation.y = 0.0;
    locked_local_end.pose.orientation.z = 0.0;
    locked_local_end.pose.orientation.w = 0.0;
    
  }
}

void HallwayObstacleCallback(const costmap_converter::ObstacleArrayMsgConstPtr &obs)
{
    obstacles_ = *obs;
    // ROS_INFO("[teb_start_end_via] Received %lu  Obstacle.", obstacles_.obstacles.size());
}

void GlobalPathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    if (msg->poses.empty())
    {
        return;
    }

    static geometry_msgs::Pose last_pose;

    // 更新初始機器人位置
    initial_robot_poseStamped = robot_tf_poseStamped_;
    initial_robot_poseStamped.header.stamp = ros::Time::now();

    // 找出 msg 中與 initial_robot_poseStamped 最近的點及其 index
    int nearest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < msg->poses.size(); ++i)
    {
        const geometry_msgs::Point &pt = msg->poses[i].pose.position;
        double dx = pt.x - initial_robot_poseStamped.pose.position.x;
        double dy = pt.y - initial_robot_poseStamped.pose.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < min_dist)
        {
            min_dist = dist;
            nearest_idx = i;
        }
    }

    // 重新建立 teb_global_path：先加入當前位置，再從最近點開始到原 global_path 的最終點
    nav_msgs::Path teb_global_path ;

    // 加入起始點 (初始機器人位置)
    teb_global_path.poses.push_back(initial_robot_poseStamped);

    // 從原來 global path 中最近的點開始，依序加入所有點, 但要確保 teb_global_path 中相鄰的點距離均大於 0.05。
    for (size_t i = nearest_idx; i < msg->poses.size() - 1; ++i)
    {
        // 計算當前點與 teb_global_path 中最後一個點的距離
        double dx = msg->poses[i].pose.position.x - teb_global_path.poses.back().pose.position.x;
        double dy = msg->poses[i].pose.position.y - teb_global_path.poses.back().pose.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        // 如果距離大於 dis_threshold(0.05) 才加入 teb_global_path
        if (dist >= dis_threshold)
        {
            teb_global_path.poses.push_back(msg->poses[i]);
        }
    }

    // 加入路徑最終點
    teb_global_path.poses.push_back(msg->poses.back());

    // SamePoseFilter
    teb_filter_path.header = msg->header;
    teb_filter_path.header.stamp = ros::Time::now();
    teb_filter_path.poses.clear();    

    // dis_threshold =  5 cm
    SamePoseFilter(teb_global_path, dis_threshold, teb_filter_path); 

    // 計算路徑總長度
    double total_length = 0.0;
    if (teb_filter_path.poses.size() >= 2)
    {
        // 第一段：從當前位置到 teb_filter_path 中第一個點（即原 global path 中最近點）
        const geometry_msgs::Point &p0 = teb_filter_path.poses[0].pose.position;
        const geometry_msgs::Point &p1 = teb_filter_path.poses[1].pose.position;
        total_length += std::sqrt(std::pow(p1.x - p0.x, 2) + std::pow(p1.y - p0.y, 2));

        // 後續段落：沿 teb_filter_path 計算相鄰點間的距離
        for (size_t i = 1; i < teb_filter_path.poses.size() - 1; ++i)
        {
            const geometry_msgs::Point &pt1 = teb_filter_path.poses[i].pose.position;
            const geometry_msgs::Point &pt2 = teb_filter_path.poses[i + 1].pose.position;
            total_length += std::sqrt(std::pow(pt2.x - pt1.x, 2) + std::pow(pt2.y - pt1.y, 2));
        }
    }
    else
    {
        total_length = 0.0 ;
    }

    const auto& last_teb_filter_pose = teb_filter_path.poses.back().pose;
    if ( abs(last_teb_filter_pose.position.x - last_pose.position.x) > 0.0001|| abs(last_teb_filter_pose.position.y - last_pose.position.y) > 0.0001||
        abs(last_teb_filter_pose.orientation.z - last_pose.orientation.z ) > 0.0001|| abs(last_teb_filter_pose.orientation.w - last_pose.orientation.w) > 0.0001 )
    {
        path_mode = NEW_PATH ;
        last_pose = last_teb_filter_pose;
    }
    // 計算總的 index 數量：包含起始點及從最近點開始到最後的所有點
    int index_count = teb_filter_path.poses.size();

    total_teb_filter_path_length = total_length;
    total_teb_index = index_count;
    ROS_INFO("TEB Path total length: %.3f", total_teb_filter_path_length);
    ROS_INFO("TEB Path total index count: %d", total_teb_index);
}

void StartEndViaTimer(const ros::TimerEvent &event)
{
    // 若 total_teb_filter_path_length 長度(<0.2)或 total_teb_index 數量不足，直接返回    
    if (total_teb_filter_path_length < total_teb_filter_path_length_threshold || total_teb_index < 2 || path_mode == ARRIVED_PATH) 
    {
        return;
    }

    geometry_msgs::PoseStamped local_start_pose, local_end_pose;

    // -------------------------------
    // 算 當前到最近點 + 最近點index到最近點index+1 + ... + 路徑最後一點
    // 這邊不用管<0.05的事
    // -------------------------------

    // 找出 teb_filter_path 上與 robot_tf_poseStamped_ 最近的點（nearest_idx）
    int nearest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < total_teb_index; ++i)
    {
        const geometry_msgs::Point &pt = teb_filter_path.poses[i].pose.position;
        double dx = pt.x - robot_tf_poseStamped_.pose.position.x;
        double dy = pt.y - robot_tf_poseStamped_.pose.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist < min_dist) //這邊nearest_idx有可能會找到自己
        {
            min_dist = dist;
            nearest_idx = i;
        }
    }

    // 計算從 robot_tf_poseStamped_ 到 teb_filter_path_end 的累積距離，
    // 這裡分為：
    // d1 = robot_tf_poseStamped_ 到 teb_filter_path.poses[nearest_idx]；
    // d2 = 累計從 nearest_idx 到 teb_filter_path_end

    double d1 = min_dist;
    double d2 = 0.0;
    for (size_t i = nearest_idx; i < total_teb_index - 1; ++i)
    {
        const geometry_msgs::Point &p1 = teb_filter_path.poses[i].pose.position;
        const geometry_msgs::Point &p2 = teb_filter_path.poses[i + 1].pose.position;
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        d2 += std::sqrt(dx * dx + dy * dy);
    }

    double total_dist_current_to_end = d1 + d2; //快到終點前用的

    // -------------------------------
    // 1. 決定 local_start
    // -------------------------------
    // 情況1：從TebGlobalPathCallback 得到的 total_teb_filter_path_length <= 2.0, > 0.2
    if (total_teb_filter_path_length <= local_start_threshold && total_dist_current_to_end <= local_start_threshold)
    {
        if (!local_start_locked) //開使鎖
        {
            locked_local_start = initial_robot_poseStamped;
            via_start_idx = nearest_idx;
            local_start_locked = true;
        }

        // 如果locked_local_start有障礙物，直接4方位 找沒有障礙物的位置
        if (checkObstacle(locked_local_start))
        {
            // ROS_INFO("local_start 1. (%.2f, %.2f) has obstacle ", locked_local_start.pose.position.x, locked_local_start.pose.position.y);
            
            int start_next_idx = via_start_idx;

            //只有2顆也沒關係 就是終點 dis_threshold=0.05
            while (calculateDistance(teb_filter_path.poses[start_next_idx], locked_local_start) <= dis_threshold)
            {
                start_next_idx++;
            }
            geometry_msgs::PoseStamped start_next_pose = teb_filter_path.poses[start_next_idx];
            std::string selected_orientation; //跟後面用同一個function 但這裡沒用到 反正就給它一直變不影響
            locked_local_start = adjustCandidate(start_next_pose, locked_local_start, "back_mode", selected_orientation);
            
            // ROS_INFO("local_start 1. change to (%.2f, %.2f) ", locked_local_start.pose.position.x, locked_local_start.pose.position.y);
            
        }
    }
    // 情況2：total_teb_filter_path_length > 2.0 ，但從 robot_tf_poseStamped_ 到 teb_filter_path_end 的累加距離 <= 2.0m (快到終點) (應該至少有個15個點)
    else if (total_dist_current_to_end <= local_start_threshold)
    {
        if (!local_start_locked)
        {
            locked_local_start = robot_tf_poseStamped_;
            via_start_idx = nearest_idx;  // via_point可以是start或end 所以沒問題
            local_start_locked = true;
        }

        // 如果locked_local_start有障礙物，直接4方位 找沒有障礙物的位置
        if (checkObstacle(locked_local_start))
        {
            // ROS_INFO("local_start 2. (%.2f, %.2f) has obstacle ", locked_local_start.pose.position.x, locked_local_start.pose.position.y);
            int start_next_idx = via_start_idx;
            // dis_threshold = 0.05
            while (calculateDistance(teb_filter_path.poses[start_next_idx], locked_local_start) <= dis_threshold)
            {
                start_next_idx++;
            }
            geometry_msgs::PoseStamped start_next_pose = teb_filter_path.poses[start_next_idx];
            std::string selected_orientation; //跟後面用同一個function 但這裡沒用到 反正就給它一直變不影響
            locked_local_start = adjustCandidate(start_next_pose, locked_local_start, "back_mode", selected_orientation);

            // ROS_INFO("local_start 2. change to (%.2f, %.2f) ", locked_local_start.pose.position.x, locked_local_start.pose.position.y);

        }
    }
    // 情況3：total_teb_filter_path_length > 2.0 ，且從 robot_tf_poseStamped_ 到 teb_filter_path_end 的累加距離 > 2.0m 
    // 直接使用 robot_tf_poseStamped_ 作為 locked_local_start
    else
    {
        locked_local_start = robot_tf_poseStamped_;
        via_start_idx = nearest_idx; // via_point可以是start或end 所以沒問題
        local_start_locked = false;

        if (checkObstacle(locked_local_start))
        {
            // ROS_INFO("local_start 3. (%.2f, %.2f) has obstacle ", locked_local_start.pose.position.x, locked_local_start.pose.position.y);
            int start_next_idx = via_start_idx;
            // dis_threshold = 0.05
            while (calculateDistance(teb_filter_path.poses[start_next_idx], locked_local_start) <= dis_threshold)
            {
                start_next_idx++;
            }
            geometry_msgs::PoseStamped start_next_pose = teb_filter_path.poses[start_next_idx];
            std::string selected_orientation; //跟後面用同一個function 但這裡沒用到 反正就給它一直變不影響
            locked_local_start = adjustCandidate(start_next_pose, locked_local_start, "back_mode", selected_orientation);

            // ROS_INFO("local_start 3. change to (%.2f, %.2f) ", locked_local_start.pose.position.x, locked_local_start.pose.position.y);
        }
    }

    local_start_pose = locked_local_start;

    // -------------------------------
    // 2. 決定 candidate local_end
    // -------------------------------
    geometry_msgs::PoseStamped teb_filter_path_end = teb_filter_path.poses.back();
    
    geometry_msgs::PoseStamped candidate_local_end, adj_candidate_local_end;
    int candidate_idx = 0;

    // 若 total_teb_filter_path_length 長度 <=8m, 或 total_dist_current_to_end <= 8.0, 
    // 候選 local_end 為 teb_filter_path 的最後一點
    if (total_teb_filter_path_length <= candidate_local_end_threshold || total_dist_current_to_end <= candidate_local_end_threshold)
    {
        candidate_local_end = teb_filter_path_end;
        candidate_idx = total_teb_index - 1; //最後一點
        via_end_idx = candidate_idx;
    }
    // else , 
    // 先計算 robot_tf_poseStamped_到 teb_filter_path.poses[via_start_idx]的 距離 然後
    // 從 via_start_idx 開始累計，直到累積距離 >=8.0，取該點的index 為candidate_idx, 該點的index-1 為via_end_idx
    else
    {
        double accum = 0.0;
        accum += calculateDistance(teb_filter_path.poses[via_start_idx], robot_tf_poseStamped_);
        
        // 判斷robot_tf_poseStamped_到 teb_filter_path.poses[via_start_idx]的距離有沒有超過8m
        if (accum >= candidate_local_end_threshold) //這裡不用擔心via_start_idx的問題 因為如果成立了 那當前機器人位置會離原始的global path很遠了 不會是當前的車的位置
        {
            candidate_idx = via_start_idx;
            via_end_idx = candidate_idx;
        }
        else
        {
            for (size_t i = via_start_idx; i < total_teb_index - 1; ++i)
            {
                const geometry_msgs::Point &p1 = teb_filter_path.poses[i].pose.position;
                const geometry_msgs::Point &p2 = teb_filter_path.poses[i + 1].pose.position;
                double dx = p2.x - p1.x;
                double dy = p2.y - p1.y;
                accum += std::sqrt(dx * dx + dy * dy);
                if (accum >= candidate_local_end_threshold)
                {
                    candidate_idx = i;
                    via_end_idx = candidate_idx ;
                    break;
                }
            }

            // 如果累積距離仍然小於8.0，則選擇最後一個點作為 candidate
            if (accum < candidate_local_end_threshold)
            {
                candidate_idx = total_teb_index - 1;
                via_end_idx = candidate_idx;
            }
        }
        
        candidate_local_end = teb_filter_path.poses[candidate_idx];
    }
    adj_candidate_local_end = candidate_local_end;
    // ROS_INFO("adj_candidate_local_end (%.2f, %.2f) ", adj_candidate_local_end.pose.position.x, adj_candidate_local_end.pose.position.y);
    // ROS_INFO("teb_filter_path_end (%.2f, %.2f) ", teb_filter_path_end.pose.position.x, teb_filter_path_end.pose.position.y);
    // -------------------------------
    // 3. 調整 adj_candidate_local_end 得到真的 local_end
    // -------------------------------

    // 情況A：adj_candidate_local_end 不是 teb_filter_path_end
    if (!((std::fabs(adj_candidate_local_end.pose.position.x - teb_filter_path_end.pose.position.x) < 0.001) &&
          (std::fabs(adj_candidate_local_end.pose.position.y - teb_filter_path_end.pose.position.y) < 0.001) ))
    {
        if (checkObstacle(adj_candidate_local_end))
        {
            // ROS_INFO("local_end a. (%.2f, %.2f) has obstacle ", adj_candidate_local_end.pose.position.x, adj_candidate_local_end.pose.position.y);
            // 從候選點往後延伸，逐步找尋無障礙區間
            bool found = false;
            for (size_t i = candidate_idx + 1; i < total_teb_index; ++i)
            {
                if (!checkObstacle(teb_filter_path.poses[i]))
                {
                    adj_candidate_local_end = teb_filter_path.poses[i];
                    candidate_idx = i;
                    via_end_idx = candidate_idx;
                    found = true;
                    break;
                }
            }

            // 若延伸到最後仍有障礙，則後續將以 candidate local_end 是 teb_filter_path_end 進入條件B處理
            if (!found)
            {
                adj_candidate_local_end = teb_filter_path_end;
                candidate_idx = total_teb_index - 1;
                via_end_idx = candidate_idx;
                // ROS_INFO("local_end adjust !found extend");
            }
            // ROS_INFO("local_end a. change to (%.2f, %.2f) ", adj_candidate_local_end.pose.position.x, adj_candidate_local_end.pose.position.y);
        }
    }
    // 情況B：candidate local_end 是 teb_filter_path_end
    if ((std::fabs(adj_candidate_local_end.pose.position.x - teb_filter_path_end.pose.position.x) < 0.001) &&
        (std::fabs(adj_candidate_local_end.pose.position.y - teb_filter_path_end.pose.position.y) < 0.001) )
    {
        if (checkObstacle(adj_candidate_local_end))
        {
            // ROS_INFO("local_end b. (%.2f, %.2f) has obstacle ", adj_candidate_local_end.pose.position.x, adj_candidate_local_end.pose.position.y);
            if (total_dist_current_to_end > 2.0)
            {
                //最後一點的前一點 一定>0.05 這裡是 candidate local_end 為 teb_filter_path_end, 至少兩個點 [2-2=0]
                geometry_msgs::PoseStamped end_prev_pose = teb_filter_path.poses[total_teb_index - 2];
                std::string selected_orientation;
                adj_candidate_local_end = adjustCandidate(adj_candidate_local_end, end_prev_pose, "front_mode", selected_orientation);
                // ROS_INFO("local_end b. 1. change to (%.2f, %.2f) ", adj_candidate_local_end.pose.position.x, adj_candidate_local_end.pose.position.y);
            }
            // 距離 total_dist_current_to_end < 2.0 開始 紀錄選用的方向
            else
            {
                geometry_msgs::PoseStamped locked_local_end_prev_pose;
                if(!local_end_locked)
                {
                    //最後一點的前一點 一定>0.05 這裡是 candidate local_end 為 teb_filter_path_end, 至少兩個點 [2-2=0]
                    geometry_msgs::PoseStamped end_prev_pose = teb_filter_path.poses[total_teb_index - 2];
                    std::string selected_orientation;
                    adj_candidate_local_end = adjustCandidate(adj_candidate_local_end, end_prev_pose, "front_mode", selected_orientation);
                    
                    // 根據選用的方向更新各方向的 patience 計數
                    if (selected_orientation == "front")        {patience_front++;}
                    else if (selected_orientation == "back")    {patience_back++;}
                    else if (selected_orientation == "left")    {patience_left++;}
                    else if (selected_orientation == "right")   {patience_right++;}
                    else if (selected_orientation == "all")     {patience_front++;  patience_back++;  patience_left++;  patience_right++;}
                    else if (selected_orientation == "no")
                    {
                        patience_front = std::max(0, patience_front - 1);
                        patience_back  = std::max(0, patience_back - 1);
                        patience_left  = std::max(0, patience_left - 1);
                        patience_right = std::max(0, patience_right - 1);
                    }
                    // 如果任一方向的 patience 達到 3，則鎖定該方向（這裡示意直接保持 adj_candidate_local_end 不再更新）
                    if (patience_front >= patience || patience_back >= patience || patience_left >= patience || patience_right >= patience)
                    {
                        locked_local_end = adj_candidate_local_end;
                        
                        // 將原始位姿轉換為 tf2::Transform
                        tf2::Transform tf_end, tf_prev;
                        tf2::fromMsg(teb_filter_path_end.pose, tf_end);
                        tf2::fromMsg(end_prev_pose.pose, tf_prev);

                        // 定義 T_rel 使得： teb_filter_path_end = T_rel * end_prev_pose
                        // 則有 T_rel = teb_filter_path_end * (end_prev_pose)⁻¹
                        tf2::Transform T_rel = tf_end * tf_prev.inverse();

                        // 若想在新的 locked_local_end 下保持相同相對關係，則 new_prev_pose = T_rel⁻¹ * new_end_pose
                        tf2::Transform tf_lock_end, tf_lock_prev;
                        tf2::fromMsg(locked_local_end.pose, tf_lock_end);

                        // 注意乘法順序：tf2 中如果 A = T_rel，且有 T = A * X 則 X = A⁻¹ * T
                        tf_lock_prev = T_rel.inverse() * tf_lock_end;

                        geometry_msgs::Pose pose_msg;
                        pose_msg.position.x = tf_lock_prev.getOrigin().x();
                        pose_msg.position.y = tf_lock_prev.getOrigin().y();
                        pose_msg.position.z = tf_lock_prev.getOrigin().z();
                        pose_msg.orientation = tf2::toMsg(tf_lock_prev.getRotation());
                        locked_local_end_prev_pose.header = locked_local_end.header; // 根據需要設置 header
                        locked_local_end_prev_pose.pose = pose_msg;

                        local_end_locked = true;
                    }
                    // ROS_INFO("local_end b. 2. change to (%.2f, %.2f) ", adj_candidate_local_end.pose.position.x, adj_candidate_local_end.pose.position.y);
                }
                else
                {
                    if (checkObstacle(locked_local_end))
                    {
                        std::string selected_orientation; //跟後面用同一個function 但這裡沒用到 反正就給它一直變不影響 這邊要直接從定下來的點位再找
                        locked_local_end = adjustCandidate(locked_local_end, locked_local_end_prev_pose, "front_mode", selected_orientation);
                        // ROS_INFO("local_end b. 3. change to (%.2f, %.2f) ", locked_local_end.pose.position.x, locked_local_end.pose.position.y);
                    }
                }
            }
        }
    }

    // 最終設定 local_end_pose：若已鎖定則使用 locked_local_end，否則使用 adj_candidate_local_end
    if (local_end_locked)
    {
        local_end_pose = locked_local_end;
    }
    else
    {
        local_end_pose = adj_candidate_local_end;
    }
    // ROS_INFO("local_end (%.2f, %.2f) ", local_end_pose.pose.position.x, local_end_pose.pose.position.y);

    // 比對 candidate_local_end 與 teb_filter_path 的最後一點
    geometry_msgs::PoseStamped global_end;
    global_end.header.frame_id = target_frame;  
    global_end.header.stamp = ros::Time::now();

    // 如果candidate_local_end是teb_filter_path_end 那代表在偵測範圍內有可能做些位置修正, 
    // teb_filter_path_end 是真的發一次的global_path的最後一點
    if ((std::fabs(candidate_local_end.pose.position.x - teb_filter_path_end.pose.position.x) < 0.001) &&
        (std::fabs(candidate_local_end.pose.position.y - teb_filter_path_end.pose.position.y) < 0.001) )
    {
        global_end = local_end_pose;
    }
    //反之沒在偵測範圍內 用假的就好沒關係
    else
    {
        global_end = teb_filter_path_end;
    }
    global_path_end_pub.publish(global_end);
    ROS_INFO("global_end (%.2f, %.2f) ", global_end.pose.position.x, global_end.pose.position.y);
    
    // 4. 產生 via_points：取 local_start 到 local_end 之間的 teb_filter_path 點
    nav_msgs::Path via_points;
    via_points.header = teb_filter_path.header;
    via_points.header.stamp = ros::Time::now();
    via_points.poses.clear();

    for (int i = via_start_idx; i <= via_end_idx; ++i) 
    {
        // 將 teb_filter_path 中的點加入到 path.poses 中
        via_points.poses.push_back(teb_filter_path.poses[i]);
    }

    // 5. 呼叫服務發佈結果
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);    

    geometry_msgs::PoseStamped odom_local_start_pose, odom_local_end_pose;
    nav_msgs::Path odom_via_points;
    odom_via_points.header.frame_id = "odom";
    odom_via_points.header.stamp = ros::Time::now();

    geometry_msgs::TransformStamped tf_world_to_odom;
    try // 將 "world" 座標系轉換到 "odom" 座標系 (以 ros::Time(0) 取得最新的轉換)
    {
        tf_world_to_odom = tfBuffer.lookupTransform("odom", "world", ros::Time(0), ros::Duration(2.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("[teb_start_end_via] Transform failed: %s", ex.what());
        return;
    }

    // 將 local_start_pose (world) 轉換到 odom 座標系
    tf2::doTransform(local_start_pose, odom_local_start_pose, tf_world_to_odom);

    // 將 local_end_pose (world) 轉換到 odom 座標系
    tf2::doTransform(local_end_pose, odom_local_end_pose, tf_world_to_odom);

    // 將 via_points 中所有點從 world 座標系轉換到 odom 座標系
    for (size_t i = 0; i < via_points.poses.size(); ++i)
    {
        geometry_msgs::PoseStamped odom_pose;
        tf2::doTransform(via_points.poses[i], odom_pose, tf_world_to_odom);
        odom_via_points.poses.push_back(odom_pose);
    }

    campusrover_msgs::TEBStartEndViasMsg teb_msg;
    teb_msg.start = odom_local_start_pose;
    teb_msg.end   = odom_local_end_pose;
    teb_msg.vias  = odom_via_points;

    ROS_INFO("odom_local_start_pose (%.2f, %.2f) ", odom_local_start_pose.pose.position.x, odom_local_start_pose.pose.position.y);
    ROS_INFO("odom_local_end_pose (%.2f, %.2f) ", odom_local_end_pose.pose.position.x, odom_local_end_pose.pose.position.y);

    teb_start_end_vias_pub.publish(teb_msg);

    std_msgs::Bool start_lock_flag;
    start_lock_flag.data = local_start_locked;
    start_lock_pub.publish(start_lock_flag);
}

void CurrentPoseTimer(const ros::TimerEvent &event)
{
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);
    static geometry_msgs::TransformStamped transformStamped;
    
    try
    {
        transformStamped = tfBuffer.lookupTransform(target_frame, child_frame, ros::Time(0), ros::Duration(2.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("[teb_start_end_via] %s. Can't update pose from TF, using latest available.", ex.what());
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teb_start_end_via_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    get_parameters(nh_private);

    ros::Subscriber initial_sub = nh.subscribe("/teb_mpc_finish", 10, InitialCallback);
    ros::Subscriber teb_goal_sub = nh.subscribe("/teb_global_path", 10, GlobalPathCallback);
    ros::Subscriber hallway_world_obs_sub = nh.subscribe("/hallway_world_obs", 10, HallwayObstacleCallback);

    teb_start_end_vias_pub = nh.advertise<campusrover_msgs::TEBStartEndViasMsg>("/teb_start_end_vias", 10);
    global_path_end_pub = nh.advertise<geometry_msgs::PoseStamped>("/global_path_end", 10);
    start_lock_pub = nh.advertise<std_msgs::Bool>("/start_lock", 10);

    via_client = nh.serviceClient<campusrover_msgs::TEBVias>("/teb_via_points");
    start_end_client = nh.serviceClient<campusrover_msgs::TEBStartEnd>("/teb_start_end");

    ros::Timer current_pose_timer = nh.createTimer(ros::Duration(0.1), CurrentPoseTimer);
    ros::Timer start_end_via_timer = nh.createTimer(ros::Duration(0.05), StartEndViaTimer);

    ros::spin();
    return 0;
}
