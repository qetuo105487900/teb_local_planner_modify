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
#include <std_msgs/Bool.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <campusrover_msgs/TEBVias.h> //srv
#include <campusrover_msgs/TEBStartEnd.h> //srv
#include <campusrover_msgs/TEBStartEndViasMsg.h> //msg
#include <visualization_msgs/Marker.h>

#include <cmath>
#include <limits>
#include <vector>
#include <random>  // 新增隨機採樣所需標頭

// Global variables
ros::ServiceClient via_client, start_end_client;
ros::Publisher teb_start_end_vias_pub, global_path_end_pub, start_lock_pub, marker_pub;;

std::string child_frame = "base_link";
std::string target_frame = "world";

double car_radius = 0.35;
double max_extension = 8.0; // 上限，例如8.0m
double max_extension_ratio = 0.5 ;
int num_samples = 100;                      // 採樣候選點數量
int max_attempts = 3;                       // 最多嘗試擴大採樣的次數

double dis_threshold = 0.05;
double total_teb_filter_path_length_threshold = 0.2 ;
double local_start_threshold = 2.0 ;
double candidate_local_end_threshold = 8.0;
double lock_end_threshold = 2.0 ;

double patience = 3.0;

geometry_msgs::PoseStamped robot_tf_poseStamped_, initial_robot_poseStamped, locked_local_end_prev_pose;
nav_msgs::Path teb_filter_path;
double total_teb_filter_path_length;
int total_teb_index;

int path_mode = ARRIVED_PATH ;

costmap_converter::ObstacleArrayMsg obstacles_;

std::vector<double> local_end_patience(16, 0.0);
bool local_start_locked = false;
bool local_end_locked = false;
geometry_msgs::PoseStamped locked_local_start;
geometry_msgs::PoseStamped locked_local_end;
int via_start_idx = -1;
int via_end_idx = -1;

/*---------------------------
  以下為障礙物檢查與調整功能
---------------------------*/

// 讀取參數
void get_parameters(ros::NodeHandle n_private)
{
    n_private.param<std::string>("target_frame", target_frame, "world");
    n_private.param<std::string>("child_frame", child_frame, "base_link");
    n_private.param<double>("car_radius", car_radius, 0.35);
    n_private.param<double>("max_extension", max_extension, 8.0);
    n_private.param<double>("max_extension_ratio", max_extension_ratio, 0.5);    
    n_private.param<int>("num_samples", num_samples, 100);  
    n_private.param<int>("max_attempts", max_attempts, 3);          
    n_private.param<double>("dis_threshold", dis_threshold, 0.05);
    n_private.param<double>("total_teb_filter_path_length_threshold", total_teb_filter_path_length_threshold, 0.2);
    n_private.param<double>("local_start_threshold", local_start_threshold, 2.0);
    n_private.param<double>("candidate_local_end_threshold", candidate_local_end_threshold, 8.0);
    n_private.param<double>("patience", patience, 3.0);
}

void publishCandidateMarkers(const std::vector<geometry_msgs::PoseStamped>& candidates, const std::string& frame_id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "candidate_points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    
    for (const auto &cand : candidates)
    {
        marker.points.push_back(cand.pose.position);
    }
    
    marker_pub.publish(marker);
}

// 發佈最佳候選點 marker
void publishBestCandidateMarker(const geometry_msgs::PoseStamped& best, const std::string& frame_id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "best_candidate";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = best.pose;
    marker.scale.x = 0.06;
    marker.scale.y = 0.06;
    marker.scale.z = 0.06;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    
    marker_pub.publish(marker);
}

// 計算兩個位姿的歐氏距離
double calculateDistance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    return std::sqrt(dx*dx + dy*dy);
}

// 濾除相鄰重複點（小於 dis_threshold）
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

/*===========================================
   以下為利用均勻採樣與 cost function 調整候選點的方法
   原本的 adjustCandidate 改成使用均勻灑點找出離原始路徑較近且無障礙的最佳候選點
===========================================*/

// 2. 評估候選點的成本函式
//   成本包含：與中心點的距離成本與障礙物懲罰
double evaluateCandidateCost(const geometry_msgs::PoseStamped &candidate, const geometry_msgs::PoseStamped &center)
{
    // 成本1：與中心點的歐氏距離
    double dx = candidate.pose.position.x - center.pose.position.x;
    double dy = candidate.pose.position.y - center.pose.position.y;
    double cost_distance = std::sqrt(dx * dx + dy * dy);

    // 成本2：障礙物懲罰
    double cost_obstacle = 0.0;
    for (const auto &obs : obstacles_.obstacles)
    {
        // 假設 obs.polygon.points[0] 為障礙物中心
        const geometry_msgs::Point32 &obs_center = obs.polygon.points[0];
        double ddx = candidate.pose.position.x - obs_center.x;
        double ddy = candidate.pose.position.y - obs_center.y;
        double dist_to_obs = std::sqrt(ddx * ddx + ddy * ddy);
        double safe_distance = obs.radius + car_radius;
        if (dist_to_obs < safe_distance)
        {
            // 候選點在不安全範圍內，直接返回極高成本
            return std::numeric_limits<double>::max();
        }
        else
        {
            // 當離安全距離越遠時，懲罰越小
            cost_obstacle += 1.0 / (dist_to_obs - safe_distance + 0.001);
        }
    }
    // 可調整障礙物懲罰權重（此處設為 10.0）
    double total_cost = cost_distance + cost_obstacle * 10.0;
    return total_cost;
}

// 調整候選點的主函式
// 以 front 往 back 的方向定義 0°（採用向量 back - front），
// 利用隨機均勻採樣在 candidate_center 周圍產生候選點，先排除落在障礙物上的點，再從安全候選點中選出成本最低者。
// 接著根據候選點相對於參考方向（以 front→back 為 0°）的角度，計算 fuzzy 值（以 22.5° 為間隔），
// 若 fraction 非零則回傳「主區域:1.00, 左區域:(1-fraction), 右區域:(fraction)」；
// 若 fraction 接近 0，則認為候選點在線上，回傳左右區域各 0.50。
geometry_msgs::PoseStamped adjustCandidate(const geometry_msgs::PoseStamped &front, const geometry_msgs::PoseStamped &back, 
                                            const std::string &mode, std::string &patience_orientation)
{
    // 根據 mode 選定候選中心點
    geometry_msgs::PoseStamped candidate_center;
    if (mode == "front_mode")
    {
        candidate_center = front;
    }
    else if (mode == "back_mode")
    {
        candidate_center = back;
    }

    // 若沒有障礙物，直接回傳中心點
    if (obstacles_.obstacles.empty())
    {
        patience_orientation = "all";
        return candidate_center;
    }
    
    // 採樣參數
    double sample_radius = max_extension * max_extension_ratio;  // 初始採樣半徑（可自行調整）
    int attempt = 0;
    std::vector<geometry_msgs::PoseStamped> candidates;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist_angle(0, 2 * M_PI);
    std::uniform_real_distribution<> dist_radius(0, 1);
    
    // 若沒有候選點，嘗試擴大採樣半徑
    do 
    {
        candidates.clear();
        for (int i = 0; i < num_samples; i++)
        {
            double angle = dist_angle(gen);
            double r = sample_radius * std::sqrt(dist_radius(gen));  // 均勻分布
            geometry_msgs::PoseStamped candidate = candidate_center;
            candidate.pose.position.x += r * std::cos(angle);
            candidate.pose.position.y += r * std::sin(angle);
            if (checkObstacle(candidate))
            {
                continue;
            }
            candidates.push_back(candidate);
        }
        if (!candidates.empty()) //有找到候選點就會跳出去
        {
            break;
        }
        // 若沒找到安全點，擴大採樣半徑
        sample_radius *= 1.5;
        attempt++;
    } while (attempt < max_attempts);
    
    // 若仍無安全候選點，則回傳中心點
    if (candidates.empty())
    {
        patience_orientation = "none";
        return candidate_center;
    }
    
    // 從候選點中選出成本最低的
    double best_cost = std::numeric_limits<double>::max();
    geometry_msgs::PoseStamped best_candidate;
    for (const auto &cand : candidates)
    {
        double cost = evaluateCandidateCost(cand, candidate_center);
        if (cost < best_cost)
        {
            best_cost = cost;
            best_candidate = cand;
        }
    }
    
    // 呼叫輔助函式發佈候選點 marker和最佳候選點 marker
    publishCandidateMarkers(candidates, candidate_center.header.frame_id);
    publishBestCandidateMarker(best_candidate, candidate_center.header.frame_id);

    // 計算參考方向：以 back 往 front 為 0°（使用向量 front(遠) - back(近))
    // v_front_to_back(固定)
    double ref_dx = front.pose.position.x - back.pose.position.x;
    double ref_dy = front.pose.position.y - back.pose.position.y;
    double ref_angle = std::atan2(ref_dy, ref_dx); // 參考角（弧度）
    
    // 計算最佳候選點相對於 candidate_center 的角度
    // v_candidate_center_to_best_candidate
    double cand_dx = best_candidate.pose.position.x - candidate_center.pose.position.x;
    double cand_dy = best_candidate.pose.position.y - candidate_center.pose.position.y;
    double cand_angle = std::atan2(cand_dy, cand_dx); // 弧度
    
    // 計算相對角度（轉換到 [0,360]）
    double rel_angle = (cand_angle - ref_angle) * 180.0 / M_PI;
    if (rel_angle < 0)
        rel_angle += 360.0;
    
    // 將 0~360 度以 22.5 度為間隔劃分成 16 區
    double fuzzy = rel_angle / 22.5;  // fuzzy 值
    int lower = static_cast<int>(std::floor(fuzzy)); //無條件捨去+強制轉成int
    double fraction = fuzzy - lower;
    
    const double epsilon = 0.01;  // 判斷是否在線上的門檻
    std::string orientation_info;
    if (fraction < epsilon || (1 - fraction) < epsilon)
    {
        // 若在線上，則僅回傳左右兩區：左區 (lower-1) 與右區 lower
        int seg_left = (lower + 15) % 16;
        int seg_right = lower % 16;
        orientation_info = std::to_string(seg_left) + ":0.50," + std::to_string(seg_right) + ":0.50";
    }
    else
    {
        // 候選點不在線上：回傳主區域、左鄰與右鄰的隸屬度
        int seg_main = lower % 16;
        int seg_left = (lower + 15) % 16;
        int seg_right = (lower + 1) % 16;
        // 利用 std::to_string 轉換，並截取小數點後 2 位
        std::string mem_left = std::to_string(1.0 - fraction);
        std::string mem_right = std::to_string(fraction);
        size_t pos;
        if ((pos = mem_left.find('.')) != std::string::npos)
        {
            mem_left = mem_left.substr(0, pos + 3);
        }
        if ((pos = mem_right.find('.')) != std::string::npos)
        {
            mem_right = mem_right.substr(0, pos + 3);
        }
        orientation_info = std::to_string(seg_main) + ":1.00," + std::to_string(seg_left) + ":" + mem_left + "," + std::to_string(seg_right) + ":" + mem_right;
    }
    patience_orientation = orientation_info;
    
    return best_candidate;
}

void InitialCallback(const std_msgs::Bool &flag)
{
  if (flag.data)
  {
    path_mode = ARRIVED_PATH ;

    local_end_patience.assign(16, 0.0);

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

    locked_local_end_prev_pose.header.seq = 0;
    locked_local_end_prev_pose.header.frame_id = "";
    locked_local_end_prev_pose.header.stamp = ros::Time::now();
    locked_local_end_prev_pose.pose.position.x = 0.0;
    locked_local_end_prev_pose.pose.position.y = 0.0;
    locked_local_end_prev_pose.pose.position.z = 0.0;
    locked_local_end_prev_pose.pose.orientation.x = 0.0;
    locked_local_end_prev_pose.pose.orientation.y = 0.0;
    locked_local_end_prev_pose.pose.orientation.z = 0.0;
    locked_local_end_prev_pose.pose.orientation.w = 0.0;
    
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
    if ( std::fabs(last_teb_filter_pose.position.x - last_pose.position.x) > 0.0001 ||
         std::fabs(last_teb_filter_pose.position.y - last_pose.position.y) > 0.0001 ||
         std::fabs(last_teb_filter_pose.orientation.z - last_pose.orientation.z) > 0.0001 ||
         std::fabs(last_teb_filter_pose.orientation.w - last_pose.orientation.w) > 0.0001 )
    {
        path_mode = NEW_PATH ;
        last_pose = last_teb_filter_pose;
        // 計算總的 index 數量：包含起始點及從最近點開始到最後的所有點
        int index_count = teb_filter_path.poses.size();
        total_teb_filter_path_length = total_length;
        total_teb_index = index_count;
        ROS_INFO("TEB Path total length: %.3f", total_teb_filter_path_length);
        ROS_INFO("TEB Path total index count: %d", total_teb_index);        
    }
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
    double total_dist_current_to_end = d1 + d2; // 快到終點前用的

    // -------------------------------
    // 1. 決定 local_start
    // -------------------------------
    // 情況1：從TebGlobalPathCallback 得到的 total_teb_filter_path_length <= 2.0, > 0.2
    if (total_teb_filter_path_length <= local_start_threshold && total_dist_current_to_end <= local_start_threshold)
    {
        if (!local_start_locked) // 開始鎖定
        {
            locked_local_start = initial_robot_poseStamped;
            via_start_idx = nearest_idx;
            local_start_locked = true;
        }
        // 如果 locked_local_start 有障礙物，直接用四方位調整找沒有障礙的位置
        if (checkObstacle(locked_local_start))
        {
            // ROS_INFO("local_start 1. (%.2f, %.2f) has obstacle ", locked_local_start.pose.position.x, locked_local_start.pose.position.y);
            int start_next_idx = via_start_idx;
            // 只有2顆也沒關係，就是終點（dis_threshold = 0.05）
            while (calculateDistance(teb_filter_path.poses[start_next_idx], locked_local_start) <= dis_threshold)
            {
                start_next_idx++;
            }
            geometry_msgs::PoseStamped start_next_pose = teb_filter_path.poses[start_next_idx];
            std::string selected_orientation;  //跟後面用同一個function 但這裡沒用到 反正就給它一直變不影響
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
        via_start_idx = nearest_idx; // via_point 可以是 start 或 end
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
        candidate_idx = total_teb_index - 1; // 最後一點
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
            // 從候選點往後延伸，逐步找尋無障礙區域
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
    // 情況B：candidate local_end 為 teb_filter_path_end
    if ((std::fabs(adj_candidate_local_end.pose.position.x - teb_filter_path_end.pose.position.x) < 0.001) &&
        (std::fabs(adj_candidate_local_end.pose.position.y - teb_filter_path_end.pose.position.y) < 0.001) )
    {
        if (checkObstacle(adj_candidate_local_end))
        {
            // ROS_INFO("local_end b. (%.2f, %.2f) has obstacle ", adj_candidate_local_end.pose.position.x, adj_candidate_local_end.pose.position.y);
            if (total_dist_current_to_end > lock_end_threshold)
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
                if(!local_end_locked)
                {
                    //最後一點的前一點 一定>0.05 這裡是 candidate local_end 為 teb_filter_path_end, 至少兩個點 [2-2=0]
                    geometry_msgs::PoseStamped end_prev_pose = teb_filter_path.poses[total_teb_index - 2];
                    std::string selected_orientation;
                    adj_candidate_local_end = adjustCandidate(adj_candidate_local_end, end_prev_pose, "front_mode", selected_orientation);
                    
                    if (selected_orientation == "all") 
                    {
                        // 若返回 "all"，則 16 區域全加 1
                        for (int i = 0; i < 16; i++) 
                        {
                            local_end_patience[i] += 1.0;
                        }
                    } 
                    else if (selected_orientation == "none") 
                    {
                        // 若返回 "none"，則不做任何更新
                        ;
                    } 
                    else 
                    {
                        // 否則，解析返回字串，例如 "1:1.00,0:0.47,2:0.53"
                        std::istringstream iss(selected_orientation);
                        std::string token;
                        while (std::getline(iss, token, ',')) 
                        {
                            size_t pos = token.find(':');
                            if (pos != std::string::npos) 
                            {
                                int zone = std::stoi(token.substr(0, pos));
                                double val = std::stod(token.substr(pos + 1));
                                local_end_patience[zone] += val;
                            }
                        }
                    }

                    // 遍歷 16 區域，找出 patience 值最大的區域，若最大值>= 3.0 則鎖定該 candidate
                    int locked_zone = -1;
                    double max_patience = -1.0;
                    for (int i = 0; i < 16; i++) 
                    {
                        if (local_end_patience[i] >= 3.0 && local_end_patience[i] > max_patience)
                        {
                            max_patience = local_end_patience[i];
                            locked_zone = i;
                        }
                    }
                    
                    if (locked_zone != -1)
                    {
                        locked_local_end = adj_candidate_local_end;
                                        
                        // 將原始位姿轉換為 tf2::Transform
                        tf2::Transform tf_end, tf_prev;
                        tf2::fromMsg(teb_filter_path_end.pose, tf_end);
                        tf2::fromMsg(end_prev_pose.pose, tf_prev);

                        // 定義 T_rel 使得： teb_filter_path_end = T_rel * end_prev_pose
                        tf2::Transform T_rel = tf_end * tf_prev.inverse();

                        tf2::Transform tf_lock_end, tf_lock_prev;
                        tf2::fromMsg(locked_local_end.pose, tf_lock_end);
                        // 計算 new_prev_pose = T_rel⁻¹ * new_end_pose
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
        (std::fabs(candidate_local_end.pose.position.y - teb_filter_path_end.pose.position.y) < 0.001))
    {
        global_end = local_end_pose;
    }
    //反之沒在偵測範圍內 用假的就好沒關係
    else
    {
        global_end = teb_filter_path_end;
    }
    global_path_end_pub.publish(global_end);
    ROS_INFO("global_end (%.2f, %.2f)", global_end.pose.position.x, global_end.pose.position.y);
    
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

    ROS_INFO("odom_local_start_pose (%.2f, %.2f)", odom_local_start_pose.pose.position.x, odom_local_start_pose.pose.position.y);
    ROS_INFO("odom_local_end_pose (%.2f, %.2f)", odom_local_end_pose.pose.position.x, odom_local_end_pose.pose.position.y);

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
    marker_pub = nh.advertise<std_msgs::Bool>("/adjust_candidate", 10);

    via_client = nh.serviceClient<campusrover_msgs::TEBVias>("/teb_via_points");
    start_end_client = nh.serviceClient<campusrover_msgs::TEBStartEnd>("/teb_start_end");

    ros::Timer current_pose_timer = nh.createTimer(ros::Duration(0.1), CurrentPoseTimer);
    ros::Timer start_end_via_timer = nh.createTimer(ros::Duration(0.05), StartEndViaTimer);

    ros::spin();
    return 0;
}
