#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Trigger.h>
#include <campusrover_msgs/TEBStartEnd.h>

// 服務 client，呼叫 "/teb_start_end" 服務（型態為 campusrover_msgs::TEBStartEnd）
ros::ServiceClient start_end_client;

// Trigger 服務 Callback（Set1）：設定起點與終點並呼叫外部服務
bool tebStartEndSet1Callback(std_srvs::Trigger::Request &req,
                             std_srvs::Trigger::Response &res)
{
    geometry_msgs::PoseStamped start_pose, end_pose;
    // 設定 Set1 起點：例如 (-4,-4)
    start_pose.header.frame_id = "odom";
    start_pose.pose.position.x = 1.0;
    start_pose.pose.position.y = 1.0;
    start_pose.pose.position.z = 0.0;
    start_pose.pose.orientation.x = 0.0;
    start_pose.pose.orientation.y = 0.0;
    start_pose.pose.orientation.z = 0.707;
    start_pose.pose.orientation.w = 0.707;
    
    // 設定 Set1 終點：例如 (4,4)
    end_pose.header.frame_id = "odom";
    end_pose.pose.position.x = 7.0;
    end_pose.pose.position.y = 7.0;
    end_pose.pose.position.z = 0.0;
    end_pose.pose.orientation.x = 0.0;
    end_pose.pose.orientation.y = 0.0;
    end_pose.pose.orientation.z = 0.707;
    end_pose.pose.orientation.w = 0.707;

    campusrover_msgs::TEBStartEnd start_end_srv;
    start_end_srv.request.start = start_pose;
    start_end_srv.request.end = end_pose;

    if(start_end_client.call(start_end_srv))
    {
        if(start_end_srv.response.success)
        {
            ROS_INFO("TEB start/end service call succeeded (set1).");
        }
        else
        {
            ROS_WARN("TEB start/end service call returned failure (set1).");
        }
    }
    else
    {
        ROS_WARN("TEB start/end service call failed (set1).");
    }

    res.success = true;
    res.message = "Set1: start and end processed.";
    ROS_INFO("Set1: start and end returned.");
    return true;
}

// Trigger 服務 Callback（Set2）：設定起點與終點並呼叫外部服務
bool tebStartEndSet2Callback(std_srvs::Trigger::Request &req,
                             std_srvs::Trigger::Response &res)
{
    geometry_msgs::PoseStamped start_pose, end_pose;
    // 設定 Set2 起點：例如 (-1,-2)
    start_pose.header.frame_id = "odom";
    start_pose.pose.position.x = -1.0;
    start_pose.pose.position.y = -2.0;
    start_pose.pose.position.z = 0.0;
    start_pose.pose.orientation.x = 0.0;
    start_pose.pose.orientation.y = 0.0;
    start_pose.pose.orientation.z = -0.707;
    start_pose.pose.orientation.w = 0.707;
    
    // 設定 Set2 終點：例如 (3,4)
    end_pose.header.frame_id = "odom";
    end_pose.pose.position.x = 3.0;
    end_pose.pose.position.y = 4.0;
    end_pose.pose.position.z = 0.0;
    end_pose.pose.orientation.x = 0.0;
    end_pose.pose.orientation.y = 0.0;
    end_pose.pose.orientation.z = 0.0;
    end_pose.pose.orientation.w = 1.0;

    campusrover_msgs::TEBStartEnd start_end_srv;
    start_end_srv.request.start = start_pose;
    start_end_srv.request.end = end_pose;
    
    if(start_end_client.call(start_end_srv))
    {
        if(start_end_srv.response.success)
        {
            ROS_INFO("TEB start/end service call succeeded (set2).");
        }
        else
        {
            ROS_WARN("TEB start/end service call returned failure (set2).");
        }
    }
    else
    {
        ROS_WARN("TEB start/end service call failed (set2).");
    }
    
    res.success = true;
    res.message = "Set2: start and end processed.";
    ROS_INFO("Set2: start and end returned.");
    return true;
}

// Trigger 服務 Callback（Set3）：設定起點與終點並呼叫外部服務
bool tebStartEndSet3Callback(std_srvs::Trigger::Request &req,
                             std_srvs::Trigger::Response &res)
{
    geometry_msgs::PoseStamped start_pose, end_pose;
    // 設定 Set3 起點：例如 (4,4)
    start_pose.header.frame_id = "odom";
    start_pose.pose.position.x = 4.0;
    start_pose.pose.position.y = 4.0;
    start_pose.pose.position.z = 0.0;
    start_pose.pose.orientation.x = 0.0;
    start_pose.pose.orientation.y = 0.0;
    start_pose.pose.orientation.z = 0.707;
    start_pose.pose.orientation.w = 0.707;
    
    // 設定 Set3 終點：例如 (-3,-4)
    end_pose.header.frame_id = "odom";
    end_pose.pose.position.x = -3.0;
    end_pose.pose.position.y = -4.0;
    end_pose.pose.position.z = 0.0;
    end_pose.pose.orientation.x = 0.0;
    end_pose.pose.orientation.y = 0.0;
    end_pose.pose.orientation.z = 0.0;
    end_pose.pose.orientation.w = 1.0;

    campusrover_msgs::TEBStartEnd start_end_srv;
    start_end_srv.request.start = start_pose;
    start_end_srv.request.end = end_pose;
    
    if(start_end_client.call(start_end_srv))
    {
        if(start_end_srv.response.success)
        {
            ROS_INFO("TEB start/end service call succeeded (set3).");
        }
        else
        {
            ROS_WARN("TEB start/end service call returned failure (set3).");
        }
    }
    else
    {
        ROS_WARN("TEB start/end service call failed (set3).");
    }
    
    res.success = true;
    res.message = "Set3: start and end processed.";
    ROS_INFO("Set3: start and end returned.");
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teb_start_end_server");
    ros::NodeHandle nh;

    // 建立服務 client，連線到 "/teb_start_end" 服務
    start_end_client = nh.serviceClient<campusrover_msgs::TEBStartEnd>("/teb_start_end");

    // 建立三個 Trigger 服務，分別用來處理不同的點組 (使用 std_srvs/Trigger 型態)
    ros::ServiceServer service1 = nh.advertiseService("test_start_end_1", tebStartEndSet1Callback);
    ros::ServiceServer service2 = nh.advertiseService("test_start_end_2", tebStartEndSet2Callback);
    ros::ServiceServer service3 = nh.advertiseService("test_start_end_3", tebStartEndSet3Callback);

    ROS_INFO("TEBStartEnd trigger services are ready.");
    ros::spin();
    return 0;
}
