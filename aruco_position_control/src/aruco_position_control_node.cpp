#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>


class ArUcoPositionControl {
public:
    enum ParkingState {
        ALIGN_ROTATION,
        FINAL_ALIGNMENT,
        DOCKING,
        APPROACH_TARGET
    };      

    ArUcoPositionControl() {
        aruco_pose_sub_ = nh_.subscribe("/aruco_pose", 1, &ArUcoPositionControl::poseCallback, this);
        //pointcloud_sub_ = nh_.subscribe("/camera/depth/color/points", 1, &ArUcoPositionControl::pointCloudCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 1);
        robot_pose_sub_ = nh_.subscribe("/ekf_odometry/ekf_odom", 10, &ArUcoPositionControl::robotPoseCallback, this);
        charge_flag_sub_ = nh_.subscribe("/tfoi_charge_flag", 10, &ArUcoPositionControl::chargeFlagCallback, this);

        // Control Parameter Load
        nh_.param("max_speed", max_speed, 0.16);
        nh_.param("min_speed", min_speed, 0.05);
        nh_.param("slow_distance", slow_distance, 1.0);
        nh_.param("Kp_align_yaw", Kp_align_yaw, 1.15);
        nh_.param("Kp_yaw", Kp_yaw, 1.45);
        nh_.param("angular_z_limit", angular_z_limit, 0.25);
        last_pose_time_ = ros::Time::now(); 
        last_pose_received_ = false;
        pose_received = false;
        aruco_received = false;

        current_frame_ = cv::Mat::zeros(320, 480, CV_8UC3);
        current_frame_.setTo(cv::Scalar(128, 0, 0));  
        last_pose_ = geometry_msgs::PoseStamped(); 
    }


// 콜백 함수: 로봇의 현재 pose 업데이트 (robot_pose_ekf 기준)
void robotPoseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    robot_pose = msg->pose.pose;
    pose_received = true;
}

void chargeFlagCallback(const std_msgs::Bool::ConstPtr& charge_flag_msg)
{
  charge_flag = charge_flag_msg->data;  
}


void controlLoop() {
    if (!pose_received || !aruco_received) {
        ROS_WARN("Waiting for pose data...");
        return;
    }

    double aruco_x = aruco_pose.position.x;
    double aruco_z = aruco_pose.position.z;    
    double robot_x = robot_pose.position.x;
    double robot_y = robot_pose.position.y;

   
    // 로봇의 현재 yaw 값 추출
    tf::Quaternion q(robot_pose.orientation.x, robot_pose.orientation.y,
                     robot_pose.orientation.z, robot_pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, robot_yaw;
    m.getRPY(roll, pitch, robot_yaw);

    // 로봇과 아루코 마커의 상대적인 방향 계산 (arctan으로 각도 계산)
    double delta_x = std::fabs(aruco_z - robot_x);
    double delta_y = std::fabs(aruco_x - robot_y);

    // goal_yaw: 로봇이 아루코 마커를 향해야 하는 목표 yaw 값
    double angle_to_heading = atan2(delta_x, delta_y);

    // 세련된 하늘색으로 출력
    ROS_INFO("\033[38;5;39mRobot Yaw: %f\033[0m", robot_yaw);
    ROS_INFO("\033[38;5;213mangle_to_heading: %f\033[0m", angle_to_heading);

    // 충전소의 목표 Yaw 각도 계산
    goal_yaw = atan2(aruco_x, aruco_z);  // 목표 yaw 계산
    double yaw_error = goal_yaw; // - robot_yaw;    
    if (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    if (yaw_error < -M_PI) yaw_error += 2 * M_PI;  // -π ~ π 범위로 정규화

    // 로봇의 현재 각도
    double goal_align_yaw =  robot_yaw; 
    if (goal_align_yaw > M_PI) goal_align_yaw -= 2 * M_PI;
    if (goal_align_yaw < -M_PI) goal_align_yaw += 2 * M_PI;    // -π ~ π 범위로 정규화

    // 충전소까지 거리 계산
    double distance_to_dock = sqrt(pow(aruco_x, 2) + pow(aruco_z, 2));

    ParkingState state;

    if (distance_to_dock < 0.3 && distance_to_dock > 0.18) {
        state = ALIGN_ROTATION;
    } else if (distance_to_dock < 0.16) {
        state = FINAL_ALIGNMENT;
    } else if(charge_flag == true ) {
        state = DOCKING;
        ROS_INFO("charge_flag : %d", charge_flag);
    } else {
        state = APPROACH_TARGET;
    }

    switch(state){
        case ALIGN_ROTATION :
            ROS_INFO("\033[38;5;46m[ALIGN_ROTATION] Aligning yaw...\033[0m");
            if(fabs(yaw_error) > 0.02){
                double sign = (yaw_error > 0.0) ? -1.0 : 1.0;
                cmd_vel_msg.linear.x = 0.0;
                cmd_vel_msg.angular.z = Kp_align_yaw * yaw_error;
            }
            break;
        
        case FINAL_ALIGNMENT :
            ROS_INFO("\033[38;5;39m[FINAL_ALIGNMENT] Final adjustment...\033[0m");
            if (fabs(goal_align_yaw) > 0.01){
                double sign = (goal_align_yaw > 0.0) ? -1.0 : 1.0;
                cmd_vel_msg.linear.x = -(distance_to_dock * 0.28);
                cmd_vel_msg.angular.z = sign * goal_align_yaw * distance_to_dock * distance_to_dock;
                if(distance_to_dock < 0.1) cmd_vel_msg.angular.z = 0.0;
            }
            break;

        case DOCKING :
            ROS_INFO("\033[55;22;94m[FINAL_ALIGNMENT] DOCKING Completed!!!\033[0m");
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = 0.0;
            
        case APPROACH_TARGET :
            ROS_INFO("\033[38;5;213m[APPROACH_TARGET] Moving to target...\033[0m");
            cmd_vel_msg.angular.z = Kp_yaw * yaw_error;
            if(distance_to_dock < 0.1){
                cmd_vel_msg.linear.x = 0.0;
            } else if (distance_to_dock < slow_distance) {
                cmd_vel_msg.linear.x = -min_speed - (max_speed - min_speed) * (distance_to_dock / slow_distance);
            } else {
                cmd_vel_msg.linear.x = -max_speed;
            }
            break;
        }    
    // limit speed
    if (cmd_vel_msg.linear.x < -max_speed) cmd_vel_msg.linear.x = -max_speed;
    if (cmd_vel_msg.linear.x > -min_speed && distance_to_dock > 0.1) cmd_vel_msg.linear.x = -min_speed;
    // 퍼블리시
    cmd_vel_pub_.publish(cmd_vel_msg);

    ROS_INFO("goal_yaw: %f, yaw_error: %f, angular_z: %f, distance: %f, linear_x: %f",
            goal_yaw, yaw_error, cmd_vel_msg.angular.z, distance_to_dock, cmd_vel_msg.linear.x);
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    aruco_pose = msg->pose;
    aruco_received = true;

    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;

    // 현재 상태 시각화 (이미지 표시)
    current_frame_ = cv::Mat::zeros(480, 640, CV_8UC3);
    current_frame_.setTo(cv::Scalar(86, 0, 0));

    // 충전소 박스 그리기
    int charger_x = current_frame_.cols - 20;
    cv::Rect charger_rect(charger_x, current_frame_.rows / 2 - 10, 150, 50);
    cv::rectangle(current_frame_, charger_rect, cv::Scalar(0, 0, 255), -1);

    // 카메라 박스 그리기 (마커 위치 기반)
    int camera_x = charger_x - static_cast<int>(y * 150) - 93; 
    int camera_y = static_cast<int>(x * 100) + current_frame_.rows / 2 - 25;  
    cv::Rect camera_rect(camera_x, camera_y, 100, 70);
    cv::rectangle(current_frame_, camera_rect, cv::Scalar(0, 165, 255), -1);

    // 포즈 텍스트 표시 (Pose(x, y, z) 값 표시)
    std::string pose_text = "Pose (x, y, z): (" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
    cv::putText(current_frame_, pose_text, cv::Point(10, current_frame_.rows - 50), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    // 속도 텍스트 표시
    std::string vel_text = "Vel (x, y, z): (" + std::to_string(cmd_vel_msg.linear.x) + ", " + 
                        std::to_string(cmd_vel_msg.linear.y) + ", " + std::to_string(cmd_vel_msg.angular.z) + ")";
    cv::putText(current_frame_, vel_text, cv::Point(10, current_frame_.rows - 10), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    // 이미지 출력
    cv::imshow("ArUco Position Visualization", current_frame_);
    cv::waitKey(1);
}


private:
    ros::NodeHandle nh_;
    ros::Subscriber aruco_pose_sub_;
    ros::Subscriber robot_pose_sub_;
    ros::Subscriber charge_flag_sub_;

    ros::Publisher cmd_vel_pub_; 
    ros::Timer timer_;

    ros::Time last_pose_time_;
    bool last_pose_received_;
    bool pose_received; 
    bool aruco_received;
    bool charge_flag;
    double max_speed, min_speed, slow_distance;
    double Kp_align_yaw, Kp_yaw, angular_z_limit, goal_yaw;

    
    cv::Mat current_frame_;
    geometry_msgs::PoseStamped last_pose_;
    geometry_msgs::Pose robot_pose;
    geometry_msgs::Pose aruco_pose;
    geometry_msgs::Twist cmd_vel_msg;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_position_control");
    ArUcoPositionControl controller;
    //ros::spin();

    ros::Rate rate(100);  

    while (ros::ok()) {
        controller.controlLoop();  // 클래스 메서드 호출 시 객체를 통해 접근
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}