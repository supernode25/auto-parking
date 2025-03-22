// 1. angle_to_heading , goal_yaw 비교
// 2. 제어파라미터 맞추기
// 3. 스펙 평가
// 4. UI 개선

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
        pointcloud_sub_ = nh_.subscribe("/camera/depth/color/points", 1, &ArUcoPositionControl::pointCloudCallback, this);


        // Control Parameter Load
        nh_.param("max_speed", max_speed, 0.16);
        nh_.param("min_speed", min_speed, 0.05);
        nh_.param("slow_distance", slow_distance, 1.0);
        nh_.param("Kp_align_yaw", Kp_align_yaw, 1.08);
        nh_.param("Kp_yaw", Kp_yaw, 1.15);
        nh_.param("angular_z_limit", angular_z_limit, 0.25);
        last_pose_time_ = ros::Time::now(); 
        last_pose_received_ = false;
        pose_received = false;
        aruco_received = false;
        charge_flag = false;

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

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    // PointCloud2 메시지를 PCL 포인트 클라우드로 변환
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    double min_distance = std::numeric_limits<double>::infinity();  // 무한대로 초기화

    // 포인트 클라우드에서 각 점에 대해 전방에서 가장 가까운 점을 찾음
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        pcl::PointXYZ point = cloud->points[i];

        // 카메라 전방을 기준으로, 점이 전방에 있을 경우에만 거리 계산
        if (point.z > 0) {
            double distance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

            if (distance < min_distance) {
                min_distance = distance;
            }
        }
    }

    ROS_INFO("Minimum distance to the front: %f meters", min_distance);
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
    double delta_x = aruco_z - robot_x;
    double delta_y = aruco_x - robot_y;

    // goal_yaw: 로봇이 아루코 마커를 향해야 하는 목표 yaw 값
    double angle_to_heading = atan2(delta_y, delta_x);
    double target_yaw = angle_to_heading; // - robot_yaw;    
    if (target_yaw > M_PI) target_yaw -= 2 * M_PI;
    if (target_yaw < -M_PI) target_yaw += 2 * M_PI;  // -π ~ π 범위로 정규화

    // 세련된 하늘색으로 출력
    ROS_INFO("\033[38;5;39mRobot Yaw: %f\033[0m", robot_yaw);
    ROS_INFO("\033[38;5;213mangle_to_heading: %f\033[0m", angle_to_heading);

    // 충전소의 목표 Yaw 각도 계산
    goal_yaw = atan2(aruco_x, aruco_z);  // 목표 yaw 계산
    // 로봇의 현재 각도
    double goal_align_yaw =  goal_yaw; 
    if (goal_align_yaw > M_PI) goal_align_yaw -= 2 * M_PI;
    if (goal_align_yaw < -M_PI) goal_align_yaw += 2 * M_PI;    // -π ~ π 범위로 정규화

    // 충전소까지 거리 계산
    double distance_to_dock = sqrt(pow(aruco_x, 2) + pow(aruco_z, 2));

    ParkingState state;

    if (distance_to_dock > 1.0) {
        state = APPROACH_TARGET;
    } else if (distance_to_dock <= 1.0 && distance_to_dock > 0.3) {
        state = ALIGN_ROTATION;
    } else if (distance_to_dock <= 0.3) { 
        if (fabs(goal_align_yaw) < 0.35) {  // goal_yaw 오차가 0.25 이내면 FINAL_ALIGNMENT
            if (distance_to_dock < 0.05 && fabs(goal_align_yaw) < 0.04 && charge_flag == true ) {  
                state = DOCKING;  // 도킹 완료
            } else {
                state = FINAL_ALIGNMENT;
            }
        } else {
            state = ALIGN_ROTATION;  // 아직 yaw가 정렬되지 않았다면 유지
        }
    }

    switch(state){
         case APPROACH_TARGET :
            ROS_INFO("\033[38;5;213m[APPROACH_TARGET] Moving to target...\033[0m");
            if(fabs(goal_align_yaw) > 0.01){
                cmd_vel_msg.angular.z = Kp_yaw * goal_align_yaw;
            }            
            if(distance_to_dock < 0.05){
                cmd_vel_msg.linear.x = 0.0;
            } else if (distance_to_dock < slow_distance) {
                cmd_vel_msg.linear.x = -min_speed - (max_speed - min_speed) * (distance_to_dock / slow_distance);
            } else {
                cmd_vel_msg.linear.x = -max_speed;
            }
            break;

        case ALIGN_ROTATION :
            ROS_INFO("\033[38;5;46m[ALIGN_ROTATION] Aligning yaw...\033[0m");
            speed_factor = std::max(0.1, distance_to_dock * 0.3);
            if(fabs(goal_align_yaw) > 0.01){
                double sign = (goal_align_yaw > 0.0) ? 1.0 : -1.0;             
                cmd_vel_msg.angular.z = Kp_align_yaw * goal_align_yaw;
            }
            cmd_vel_msg.linear.x = -speed_factor;
            break;
        
        case FINAL_ALIGNMENT :
            ROS_INFO("\033[38;5;39m[FINAL_ALIGNMENT] Final adjustment...\033[0m");
            speed_factor = std::max(0.08, distance_to_dock * 0.25);
            if (fabs(goal_align_yaw) > 0.002){
                double sign = (goal_align_yaw > 0.0) ? 1.0 : -1.0;             
                cmd_vel_msg.angular.z = sign * goal_align_yaw * distance_to_dock * 1.02;
            }
            if(distance_to_dock < 0.08 && fabs(goal_align_yaw < 0.01)) {
            cmd_vel_msg.angular.z = 0.0;  // 도킹 직전에 각속도 멈추기
            }
            cmd_vel_msg.linear.x = -speed_factor;
            break;

        case DOCKING :
            ROS_INFO("\033[55;22;94m[FINAL_ALIGNMENT] DOCKING Completed!!!\033[0m");
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = 0.0;
            break;   
        }    
    // limit speed
    if (cmd_vel_msg.linear.x < -max_speed) cmd_vel_msg.linear.x = -max_speed;
    if (cmd_vel_msg.linear.x > -min_speed && distance_to_dock > 0.1) cmd_vel_msg.linear.x = -min_speed;
    // 퍼블리시
    cmd_vel_pub_.publish(cmd_vel_msg);

    ROS_INFO("goal_align_yaw : %f, target_yaw : %f, angular_z: %f, distance: %f, linear_x: %f",
            goal_align_yaw, target_yaw, cmd_vel_msg.angular.z, distance_to_dock, cmd_vel_msg.linear.x);
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    aruco_pose = msg->pose;
    aruco_received = true;

    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;

    // 이전 값 저장
    static double smoothed_x = 0.0;
    static double smoothed_z = 0.0;

    // 필터 계수 (0.0 ~ 1.0 사이) - 0.1: 강한 필터링, 0.5: 약한 필터링
    const double alpha = 0.1; 

    // 새로운 값에 저속 필터 적용
    smoothed_x = alpha * x + (1 - alpha) * smoothed_x;
    smoothed_z = alpha * z + (1 - alpha) * smoothed_z;

    // 현재 상태 시각화 (이미지 표시)
    current_frame_ = cv::Mat::zeros(480, 640, CV_8UC3);
    current_frame_.setTo(cv::Scalar(86, 0, 0));

    // 충전소 박스 그리기
    int charger_x = current_frame_.cols - 10;
    cv::Rect charger_rect(charger_x, current_frame_.rows / 2, 150, 50);
    cv::rectangle(current_frame_, charger_rect, cv::Scalar(0, 0, 255), -1);

    // 카메라 박스 그리기 (마커 위치 기반)
    int camera_x = charger_x - static_cast<int>(smoothed_z * 150) - 90;
    int camera_y = static_cast<int>((-smoothed_x) * 100) + current_frame_.rows / 2 - 10;  
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
    ros::Subscriber pointcloud_sub_; 


    ros::Publisher cmd_vel_pub_; 
    ros::Timer timer_;

    ros::Time last_pose_time_;
    bool last_pose_received_;
    bool pose_received; 
    bool aruco_received;
    bool charge_flag;
    double max_speed, min_speed, slow_distance, speed_factor;
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

    ros::Rate rate(30);  

    while (ros::ok()) {
        controller.controlLoop();  // 클래스 메서드 호출 시 객체를 통해 접근
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}