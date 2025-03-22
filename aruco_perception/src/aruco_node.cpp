#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>  // 속도 명령 메시지
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <tf/transform_datatypes.h>  
#include <tf/transform_broadcaster.h>

class ArUcoDetector {
public:
    ArUcoDetector() {
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("aruco_dictionary", aruco_dict_name_, "DICT_4X4_250");
        private_nh.param<bool>("display_output", display_output_, true);

        camera_info_sub_ = nh_.subscribe("/camera/color/camera_info", 1, &ArUcoDetector::cameraInfoCallback, this);
        image_sub_ = nh_.subscribe("/camera/color/image_raw", 1, &ArUcoDetector::imageCallback, this);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/aruco_pose", 1);

        ROS_INFO("Using ArUco dictionary: %s", aruco_dict_name_.c_str());
        ROS_INFO("display_output : %d", display_output_);

        aruco_dict_ = cv::aruco::getPredefinedDictionary(getArucoDictionary(aruco_dict_name_));
        aruco_params_ = cv::Ptr<cv::aruco::DetectorParameters>(cv::aruco::DetectorParameters::create());
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
        camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->K.data())).clone();
        dist_coeffs_ = cv::Mat(1, 5, CV_64F, const_cast<double*>(msg->D.data())).clone();

        // 카메라 매트릭스와 왜곡 계수 출력
        // ROS_INFO("Camera Matrix: \n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]",
        //          camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(0, 1), camera_matrix_.at<double>(0, 2),
        //          camera_matrix_.at<double>(1, 0), camera_matrix_.at<double>(1, 1), camera_matrix_.at<double>(1, 2),
        //          camera_matrix_.at<double>(2, 0), camera_matrix_.at<double>(2, 1), camera_matrix_.at<double>(2, 2));
        
        // ROS_INFO("Distortion Coefficients: \n[%f, %f, %f, %f, %f]",
        //          dist_coeffs_.at<double>(0), dist_coeffs_.at<double>(1), dist_coeffs_.at<double>(2),
        //          dist_coeffs_.at<double>(3), dist_coeffs_.at<double>(4));
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
            cv::Mat frame_undistorted;
            cv::undistort(frame, frame_undistorted, camera_matrix_, dist_coeffs_);

            // 왜곡 보정된 이미지 그대로 사용
            cv::Mat frame_resized = frame_undistorted;

            std::vector<std::vector<cv::Point2f>> corners, rejected;
            std::vector<int> ids;
            cv::aruco::detectMarkers(frame_resized, aruco_dict_, corners, ids, aruco_params_, rejected);

            // 새로운 변수로 이전 위치와 회전 값을 저장
            static double last_x = 0.0, last_y = 0.0, last_z = 0.0;
            static double last_rx = 0.0, last_ry = 0.0, last_rz = 0.0;

            if (!corners.empty()) {
                for (size_t i = 0; i < corners.size(); ++i) {
                    for (size_t j = 0; j < 4; ++j) {
                        cv::circle(frame_resized, corners[i][j], 8, cv::Scalar(255, 0, 0), -1);
                    }

                    cv::Mat rvec, tvec;
                    std::vector<cv::Point3f> marker_3d_edges = {
                        cv::Point3f(-25, -25, 0),  // 왼쪽 하단
                        cv::Point3f(25, -25, 0),   // 오른쪽 하단
                        cv::Point3f(25, 25, 0),    // 오른쪽 상단
                        cv::Point3f(-25, 25, 0)    // 왼쪽 상단
                    };

                    bool ret = cv::solvePnP(marker_3d_edges, corners[i], camera_matrix_, dist_coeffs_, rvec, tvec);

                    if (ret) {
                        double x = tvec.at<double>(0) / 1000.0;  // x 값 (m)
                        double y = tvec.at<double>(1) / 1000.0;  // y 값 (m)
                        double z = tvec.at<double>(2) / 1000.0;  // z 값 (m)

                        double rx = rvec.at<double>(0);  // 회전 (radians)
                        double ry = rvec.at<double>(1);
                        double rz = rvec.at<double>(2);

                        // 위치 값 변화가 작은 경우 스케일링 처리
                        if (std::abs(x - last_x) < 0.01) x = last_x;
                        if (std::abs(y - last_y) < 0.01) y = last_y;
                        if (std::abs(z - last_z) < 0.01) z = last_z;

                        // 회전 값 변화가 작은 경우 스케일링 처리
                        if (std::abs(rx - last_rx) < 1.0) rx = last_rx;
                        if (std::abs(ry - last_ry) < 1.0) ry = last_ry;
                        if (std::abs(rz - last_rz) < 1.0) rz = last_rz;

                        last_x = x;
                        last_y = y;
                        last_z = z;
                        last_rx = rx;
                        last_ry = ry;
                        last_rz = rz;

                        // rvec을 쿼터니언으로 변환
                        cv::Mat R;
                        cv::Rodrigues(rvec, R);
                        // 회전 행렬을 tf::Matrix3x3으로 변환
                        tf::Matrix3x3 tf_R(
                            R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2)
                        );

          
                        tf::Quaternion q;
                        q.setRPY(rx, ry, rz); 
                        // 쿼터니언을 정규화 (정규화되지 않으면 TF에서 경고 발생)
                        q.normalize();


                        // Pose 메시지 생성
                        geometry_msgs::PoseStamped pose_msg;
                        pose_msg.header.stamp = ros::Time::now();
                        pose_msg.header.frame_id = "aruco_marker";
                        pose_msg.pose.position.x = -x; 
                        pose_msg.pose.position.y = y;
                        pose_msg.pose.position.z = z;
                        pose_msg.pose.orientation.x = q.x();  
                        pose_msg.pose.orientation.y = q.y();
                        pose_msg.pose.orientation.z = q.z();
                        pose_msg.pose.orientation.w = q.w();

                        // TF 변환 브로드캐스트 
                        tf::Transform transform;
                        transform.setOrigin(tf::Vector3(z, -x, y));  
                        transform.setRotation(q);  // 정규화된 회전값 사용
                        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "depth_link", "aruco_marker"));


                        // Pose 메시지 퍼블리시
                        pose_pub_.publish(pose_msg);

                        // 아루코 축 그리기
                        cv::aruco::drawAxis(frame_resized, camera_matrix_, dist_coeffs_, rvec, tvec, 100);

                        // 위치 및 회전 텍스트 생성
                        std::string pos_text = "Pos: (" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ") m";
                        std::string rot_text = "Rot: (" + std::to_string(rx) + ", " + std::to_string(ry) + ", " + std::to_string(rz) + ") rad";

                        cv::Point pos_loc(2, 710);   // 위치 텍스트 좌표
                        cv::Point rot_loc(620, 710);   // 회전 텍스트 좌표

                        double font_scale = 0.7;
                        int thickness = 2;
                        cv::Scalar text_color(255, 150, 50);
                        cv::Scalar outline_color(0, 0, 0);

                        cv::putText(frame_resized, pos_text, pos_loc, cv::FONT_HERSHEY_SIMPLEX, font_scale, outline_color, thickness + 2);
                        cv::putText(frame_resized, rot_text, rot_loc, cv::FONT_HERSHEY_SIMPLEX, font_scale, outline_color, thickness + 2);

                        cv::putText(frame_resized, pos_text, pos_loc, cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
                        cv::putText(frame_resized, rot_text, rot_loc, cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
                    }
                }
            }

            if (display_output_) {
                cv::imshow("ArUco Marker Detection", frame_resized);
                cv::waitKey(1);
            }
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber camera_info_sub_;
    ros::Subscriber image_sub_;
    ros::Publisher pose_pub_;
    
    tf::TransformBroadcaster br_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;

    std::string aruco_dict_name_;
    bool display_output_;

    int getArucoDictionary(const std::string& dict_name) {
        if (dict_name == "DICT_4X4_50") return cv::aruco::DICT_4X4_50;
        if (dict_name == "DICT_4X4_250") return cv::aruco::DICT_4X4_250;
        if (dict_name == "DICT_5X5_50") return cv::aruco::DICT_5X5_50;
        if (dict_name == "DICT_5X5_250") return cv::aruco::DICT_5X5_250;
        if (dict_name == "DICT_6X6_50") return cv::aruco::DICT_6X6_50;
        if (dict_name == "DICT_6X6_250") return cv::aruco::DICT_6X6_250;
        if (dict_name == "DICT_7X7_50") return cv::aruco::DICT_7X7_50;
        if (dict_name == "DICT_7X7_250") return cv::aruco::DICT_7X7_250;
        return cv::aruco::DICT_4X4_250; // 기본값
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_detector");
    ArUcoDetector detector;
    ros::spin();
    return 0;
}
