#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_info_publisher");
    ros::NodeHandle nh;

    ros::Publisher camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/camera/color/camera_info", 1);

    // 새로운 카메라 캘리브레이션 값 설정
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.stamp = ros::Time::now();
    camera_info.header.frame_id = "camera_color_optical_frame";
    
    // 예시 값 (본인의 카메라 캘리브레이션 값으로 수정)
    camera_info.K = {895.8520882881493, 0.0, 638.4792659155581, 0.0, 897.4266417756905, 346.0029948024051, 0.0, 0.0, 1.0};
    camera_info.D = {0.10410568094167576, -0.1878360784443046, -0.0006822296204200654, -0.004111776380668183, 0.0};
    camera_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    camera_info.P = {899.7741385393227, 0.0, 632.273277678556, 0.0, 0.0, 912.8644656286453, 345.64493508457383, 0.0, 0.0, 0.0, 1.0, 0.0};
    
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        // 새 카메라 정보 퍼블리시
        camera_info_pub.publish(camera_info);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

