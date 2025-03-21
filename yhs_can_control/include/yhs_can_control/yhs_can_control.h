#ifndef __CANCONTROL_NODE_H__
#define __CANCONTROL_NODE_H__



#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Float32MultiArray.h>
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "yhs_can_msgs/ctrl_cmd.h"
#include "yhs_can_msgs/io_cmd.h"
#include "yhs_can_msgs/ctrl_fb.h"
#include "yhs_can_msgs/lr_wheel_fb.h"
#include "yhs_can_msgs/rr_wheel_fb.h"
#include "yhs_can_msgs/lf_wheel_fb.h"
#include "yhs_can_msgs/rf_wheel_fb.h"
#include "yhs_can_msgs/io_fb.h"
#include "yhs_can_msgs/steering_ctrl_cmd.h"
#include "yhs_can_msgs/front_angle_free_ctrl_cmd.h"
#include "yhs_can_msgs/front_velocity_free_ctrl_cmd.h"
#include "yhs_can_msgs/rear_angle_free_ctrl_cmd.h"
#include "yhs_can_msgs/rear_velocity_free_ctrl_cmd.h"
#include "yhs_can_msgs/bms_fb.h"
#include "yhs_can_msgs/bms_flag_fb.h"
#include "yhs_can_msgs/steering_ctrl_fb.h"
#include "yhs_can_msgs/front_angle_fb.h"
#include "yhs_can_msgs/rear_angle_fb.h"
#include "yhs_can_msgs/ultrasonic.h"


#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>


#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>


namespace yhs_tool {
class CanControl
{
public:
	CanControl();
	~CanControl();
	
	void run();
private:
	ros::NodeHandle nh_;

	ros::Publisher ctrl_fb_pub_;
	ros::Publisher lr_wheel_fb_pub_;
	ros::Publisher rr_wheel_fb_pub_;
	ros::Publisher io_fb_pub_;
	ros::Publisher rf_wheel_fb_pub_;
	ros::Publisher lf_wheel_fb_pub_;
	ros::Publisher bms_fb_pub_;
	ros::Publisher bms_flag_fb_pub_;
	ros::Publisher steering_ctrl_fb_pub_;
	ros::Publisher front_angle_fb_pub_;
	ros::Publisher rear_angle_fb_pub_;

	ros::Publisher motor_angle_state_pub;
	ros::Publisher odom_pub_;
    ros::Publisher angular_pub_;
	ros::Publisher linear_pub_;
	ros::Publisher ultrasonic_pub_;
	ros::Publisher scan_pub_;

	ros::Subscriber ctrl_cmd_sub_;
	ros::Subscriber cmd_sub_;
	ros::Subscriber io_cmd_sub_;
	ros::Subscriber steering_ctrl_cmd_sub_;
	ros::Subscriber front_angle_free_ctrl_cmd_sub_;
	ros::Subscriber front_velocity_free_ctrl_cmd_sub_;
	ros::Subscriber rear_angle_free_ctrl_cmd_sub_;
	ros::Subscriber rear_velocity_free_ctrl_cmd_sub_;
	ros::Subscriber scan_sub;

	boost::mutex cmd_mutex_;


	std::string odomFrame_, baseFrame_;
	bool tfUsed_;
	bool have_cmd_vel_ = false;
	bool lidar_distance_exception = false;

	unsigned char sendData_u_io_[8] = {0};
	unsigned char sendData_u_vel_[8] = {0};

	double front_angle_left, front_angle_right;
	double rear_angle_left, rear_angle_right;
	double lidar_distance;
    ros::Time ld1_timer;
	ros::Time ld2_timer;
		
	int dev_handler_;
	can_frame send_frames_[2];
	can_frame recv_frames_[1];


	void io_cmdCallBack(const yhs_can_msgs::io_cmd msg);
	void ctrl_cmdCallBack(const yhs_can_msgs::ctrl_cmd msg);
	void steering_ctrl_cmdCallBack(const yhs_can_msgs::steering_ctrl_cmd msg);
	void front_angle_free_ctrl_cmdCallBack(const yhs_can_msgs::front_angle_free_ctrl_cmd msg);
	void front_velocity_free_ctrl_cmdCallBack(const yhs_can_msgs::front_velocity_free_ctrl_cmd msg);
	void rear_angle_free_ctrl_cmdCallBack(const yhs_can_msgs::rear_angle_free_ctrl_cmd msg);
	void rear_velocity_free_ctrl_cmdCallBack(const yhs_can_msgs::rear_velocity_free_ctrl_cmd msg);
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);


	void cmdCallBack(const geometry_msgs::Twist msg);
	void odomPub(float linear,float angular);


	void recvData();
	void sendData();

};

}


#endif
