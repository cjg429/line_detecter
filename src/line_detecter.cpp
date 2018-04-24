#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <opencv2/opencv.hpp>
#include "laser_scan_matcher/laser_scan_matcher.h"

#include "message_filters/subscriber.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include "line_detecter/Tree.h"

#include <iostream>
#include <fstream>
#include <ctime>
#define PI 3.14159265358979323846

int save_idx = 0;

class DetectNode {
	public:
		DetectNode();
		~DetectNode();
	private:
		ros::NodeHandle nh_;
		tf::TransformListener* tf_;
		
		ros::Subscriber pos_sub_;
		ros::Subscriber front_laser_sub_;
		ros::Subscriber back_laser_sub_;
		void frontScanCallback(const sensor_msgs::LaserScanConstPtr& laser_scan);
		void backScanCallback(const sensor_msgs::LaserScanConstPtr& laser_scan);
		void PosCallback(const geometry_msgs::Pose2DConstPtr& pos_msg);
		void splitMerge(std::vector<std::vector<float>> points);
		void split(std::vector<std::vector<float>> points);
		
		float robot_pose_x_, robot_pose_y_, robot_pose_yaw_;
		tf::Stamped<tf::Pose> latest_odom_pose_;
		
		std::string front_scan_topic_, back_scan_topic_, pos_topic_;
		std::string base_frame_id_, odom_frame_id_, global_frame_id_;
		cv::Mat map_c_;
		int PLOT_FLAG_;
		int numLane_;
		int leftLane_;
		int rightLane_;
		int nx_, ny_;
		float thDist_;
		float L_;
		float rf_dist_, cf_dist_, lf_dist_, rb_dist_, cb_dist_, lb_dist_, lane_devdist_, headdeg_, pos_th_;
		Tree* A_;
		Tree* B_;
		std::ofstream myfile_;
};

boost::shared_ptr<DetectNode> detect_node_ptr;

int main(int argc, char** argv) {
	ros::init(argc, argv, "car_detect_node");
	ros::NodeHandle nh;
	detect_node_ptr.reset(new DetectNode());
	if (argc == 1) {
		ros::spin();
  }
	detect_node_ptr.reset();
	return 0;
};

DetectNode::DetectNode() {
	nh_.param("front_scan_topic", front_scan_topic_, std::string("scan"));
	nh_.param("back_scan_topic", back_scan_topic_, std::string("back_scan"));
	nh_.param("pos_topic", pos_topic_, std::string("pose2D"));
	nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
	nh_.param<int>("num_lane", numLane_, 4);
	nh_.param<int>("left_lane", leftLane_, 1);
	nh_.param<int>("right_lane", rightLane_, 3);
	nh_.param<float>("car_length", L_, 0.1);
	front_laser_sub_ = nh_.subscribe(front_scan_topic_, 10, &DetectNode::frontScanCallback, this);
	back_laser_sub_ = nh_.subscribe(back_scan_topic_, 10, &DetectNode::backScanCallback, this);
	pos_sub_ = nh_.subscribe(pos_topic_, 10, &DetectNode::PosCallback, this);
	PLOT_FLAG_ = 1;
	thDist_ = 3;
	A_ = new Tree(front_scan_topic_, numLane_, leftLane_, rightLane_, L_);
	B_ = new Tree(back_scan_topic_, numLane_, leftLane_, rightLane_, L_);
	geometry_msgs::Pose2D pose;
	
	try {
  	pose = *(ros::topic::waitForMessage<geometry_msgs::Pose2D>("pose2D", ros::Duration(3)));
  }
  catch(std::exception e) {
  	ROS_WARN("No pose message!");
  }
  robot_pose_x_ = pose.x;
  robot_pose_y_ = pose.y;
  robot_pose_yaw_ = pose.theta;
}

DetectNode::~DetectNode() {
}

void DetectNode::frontScanCallback(const sensor_msgs::LaserScanConstPtr& laser_scan) {
  int range_count = laser_scan->ranges.size();
  float angle_increment = laser_scan->angle_increment;
	float angle_min = laser_scan->angle_min;
	float range_max = laser_scan->range_max;
	float range_min = laser_scan->range_min;
	float ranges[range_count][2];
	std::vector<std::vector<float>> lsr_points;
	
	int beam_skip = 1;
	
  for(int i = 0; i < range_count; i = i+beam_skip) {
		float dist;
		if(laser_scan->ranges[i] <= range_min)
			dist = range_max;
		else
			dist = laser_scan->ranges[i];
		float angle = angle_min+i*angle_increment;
		if(angle > -PI/2 && angle < PI/2) {
			float x = dist*cos(angle);
			float y = dist*sin(angle);
			std::vector<float> point;
			point.push_back(x);
			point.push_back(y);
			lsr_points.push_back(point);
		}
	}
	A_->newScan(lsr_points, (float)0.1, robot_pose_x_, robot_pose_y_, robot_pose_yaw_);
	A_->generateTree();
	std::tie(lf_dist_, cf_dist_, rf_dist_, lane_devdist_, headdeg_, pos_th_) = A_->extractFeatures();
	ROS_INFO("FRONT : %f %f %f %f %f %f\n", lf_dist_, cf_dist_, rf_dist_, lane_devdist_, headdeg_, pos_th_);
	A_->visualizeFitLine();
	A_->cleanAll();
}

void DetectNode::backScanCallback(const sensor_msgs::LaserScanConstPtr& laser_scan) {
  int range_count = laser_scan->ranges.size();
  float angle_increment = laser_scan->angle_increment;
	float angle_min = laser_scan->angle_min;
	float range_max = laser_scan->range_max;
	float range_min = laser_scan->range_min;
	float ranges[range_count][2];
	std::vector<std::vector<float>> lsr_points;
	
	int beam_skip = 1;
	
  for(int i = 0; i < range_count; i = i+beam_skip) {
		float dist;
		if(laser_scan->ranges[i] <= range_min)
			dist = range_max;
		else
			dist = laser_scan->ranges[i];
		float angle = angle_min+i*angle_increment;
		if(angle > -PI/2 && angle < PI/2) {
			float x = dist*cos(angle);
			float y = dist*sin(angle);
			std::vector<float> point;
			point.push_back(x);
			point.push_back(y);
			lsr_points.push_back(point);
		}
	}
	B_->newScan(lsr_points, (float)0.1, robot_pose_x_, robot_pose_y_, robot_pose_yaw_);
	B_->generateTree();
	float lane_devdist, headdeg, pos_th;
	std::tie(rb_dist_, cb_dist_, lb_dist_, lane_devdist, headdeg, pos_th) = B_->extractFeatures();
	ROS_INFO("BACK : %f %f %f\n", lb_dist_, cb_dist_, rb_dist_);
	B_->extractFeatures();
	B_->cleanAll();
}

void DetectNode::PosCallback(const geometry_msgs::Pose2DConstPtr& msg) {
	robot_pose_x_ = msg->x;
	robot_pose_y_ = msg->y;
	robot_pose_yaw_ = msg->theta;
}
