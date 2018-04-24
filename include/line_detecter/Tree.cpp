#include "Tree.h"
#include <unistd.h>
#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <time.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#define PI 3.14159265358979323846
#define DEG2RAD 0.017453293f

Tree::Tree() {
	requestWall();
}

Tree::Tree(std::string scanName, int numlane, int leftlane, int rightlane, float L) {
	scanName_ = scanName;
	numLane_ = numlane;
	leftLane_ = leftlane;
	rightLane_ = rightlane;
	L_ = L;
	requestWall();
}

Tree::~Tree() {
}

void Tree::newScan(std::vector<std::vector<float>> points, float thDist, float poseX, float poseY, float poseTh) {
  currPose_[0] = poseX;
  currPose_[1] = poseY;
  currPose_[2] = poseTh;
  float delta_x = currPose_[0]-initPose_[0];
  float delta_y = currPose_[1]-initPose_[1];
  float delta_th = currPose_[2]-initPose_[2];
  
  for(int i = 0; i < points.size(); i++) {
  	float x = points[i][0]*cos(delta_th)-points[i][1]*sin(delta_th);
  	float y = points[i][0]*sin(delta_th)+points[i][1]*cos(delta_th)-(delta_x*sin(initPose_[2])-delta_y*cos(initPose_[2]));
  	std::vector<float> point;
  	point.push_back(x);
  	point.push_back(y);
  	points_.push_back(point);
  }
  
	thDist_ = thDist;
	start_ = 0;
	end_ = points.size()-1;
}

void Tree::generateTree()
{
  //cleanAll();
	split(start_, end_);
	fitting();
	merge();
	carDetect();
	//extractFeatures();
}

void Tree::split(int start, int end)
{
	float maxDist = -1;
	int idx = end+1;
	for(int i = start; i < end+1; i++) {
		float Dist;
		float x0 = points_[i][0];
		float y0 = points_[i][1];
		float x1 = points_[start][0];
		float y1 = points_[start][1];
		float x2 = points_[end][0];
		float y2 = points_[end][1];
		if(sqrt(pow(y2-y1,2)+pow(x2-x1,2)) == 0) Dist = 0;
		else {
			Dist = fabs((y2-y1)*x0-(x2-x1)*y0+x2*y1-y2*x1)/sqrt(pow(y2-y1,2)+pow(x2-x1,2));
		}
		if(Dist > maxDist) {
			maxDist = Dist;
			idx = i;
		}
	}
	if(maxDist > thDist_) {
		split(start,idx);
		split(idx,end);
	}
	else {
		float deltaX = points_[end][0]-points_[start][0];
		float deltaY = points_[end][1]-points_[start][1];
		float dist = sqrt(pow(deltaX,2)+pow(deltaY,2));
		if(dist > 0.09 && end-start+1 > 10) {
		  float maxDist = 0;
			for(int i = start; i < end; i++) {
				float deltaX = points_[i+1][0]-points_[i][0];
				float deltaY = points_[i+1][1]-points_[i][1];
				float dist = sqrt(pow(deltaX,2)+pow(deltaY,2));
				if(dist > maxDist) maxDist = dist;
			}
			if(maxDist < 0.15 || end-start+1 > 50) {
				std::vector<int> segment;
				segment.push_back(start);
				segment.push_back(end);
				segments_.push_back(segment);
			}
		}
	}
}

void Tree::fitting()
{
	int size = segments_.size();
	for(int i = 0; i < size; i++) {
		int beginIdx = segments_[i][0];
		int endIdx = segments_[i][1];
		int numPoints = endIdx-beginIdx+1;
		float sumX = 0, sumY = 0;
		float meanX, meanY;
		float weightX, weightY;
		float alpha;
		float r;
		for(int j = beginIdx; j < endIdx+1; j++) {
			sumX = sumX+points_[j][0];
			sumY = sumY+points_[j][1];
		}
		weightX = sumX/numPoints;
		weightY = sumY/numPoints;
		
		float denominator = 0, numerator = 0;
		for(int j = beginIdx; j < endIdx+1; j++) {
			numerator = numerator-2*(weightX-points_[j][0])*(weightY-points_[j][1]);
			denominator = denominator+pow(weightY-points_[j][1],2)-pow(weightX-points_[j][0],2);
		}
		alpha = 0.5*atan2(numerator,denominator);
		r = weightX*cos(alpha)+weightY*sin(alpha);
		if(alpha < 0) {
			alpha = PI+alpha;
			r = -r;
		}
		r_.push_back(r);
		theta_.push_back(alpha);
	}
}

void Tree::carDetect()
{
	int size = segments_.size();
	int start = 0;
	int end = segments_.size()-1;
	while(1) {
		if(start == size-1) break;
		if(segments_[start][1] == segments_[start+1][0]) {
			start = start+1;
		}
		else {
			start = start+1;
			break;
		}
	}
	
	while(1) {
		if(end == 0) break;
		if(segments_[end][0] == segments_[end-1][1]) {
			end = end-1;
		}
		else {
			end = end-1;
			break;
		}
	}
	for(int i = start; i < end+1; i++) {
		std::vector<int> car;
		car.push_back(segments_[i][0]);
		car.push_back(segments_[i][1]);
		car_.push_back(car);		
	}

	if(!car_.empty()) {
		int merge_size = merge_r_.size();
		int idxLeft = merge_size;
		int idxRight = merge_size;
		int idxFront = merge_size;
		float maxY = -1E18;
		float minY = 1E18;
		float maxX = -1E18;
		for(int i = 0; i < merge_size; i++) {
			float y = merge_r_[i]*sin(merge_theta_[i]);
			float x = merge_r_[i]*cos(merge_theta_[i]);
			if(y > maxY && fabs(merge_theta_[i]-PI/2) < PI/10 && fabs(merge_r_[i]-leftWall_[0]) < 0.2 && fabs(merge_theta_[i]-leftWall_[1]) < 0.2) {
				maxY = y;
				idxLeft = i;
			}
			if(y < minY && fabs(merge_theta_[i]-PI/2) < PI/10 && fabs(merge_r_[i]-rightWall_[0]) < 0.2 && fabs(merge_theta_[i]-rightWall_[1]) < 0.2) {
				minY = y;
				idxRight = i;
			}	
			if(x > maxX) {
				maxX = x;
				idxFront = i;
			}
		}
		if(idxRight == merge_size) minY = -1E18;
		if(idxLeft == merge_size) maxY = 1E18;
		if(idxRight != merge_size) {
			rightLine_.push_back(merge_r_[idxRight]);
			rightLine_.push_back(merge_theta_[idxRight]);
		}
		if(idxLeft != merge_size) {
			leftLine_.push_back(merge_r_[idxLeft]);
			leftLine_.push_back(merge_theta_[idxLeft]);
		}
		return;
	}
	else {
		int merge_size = merge_r_.size();
		int idxLeft = merge_size;
		int idxRight = merge_size;
		int idxFront = merge_size;
		float maxY = -1E18;
		float minY = 1E18;
		float maxX = -1E18;
		for(int i = 0; i < merge_size; i++) {
			float y = merge_r_[i]*sin(merge_theta_[i]);
			float x = merge_r_[i]*cos(merge_theta_[i]);
			if(y > maxY && fabs(merge_theta_[i]-PI/2) < PI/10 && fabs(merge_r_[i]-leftWall_[0]) < 0.2 && fabs(merge_theta_[i]-leftWall_[1]) < 0.2) {
				maxY = y;
				idxLeft = i;
			}
			if(y < minY && fabs(merge_theta_[i]-PI/2) < PI/10 && fabs(merge_r_[i]-rightWall_[0]) < 0.2 && fabs(merge_theta_[i]-rightWall_[1]) < 0.2) {
				minY = y;
				idxRight = i;
			}	
			if(x > maxX) {
				maxX = x;
				idxFront = i;
			}
		}
		if(idxRight == merge_size) {
			rightLine_.push_back(rightWall_[0]);
			rightLine_.push_back(rightWall_[1]);
			minY = rightLine_[0]*sin(rightLine_[1]);	
			ROS_INFO("no right wall %f %f", rightWall_[0], rightWall_[1]);
		}	
		if(idxLeft == merge_size) {
			leftLine_.push_back(leftWall_[0]);
			leftLine_.push_back(leftWall_[1]);
			maxY = leftLine_[0]*sin(leftLine_[1]);
			ROS_INFO("no left wall %f %f", leftWall_[0], leftWall_[1]); 
		}	
		if(idxRight != merge_size) {
			rightLine_.push_back(merge_r_[idxRight]);
			rightLine_.push_back(merge_theta_[idxRight]);
		}
		if(idxLeft != merge_size) {
			leftLine_.push_back(merge_r_[idxLeft]);
			leftLine_.push_back(merge_theta_[idxLeft]);
		}
		for(int i = 0; i < merge_size; i++) {
			float y = merge_r_[i]*sin(merge_theta_[i]);
			float x = merge_r_[i]*cos(merge_theta_[i]);
			if(fabs(y-maxY) > 0.5 && fabs(y-minY) > 0.5 && fabs(x-maxX) > 0.5) {
				float r1 = merge_r_[i];
				float th1 = merge_theta_[i];
				for(int j = 0; j < size; j++) {
					float r2 = r_[j];
					float th2 = theta_[j];
					if(fabs(r1-r2) < 0.5 && fabs(th1-th2) < 0.5) {
					  std::vector<int> car;
						car.push_back(segments_[j][0]);
						car.push_back(segments_[j][1]);
						car_.push_back(car);
					}		
				}
			}
		}		
	}
}

void Tree::visualizeFitLine() {
  cv::Mat img;
  img = cv::Mat::zeros(500,500,CV_8UC3);
  int img_w = 500;
  int img_h = 500;
	int size = merge_r_.size();
  float delta_x = currPose_[0]-initPose_[0];
  float delta_y = currPose_[1]-initPose_[1];
  float delta_th = currPose_[2]-initPose_[2];
  
  // DRAW MY CAR WITH AN ARROWED LINE
  float x1 = -0.2;
  float y1 = -(delta_x*sin(initPose_[2])-delta_y*cos(initPose_[2]));
  float x2 = 0.2;
  float y2 = -(delta_x*sin(initPose_[2])-delta_y*cos(initPose_[2]));
  float point1x = x1*cos(delta_th)-y1*sin(delta_th);
 	float point1y = x1*sin(delta_th)+y1*cos(delta_th);
 	float point2x = x2*cos(delta_th)-y2*sin(delta_th);
 	float point2y = x2*sin(delta_th)+y2*cos(delta_th);
 	cv::arrowedLine(img, cv::Point((int)(50*point1x+250), (int)(250-50*point1y)), cv::Point((int)(50*point2x+250), (int)(250-50*point2y)), cv::Scalar(0,255,0), 3, 8, 0, 0.4);
		
	// DRAW LASER END POINTS	
	for(int i = 0; i < points_.size(); i++) { 
		int x_point = points_[i][0]*50+250;
		int y_point = -points_[i][1]*50+250;
		cv::circle(img, cv::Point(x_point, y_point), 1, cv::Scalar(0,0,255), CV_FILLED);
	}
	
  // DRAW ANOTHER CAR
	if(!car_.empty()) {
		int car_size = car_.size();
		int minIdx = -1;
		int minDist = 1E6;
		for(int i = 0; i < car_size; i++) {
			int beginIdx = car_[i][0];
			int endIdx = car_[i][1];
			for(int j = beginIdx; j <= endIdx; j++) {
				int x_point = points_[j][0]*50+250;
				int y_point = -points_[j][1]*50+250;
				if(fabs(points_[j][0]*cos(leftLine_[1])+points_[j][1]*sin(leftLine_[1])-leftLine_[0]) < 0.2) break;
				if(fabs(points_[j][0]*cos(rightLine_[1])+points_[j][1]*sin(rightLine_[1])-rightLine_[0]) < 0.2) break;
				if(pow(x_point, 2)+pow(y_point, 2) < minDist) {
					minIdx = j;
					minDist = pow(x_point, 2)+pow(y_point, 2);
				}
			}	
		}
		int x_point = points_[minIdx][0]*50+250;
		int y_point = -points_[minIdx][1]*50+250;
		cv::circle(img, cv::Point(x_point, y_point), 8, cv::Scalar(0,255,255), CV_FILLED);
	}
	
	float r_left = leftLine_[0];
	float r_right = rightLine_[0];
	float th_left = leftLine_[1];
	float th_right = rightLine_[1];
	
	// DRAW LANES
	for(int i = 1; i < numLane_; i++) {
		std::string Text;
		Text.append(std::to_string(i));
		float r = r_left+(r_right-r_left)*i/numLane_;
		float t = th_left+(th_right-th_left)*i/numLane_;
		int pointX = 250;
	  int pointY;
		if(t >= PI/4 && t <= 3*PI/4) {
				int x1 = 0;
				int y1 = -(x1-r*cos(t)*50-img_w/2)/tan(t)+r*sin(t)*50+img_h/2;
				int x2 = img_w;
				int y2 = -(x2-r*cos(t)*50-img_w/2)/tan(t)+r*sin(t)*50+img_h/2;
				if(i == leftLane_ || i == rightLane_) {
					cv::line(img, cv::Point(x1, 500-y1), cv::Point(x2, 500-y2), cv::Scalar(255,255,255), 2);
				}
				else {
					cv::line(img, cv::Point(x1, 500-y1), cv::Point(x2, 500-y2), cv::Scalar(255,255,255), 1);
				}
				pointY = -(250-r*cos(t)*50-img_w/2)/tan(t)+r*sin(t)*50+img_h/2;
				//cv::putText(img, Text, cv::Point(pointX, pointY), 2, 1.2, cv::Scalar(255,255,255));
		}
		else {
				int y1 = 0;
				int x1 = -(y1-r*sin(t)*50-img_h/2)*tan(t)+r*cos(t)*50+img_w/2;
				int y2 = img_h;
				int x2 = -(y2-r*sin(t)*50-img_h/2)*tan(t)+r*cos(t)*50+img_w/2;
				if(i == leftLane_ || i == rightLane_) {
					cv::line(img, cv::Point(x1, 500-y1), cv::Point(x2, 500-y2), cv::Scalar(255,255,255), 2);
				}
				else {
					cv::line(img, cv::Point(x1, 500-y1), cv::Point(x2, 500-y2), cv::Scalar(255,255,255), 1);
				}
		}
	}
	cv::namedWindow("Mapping");
	cv::imshow("Mapping", img);
	cv::waitKey(1);
}

void Tree::merge()
{
	int size = segments_.size();
	std::vector<bool> table;
	for(int i = 0; i < size; i++) {
		table.push_back(true);
	}
	for(int i = 0; i < size; i++) {
		float r1 = r_[i];
		float th1 = theta_[i];
		for(int j = i+1; j < size; j++) {
			float r2 = r_[j];
			float th2 = theta_[j];
			if(fabs(r1-r2) < 0.5 && fabs(th1-th2) < 0.5) {
				int numPoints = 0;
				float sumX = 0, sumY = 0;
				float meanX, meanY;
				float weightX, weightY;
				float alpha;
				float r;
				for(int k = segments_[i][0]; k < segments_[i][1]+1; k++) {
					sumX = sumX+points_[k][0];
					sumY = sumY+points_[k][1];
					numPoints++;
				}
				for(int k = segments_[j][0]; k < segments_[j][1]+1; k++) {
					sumX = sumX+points_[k][0];
					sumY = sumY+points_[k][1];
					numPoints++;
				}
				weightX = sumX/numPoints;
				weightY = sumY/numPoints;
				float denominator = 0, numerator = 0;
				for(int k = segments_[i][0]; k < segments_[i][1]+1; k++) {
					numerator = numerator-2*(weightX-points_[k][0])*(weightY-points_[k][1]);
					denominator = denominator+pow(weightY-points_[k][1],2)-pow(weightX-points_[k][0],2);
				}
				for(int k = segments_[j][0]; k < segments_[j][1]+1; k++) {
					numerator = numerator-2*(weightX-points_[k][0])*(weightY-points_[k][1]);
					denominator = denominator+pow(weightY-points_[k][1],2)-pow(weightX-points_[k][0],2);
				}
				alpha = 0.5*atan2(numerator,denominator);
				r = weightX*cos(alpha)+weightY*sin(alpha);
				if(alpha < 0) {
					alpha = PI+alpha;
					r = -r;
				}
				merge_r_.push_back(r);	  
				merge_theta_.push_back(alpha);
				table[i] = false;
				table[j] = false;
				break;
			}
		}
	}
	for(int i = 0; i < size; i++) {
		if(table[i]) {
			merge_r_.push_back(r_[i]);
			merge_theta_.push_back(theta_[i]);
		}
	}
}

void Tree::requestWall()
{
  // get map via RPC
  geometry_msgs::Pose2D pose;
  ROS_INFO("Requesting the wall...");
  try {
  	pose = *(ros::topic::waitForMessage<geometry_msgs::Pose2D>("pose2D", ros::Duration(3)));
  }
  catch(std::exception e) {
  	ROS_WARN("No pose message!");
  }
  ROS_INFO("Initial Pose(x,y,th) : %f,%f,%f", pose.x, pose.y, pose.theta);
  initPose_[0] = pose.x;
  initPose_[1] = pose.y;
  initPose_[2] = pose.theta;
  sensor_msgs::LaserScan laser;
  try {
  	laser = *(ros::topic::waitForMessage<sensor_msgs::LaserScan>(scanName_, ros::Duration(3)));
  }
  catch(std::exception e) {
  	ROS_WARN("No laser message!");
  }
  getWall(laser, initPose_[0], initPose_[1], initPose_[2]);
}

void Tree::getWall(const sensor_msgs::LaserScan& laser_scan, float poseX, float poseY, float poseYaw)
{
  int range_count = laser_scan.ranges.size();
  double angle_increment = laser_scan.angle_increment;
	double angle_min = laser_scan.angle_min;
	double range_max = laser_scan.range_max;
	double range_min = laser_scan.range_min;
	double ranges[range_count][2];
	std::vector<std::vector<float>> lsr_points;
	
	int beam_skip = 1;
	
  for(int i = 0; i < range_count; i = i+beam_skip) {
		float dist;
		if(laser_scan.ranges[i] <= range_min)
			dist = range_max;
		else
			dist = laser_scan.ranges[i];
			
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
	newScan(lsr_points, (float)0.1, poseX, poseY, poseYaw);
	split(start_, end_);
	fitting();
	merge();
	wallDetect();
	leftWall_[0] = leftLine_[0];
	leftWall_[1] = leftLine_[1];
	rightWall_[0] = rightLine_[0];
	rightWall_[1] = rightLine_[1];
	ROS_INFO("The leftside wall : %f, %f", leftWall_[0], leftWall_[1]);
	ROS_INFO("The rightside wall : %f, %f", rightWall_[0], rightWall_[1]);
	cleanAll();
}	

void Tree::wallDetect()
{
	int merge_size = merge_r_.size();
	int idxLeft = merge_size;
	int idxRight = merge_size;
	int idxFront = merge_size;
	//maxY_ = -1E18;
	//minY_ = 1E18;
	//maxX_ = -1E18;
	float maxY = -1E18;
	float minY = 1E18;
	float maxX = -1E18;
	for(int i = 0; i < merge_size; i++) {
		float y = merge_r_[i]*sin(merge_theta_[i]);
		float x = merge_r_[i]*cos(merge_theta_[i]);
		if(y > maxY && fabs(merge_theta_[i]-PI/2) < PI/6) {
			maxY = y;
			idxLeft = i;
		}
		if(y < minY && fabs(merge_theta_[i]-PI/2) < PI/6) {
			minY = y;
			idxRight = i;
		}	
		if(x > maxX) {
			maxX = x;
			idxFront = i;
		}
	}
	if(idxRight == merge_size) minY = -1E18;
	if(idxLeft == merge_size) maxY = 1E18;
	if(idxRight != merge_size) {
		rightLine_.push_back(merge_r_[idxRight]);
		rightLine_.push_back(merge_theta_[idxRight]);
	}
	if(idxLeft != merge_size) {
		leftLine_.push_back(merge_r_[idxLeft]);
		leftLine_.push_back(merge_theta_[idxLeft]);
	}
}

void Tree::cleanAll()
{
	points_.clear();
  segments_.clear();
  merge_segments_.clear();
  car_.clear();
  r_.clear();
  wall_.clear();
  theta_.clear();
  merge_r_.clear();
  merge_theta_.clear();
  leftLine_.clear();
  rightLine_.clear();
}

std::tuple<float, float, float, float, float, float> Tree::extractFeatures()
{
  float delta_x = currPose_[0]-initPose_[0];
  float delta_y = currPose_[1]-initPose_[1];
  float delta_th = currPose_[2]-initPose_[2];
 	float th_mid = delta_th-((leftLine_[1]+rightLine_[1])/2-PI/2);
 	//std::cout << "theta to mid line : " << th_mid << std::endl;
 	
 	float pointx = 0;
 	float pointy = -(delta_x*sin(initPose_[2])-delta_y*cos(initPose_[2]));
 	float dist2Mid = pointx*cos((leftLine_[1]+rightLine_[1])/2)+pointy*sin((leftLine_[1]+rightLine_[1])/2)-(leftLine_[0]+rightLine_[0])/2;
 	//std::cout << "distance to mid line : " << dist2Mid << std::endl;
	
	int minIdx = -1;
	if(!car_.empty()) {
		int car_size = car_.size();
		int minDist = 1E6;
		for(int i = 0; i < car_size; i++) {
			int beginIdx = car_[i][0];
			int endIdx = car_[i][1];
			for(int j = beginIdx; j <= endIdx; j++) {
				int x_point = points_[j][0]*50+250;
				int y_point = -points_[j][1]*50+250;
				if(fabs(points_[j][0]*cos(leftLine_[1])+points_[j][1]*sin(leftLine_[1])-leftLine_[0]) < 0.2) break;
				if(fabs(points_[j][0]*cos(rightLine_[1])+points_[j][1]*sin(rightLine_[1])-rightLine_[0]) < 0.2) break;
				if(pow(x_point, 2)+pow(y_point, 2) < minDist) {
					minIdx = j;
					minDist = pow(x_point, 2)+pow(y_point, 2);
				}
			}	
		}	
	}
	
	int mycarNum;
  int othercarNum = 0;
  if(!car_.empty()) {
		float dist = fabs(points_[minIdx][0]*sin((leftLine_[1]+rightLine_[1])/2)+points_[minIdx][1]*cos((leftLine_[1]+rightLine_[1])/2));
		float carline_dist = points_[minIdx][0]*cos((leftLine_[1]+rightLine_[1])/2)+points_[minIdx][1]*sin((leftLine_[1]+rightLine_[1])/2);
		//std::cout << "dist to the other car : " << dist << std::endl; 
		for(int i = 1; i < numLane_-1; i++) {
			float r1 = leftLine_[0]*(numLane_-i)/numLane_+rightLine_[0]*i/numLane_;
			float r2 = leftLine_[0]*(numLane_-i-1)/numLane_+rightLine_[0]*(i+1)/numLane_; 
		  if(carline_dist < r1 && carline_dist > r2) {
				othercarNum = i;
				//std::cout << "other car line_num : " << othercarNum << std::endl;
			}
		}	
	}
		
  float line_dist = pointx*cos((leftLine_[1]+rightLine_[1])/2)+pointy*sin((leftLine_[1]+rightLine_[1])/2);
  //float carline_dist = points_[minIdx][0]*cos((leftLine_[1]+rightLine_[1])/2)+points_[minIdx][1]*sin((leftLine_[1]+rightLine_[1])/2);
	//std::cout << "dist to the other car : " << dist << std::endl; 
 	for(int i = 1; i < numLane_-1; i++) {
		float r1 = leftLine_[0]*(numLane_-i)/numLane_+rightLine_[0]*i/numLane_;
		float r2 = leftLine_[0]*(numLane_-i-1)/numLane_+rightLine_[0]*(i+1)/numLane_; 
		if(line_dist < r1 && line_dist > r2) {
			mycarNum = i;
			//std::cout << "my car line_num : " << mycarNum << std::endl;
		} 
		/*if(carline_dist < r1 && carline_dist > r2) {
			othercarNum = i;
			std::cout << "other car line_num : " << othercarNum << std::endl;
		}*/
	}	
	float lf_dist;
	float cf_dist;
	float rf_dist;
	float headdeg = ((leftLine_[1]+rightLine_[1])/2-PI/2)*180/PI;
	float pos_th = delta_th*180/PI;
	float lane_devdist = dist2Mid/((leftLine_[0]-rightLine_[0])/2);
	if(lane_devdist > 1) lane_devdist = 1;
	if(lane_devdist < -1) lane_devdist = -1;
	if(othercarNum == 0) {
		lf_dist = 0;
		cf_dist = 0;
		rf_dist = 0;
	}
	else {
		if(othercarNum == mycarNum) {
			float dist = fabs(points_[minIdx][0]*sin((leftLine_[1]+rightLine_[1])/2)+points_[minIdx][1]*cos((leftLine_[1]+rightLine_[1])/2));
			lf_dist = 0;
			cf_dist = (dist+L_/2*cos(th_mid))*1000;
			rf_dist = 0;
		}
		else if(othercarNum < mycarNum) {
			float dist = fabs(points_[minIdx][0]*sin((leftLine_[1]+rightLine_[1])/2)+points_[minIdx][1]*cos((leftLine_[1]+rightLine_[1])/2));
			lf_dist = (dist+L_/2*cos(th_mid))*1000;
			cf_dist = 0;
			rf_dist = 0;
		}
		else {
			float dist = fabs(points_[minIdx][0]*sin((leftLine_[1]+rightLine_[1])/2)+points_[minIdx][1]*cos((leftLine_[1]+rightLine_[1])/2));
			lf_dist = 0;
			cf_dist = 0;
			rf_dist = (dist+L_/2*cos(th_mid))*1000;
		}
	}
	std::tuple<float, float, float, float, float, float> output(lf_dist, cf_dist, rf_dist, lane_devdist, headdeg, pos_th);
	//ROS_INFO("%f %f %f %f %f %f\n", lf_dist, cf_dist, rf_dist, lane_devdist, headdeg, pos_th);
	return output;
}
