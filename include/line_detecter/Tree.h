#include <cmath>
#include <vector>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>

class Tree
{
private:
  std::vector<std::vector<float>> points_;
  std::vector<std::vector<int>> segments_;
  std::vector<std::vector<int>> merge_segments_;
  std::vector<std::vector<int>> car_;
  std::vector<float> r_;
  std::vector<std::vector<float>> wall_;
  std::vector<float> theta_;
  std::vector<float> merge_r_;
  std::vector<float> merge_theta_;
  std::vector<float> leftLine_;
  std::vector<float> rightLine_;
  std::string scanName_;
  float leftWall_[2];
  float rightWall_[2];
  float initPose_[3];
  float currPose_[3];
  float thDist_;
  int start_, end_;
	float L_;
	int numLane_;
	int leftLane_;
	int rightLane_;  
  unsigned int* _accu;
	int _accu_w;
	int _accu_h;
	void fitting();
	void carDetect();
	void wallDetect();
	void split(int start, int end);
	void merge();
	void requestWall();
	void getWall(const sensor_msgs::LaserScan& laser_scan, float poseX, float poseY, float poseYaw);
	//std::tuple<float, float, float, float, float, float> extractFeatures();
	
public:
	Tree(std::string scanName, int numlane, int leftlane, int rightlane, float L);
	Tree();
	~Tree();
	void newScan(std::vector<std::vector<float>> points, float thDist, float poseX, float poseY, float poseTh);
	std::tuple<float, float, float, float, float, float> extractFeatures();
	void generateTree();
	void visualizeFitLine();
	void cleanAll();
};
