
#ifndef MOTION_TRACKER_H_
#define MOTION_TRACKER_H_

#include <string>

#include <opencv2/opencv.hpp>

#include <Eigen/Dense> //Matrix

#include <vector>

//MotionTracker interface:
class MotionTracker {
public:
	virtual std::string type() = 0;

	virtual void process(cv::Mat & input_2, Eigen::MatrixXd & features_added, std::vector<cv::Point2f> & features_tracked, std::vector<int> & features_removed) = 0;

	virtual ~MotionTracker(){}
};

#endif
