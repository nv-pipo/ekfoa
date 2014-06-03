
#ifndef MOTION_TRACKER_H_
#define MOTION_TRACKER_H_

#include <string>

#include <opencv2/opencv.hpp>

#include <Eigen/Dense> //Matrix

#include <vector>

#include "observation.hpp"

//MotionTracker interface:
class MotionTracker {
public:
	virtual std::string type() = 0;

	virtual void process(cv::Mat & input_2, Eigen::MatrixXd & features_new_uvds, std::vector<Observation> & features_tracked, std::vector<int> & features_remove_list) = 0;

	virtual ~MotionTracker(){}
};

#endif
