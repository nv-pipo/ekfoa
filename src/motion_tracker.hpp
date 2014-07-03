
#ifndef MOTION_TRACKER_H_
#define MOTION_TRACKER_H_

#include <string> //string
#include <iostream> //std::out

#include <opencv2/core/core.hpp> //circle, line, Point2f, putText
#include <opencv2/calib3d/calib3d.hpp> //findFundamentalMat
#include <opencv2/imgproc/imgproc.hpp> //cvtColor

#include <Eigen/Dense> //Matrix

#include <vector>

#include "kalman.hpp" //Features_extra

//MotionTracker interface:
class MotionTracker {
public:
	virtual std::string type() = 0;

	virtual void process(cv::Mat & input_2, std::vector<Features_extra> & features_extra, std::vector<cv::Point2f> & features_added) = 0;

	virtual ~MotionTracker(){}
};

#endif
