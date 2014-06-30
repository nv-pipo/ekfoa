#ifndef MOTION_TRACKER_OF_H_
#define MOTION_TRACKER_OF_H_

#include "motion_tracker.hpp"

class MotionTrackerOF: public MotionTracker {
public:
	MotionTrackerOF(int min_number_of_features_in_image, int distance_between_points) :
			min_number_of_features_in_image_(min_number_of_features_in_image),
			distance_between_points_(distance_between_points) {}

	std::string type();

	void process(cv::Mat & input_2, std::vector<Features_extra> & features_extra, std::vector<cv::Point2f> & features_added);

	~MotionTrackerOF(){}
private:

	int min_number_of_features_in_image_;
	int distance_between_points_;

	cv::Rect image_dimensions_;

	// '1' refers to previous frame
	// '2' refers to last received (input parameter) frame

	cv::Mat input_1_gray_;
	cv::Mat points_correctly_tracked_mask_; //Used to filter out currently used locations of the image, so that new features do not overlap with existing ones.

	std::vector<cv::Point2f> points_tracked_1;
	std::vector<cv::Point2f> points_tracked_2;
	std::vector<cv::Point2f> points_tracked_1_reverse;

	std::vector<uchar> status_of_; // status of tracked features (Optical Flow)
	std::vector<uchar> status_of_reverse_; // status of tracked features (Optical Flow second)
	std::vector<float> err;    // error of tracked features (Optical Flow)

	bool accept_tracked_point(int i);
};

#endif
