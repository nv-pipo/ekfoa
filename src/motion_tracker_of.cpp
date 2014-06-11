#include "motion_tracker_of.hpp"

std::string MotionTrackerOF::type(){
	return std::string("OF");
}

void MotionTrackerOF::process(cv::Mat & input_2, Eigen::MatrixXd & features_added, std::vector<cv::Point2f> & features_tracked, std::vector<int> & features_removed){
	// '1' refers to previous frame
	// '2' refers to last received (input parameter) frame

	if ( ! input_1_gray_.data ){
		//At first frame, initialize the points_currently_tracked_mask_ Mat:
		points_correctly_tracked_mask_ = cv::Mat(input_2.size(), CV_8UC1);
	}

	features_tracked.reserve(min_number_of_features_in_image_); //Don't want a lot of reallocations later, so reserve to the maximum
	points_correctly_tracked_mask_.setTo(cv::Scalar(255));

//	double time = 0;

	cv::Mat input_2_gray;
	//Create a copy of the input in grayscale:
	cv::cvtColor(input_2, input_2_gray, CV_RGB2GRAY);

	//If there are any features to track:
	if (points_tracked_1.size() > 0) {
//		time = (double)cv::getTickCount();

		//TODO: Use 'OPTFLOW_USE_INITIAL_FLOW' with the kalman filter prediction. So that the estimations are considered the initial estimate
		// Find position of feature in new image
		cv::calcOpticalFlowPyrLK(
				input_1_gray_, input_2_gray, // 2 consecutive images
				points_tracked_1,            // input: interesting features points
				points_tracked_2,            // output: the respective positions (in second frame) of the input points
				status_of_,                  // output status vector (of unsigned chars)
				err                          // output vector of errors
		);
		//Use the same images in reverse order to verify that the points we got in the previous OpticFlow were correct
		cv::calcOpticalFlowPyrLK(
				input_2_gray, input_1_gray_,  // 2 consecutive images reversed
				points_tracked_2,             // input: interesting features points
				points_tracked_1_reverse,     // output: the respective positions (in second frame) of the input points
				status_of_reverse_,           // tracking success
				err                           // tracking error
		);
//		time = (double)cv::getTickCount() - time;
//		std::cout << "time OF = " << time/((double)cvGetTickFrequency()*1000.) << "ms" << std::endl;

		std::vector<cv::Point2f> p1;
		std::vector<cv::Point2f> p2;
		std::vector<uchar> status_ransac;
		std::vector<size_t> removed_of;
		for(size_t idx=0; idx < points_tracked_1.size() ; idx++) {
			if (accept_tracked_point(idx)){
				p1.push_back(points_tracked_1[idx]);
				p2.push_back(points_tracked_2[idx]);
			} else {
				//could not track it, so mark for removal it:
				removed_of.push_back(idx);
			}
		}

		cv::findFundamentalMat(p1, p2, cv::FM_RANSAC, 0.5, 0.99, status_ransac);

		cv::Scalar color;

		size_t idx_removed_of = 0;
		for(size_t idx=0; idx < p1.size() ; idx++) {
			color = cv::Scalar(0, 0, 255, 255);//red

			if (status_ransac[idx]){
				//Could track it!, so add current position as sensed input:
				features_tracked.push_back(p2[idx]);

				//make sure new features are not above or too close to this feature:
				cv::circle(points_correctly_tracked_mask_, p2[idx], distance_between_points_, cv::Scalar(0), CV_FILLED);

				//color it in  the frame as green:
				color = cv::Scalar(0, 255, 0, 255);//green
			} else {
				//idx+idx_removed_of : the index do delete is the current index plus the number of features removed by OF with lower index

				//check if OF removed features with lower index:
				while (idx_removed_of < removed_of.size() && removed_of[idx_removed_of] < idx+idx_removed_of){
					features_removed.push_back(removed_of[idx_removed_of]);
					idx_removed_of++;
				}
				//could not track it, so mark for removal it:
				features_removed.push_back(idx+idx_removed_of);
			}


			//Draw circle at current position:
			cv::circle(input_2, p2[idx], 3, color, 1);

			//Draw line between start position and end position:
			cv::line(input_2,
					p1[idx],   // initial position
					p2[idx],   // new position
					color);

			//Write the feature index next to it:
			std::stringstream text;
			text << idx;
			cv::Point2f text_start(p2[idx].x+5, p2[idx].y+5);
			cv::putText(input_2, text.str(), text_start, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255));
		}

		//Add any remaining features removed by Optical Flow:
		while (idx_removed_of < removed_of.size()){
			features_removed.push_back(removed_of[idx_removed_of]);
			idx_removed_of++;
		}
		std::cout << "removed_of.size(): " << removed_of.size() << std::endl;

		//Finally, remember the correctly tracked points (for next call):
		points_tracked_1 = features_tracked;
	}

	int num_new_features = min_number_of_features_in_image_ - features_tracked.size();

	if (num_new_features > 0){
		//TODO: Try to use some FAST heuristics to prefer unoccupied areas for new features...like a grid and one feature per square.
//		time = (double)cv::getTickCount();
		std::vector<cv::Point2f> features_added_cv;
		cv::goodFeaturesToTrack(
				input_2_gray,      // InputArray image
				features_added_cv, // OutputArray corners
				num_new_features,  // int maxCorners - Number of points to detect
				0.1,              // double qualityLevel=0.01 (larger is better quality)
				distance_between_points_, // double minDistance=1
				points_correctly_tracked_mask_,    // InputArray mask=noArray(). Where it should not look for new features
				3,              // int blockSize=3
				true,             // bool useHarrisDetector=false
				0.04              // double k=0.04
		);
		//Add new points to the currently tracked features at the beginning:
		points_tracked_1.insert(points_tracked_1.end(), features_added_cv.begin(), features_added_cv.end());

		//Copy to the "returned" list of new features:
		features_added.resize(2, features_added_cv.size());
		for (size_t i=0 ; i<features_added_cv.size() ; i++){
			features_added.col(i) << features_added_cv[i].x, features_added_cv[i].y;
		}

//		time = (double)cv::getTickCount() - time;
//		std::cout << "goodFeaturesToTrack = " << time/((double)cvGetTickFrequency()*1000.) << "ms" << std::endl;
	}

	//Remember this frame for next call:
	input_1_gray_ = input_2_gray.clone();
}

// determine which tracked point should be accepted. Rejected by: reverse OF match, OF status (OF and reverseOF) or RANSAC)
bool MotionTrackerOF::accept_tracked_point(int i){
	// cv::norm(points_tracked_1_reverse[i]-points_tracked_1[i])) < 1 is the distance between the original feature and the estimated original feature (going from frame THIS to PREVIOUS)
	// statusLk is whether framePrev->frameCurr optical flow says it got a good tracking
	// statusLkReverse is whether frameCurr->framePrev optical flow says it got a good tracking
	// errLk is whether framePrev->frameCurr optical flow says it got a good tracking
	// errLkReverse is whether frameCurr->framePrev optical flow says it got a good tracking
	return cv::norm(points_tracked_1_reverse[i]-points_tracked_1[i]) < 1 && status_of_[i] && status_of_reverse_[i];
}
