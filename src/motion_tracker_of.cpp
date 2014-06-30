#include "motion_tracker_of.hpp"

std::string MotionTrackerOF::type(){
	return std::string("OF");
}

void MotionTrackerOF::process(cv::Mat & input_2, std::vector<Features_extra> & features_extra, std::vector<cv::Point2f> & features_added){
	// '1' refers to previous frame
	// '2' refers to last received (input parameter) frame

	assert(features_extra.size() == points_tracked_1.size());

	std::vector<cv::Point2f> features_tracked;

	if ( ! input_1_gray_.data ){
		//At first frame, initialize the points_currently_tracked_mask_ Mat:
		points_correctly_tracked_mask_ = cv::Mat(input_2.size(), CV_8UC1);

		//Set the dimensions of the input image. So that later we can easily check if the tracked point is inside it:
		image_dimensions_.x = 0;
		image_dimensions_.y = 0;
		image_dimensions_.height = input_2.rows;
		image_dimensions_.width = input_2.cols;
	}

	points_correctly_tracked_mask_.setTo(cv::Scalar(255));

//	double time = 0;

	cv::Mat input_2_gray;
	//Create a copy of the input in grayscale:
	cv::cvtColor(input_2, input_2_gray, CV_RGB2GRAY);

	//If there are any features to track:
	if (points_tracked_1.size() > 0) {
//		time = (double)cv::getTickCount();

		//Copy the predicted feature location to the OF points_tracked_2, it will try to find the feature appearance around the copied point.
		points_tracked_2.resize(features_extra.size());
		for (size_t i=0 ; i<features_extra.size() ; i++){
			points_tracked_2[i].x = features_extra[i].h(0);
			points_tracked_2[i].y = features_extra[i].h(1);
		}

		//TODO: Use 'OPTFLOW_USE_INITIAL_FLOW' with the kalman filter prediction. So that the estimations are considered the initial estimate
		// Find position of feature in new image
		cv::calcOpticalFlowPyrLK(
				input_1_gray_, input_2_gray, // 2 consecutive images
				points_tracked_1,            // input: interesting features points
				points_tracked_2,            // output: the respective positions (in second frame) of the input points
				status_of_,                  // output status vector (of unsigned chars)
				err,                          // output vector of errors
				cv::Size(21,21),
				3,
				cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01),
				cv::OPTFLOW_USE_INITIAL_FLOW
		);
		//Use the same images in reverse order to verify that the points we got in the previous OpticFlow were correct
		cv::calcOpticalFlowPyrLK(
				input_2_gray, input_1_gray_,  // 2 consecutive images reversed
				points_tracked_2,             // input: interesting features points
				points_tracked_1_reverse,     // output: the respective positions (in second frame) of the input points
				status_of_reverse_,           // tracking success
				err,                           // tracking error
				cv::Size(21,21),
				3,
				cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01),
				0
		);
//		time = (double)cv::getTickCount() - time;
//		std::cout << "time OF = " << time/((double)cvGetTickFrequency()*1000.) << "ms" << std::endl;

		std::vector<cv::Point2f> p1;
		std::vector<cv::Point2f> p2;
		std::vector<uchar> status_ransac;
		std::vector<size_t> removed_of;
		for(size_t i=0; i < points_tracked_1.size() ; i++) {
			//TODO: features_extra[i].is_valid can be checked before, for optimization, but then synchronization needs to be handled.
			if (features_extra[i].is_valid && accept_tracked_point(i) && points_tracked_2[i].inside(image_dimensions_)){
				p1.push_back(points_tracked_1[i]);
				p2.push_back(points_tracked_2[i]);
			} else {
				//could not track it, so mark for removal it:
				removed_of.push_back(i);
				features_extra[i].is_valid = false;
			}
		}

		cv::findFundamentalMat(p1, p2, cv::FM_RANSAC, 0.5, 0.99, status_ransac);

		cv::Scalar color;

		int counter_removed_of=0;//counter_removed_of is keeps track of the number of features smaller than the current feature
		for(size_t i=0; i < p1.size() ; i++) {
			color = cv::Scalar(0, 0, 255, 255);//red

			//Count if there were any features removed by OF, it will be an offset
			while(counter_removed_of<removed_of.size() && i+counter_removed_of >= removed_of[counter_removed_of]){
				counter_removed_of++;
			}

			if (status_ransac[i]){
				//Could track it!, so add current position as sensed input:
				features_extra[i+counter_removed_of].z(0) = p2[i].x;
				features_extra[i+counter_removed_of].z(1) = p2[i].y;
				features_extra[i+counter_removed_of].z_cv = p2[i];

				features_tracked.push_back(p2[i]);

				//make sure new features are not above or too close to this feature:
				cv::circle(points_correctly_tracked_mask_, p2[i], distance_between_points_, cv::Scalar(0), CV_FILLED);

				//color it in  the frame as green:
				color = cv::Scalar(0, 255, 0, 255);//green
				//Write the feature index next to it:
				std::stringstream text;
				text << features_tracked.size()-1;
				cv::Point2f text_start(p2[i].x+5, p2[i].y+5);
				cv::putText(input_2, text.str(), text_start, cv::FONT_HERSHEY_SIMPLEX, 0.5, color);

			} else {
				//Feature disappeared from image or was not correctly tracked, so mark it for deletion:
				features_extra[i+counter_removed_of].is_valid = false;
			}

			//Draw circle at current position:
			cv::circle(input_2, p2[i], 3, color, 1);

			//Draw line between start position and end position:
			cv::line(input_2,
					p1[i],   // initial position
					p2[i],   // new position
					color);
		}

		//Finally, remember the correctly tracked points (for next call):
		points_tracked_1 = features_tracked;
	}

	int num_new_features = min_number_of_features_in_image_ - features_tracked.size();

	if (num_new_features > 0){
		//TODO: Try to use some FAST heuristics to prefer unoccupied areas for new features...like a grid and one feature per square.
//		time = (double)cv::getTickCount();
		cv::goodFeaturesToTrack(
				input_2_gray,      // InputArray image
				features_added, // OutputArray corners
				num_new_features,  // int maxCorners - Number of points to detect
				0.01,              // double qualityLevel=0.01 (larger is better quality)
				distance_between_points_, // double minDistance=1
				points_correctly_tracked_mask_,    // InputArray mask=noArray(). Where it should not look for new features
				3,              // int blockSize=3
				true,             // bool useHarrisDetector=false
				0.04              // double k=0.04
		);
		//Add new points to the currently tracked features at the beginning:
		points_tracked_1.insert(points_tracked_1.end(), features_added.begin(), features_added.end());

		//Draw the newly added features in blue:
		for (size_t i=0 ; i<features_added.size() ; i++){
			cv::circle(input_2, features_added[i], 3, cv::Scalar(255,0,0), 1);
			std::stringstream text;
			text << features_tracked.size() + i;
			cv::Point2f text_start(features_added[i].x+5, features_added[i].y+5);
			cv::putText(input_2, text.str(), text_start, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0));
		}

//		time = (double)cv::getTickCount() - time;
//		std::cout << "goodFeaturesToTrack = " << time/((double)cvGetTickFrequency()*1000.) << "ms" << std::endl;
	}

	//Remember this frame for next call:
	input_1_gray_ = input_2_gray.clone();
}

// determine which tracked point should be accepted. Rejected by: reverse OF match, OF status (OF and reverseOF) or RANSAC)
bool MotionTrackerOF::accept_tracked_point(size_t i){
	// cv::norm(points_tracked_1_reverse[i]-points_tracked_1[i])) < 1 is the distance between the original feature and the estimated original feature (going from frame THIS to PREVIOUS)
	// statusLk is whether framePrev->frameCurr optical flow says it got a good tracking
	// statusLkReverse is whether frameCurr->framePrev optical flow says it got a good tracking
	// errLk is whether framePrev->frameCurr optical flow says it got a good tracking
	// errLkReverse is whether frameCurr->framePrev optical flow says it got a good tracking
	return cv::norm(points_tracked_1_reverse[i]-points_tracked_1[i]) < 1 && status_of_[i] && status_of_reverse_[i];
}
