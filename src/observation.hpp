#ifndef OBSERVATION_H_
#define OBSERVATION_H_

#include <Eigen/Core>  //Vector2d

struct Observation {
	size_t feature_idx; //The feature index corresponding to this observation
	Eigen::Vector2d uvd; //the actual sensed U and V positions. It corresponds to image positions before being undistorted.
};

#endif
