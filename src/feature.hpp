#ifndef FEATURE_H_
#define FEATURE_H_

#include "camera.hpp"
#include "print.hpp"
#include "motion_model.hpp" //MotionModel

#include <Eigen/Core>  //Derived
#include <Eigen/Eigen> //math/quaternion

class Feature {
private:
	static void compute_m( const double theta, const double phi, Eigen::Vector3d & mi );
	static void compute_dhrl_dy(const Eigen::Vector3d & rW, const Eigen::Matrix3d & qWR_rotation_matrix_inverse, const Eigen::VectorXd & yi, Eigen::Matrix<double, 3, 6> & dhrl_dy);
	static Eigen::Vector3d compute_unshifted_3d_position( const Eigen::Vector3d & rW, const Eigen::VectorXd & yi);

public:
	static void compute_cartesian( const Eigen::VectorXd & yi, Eigen::Vector3d & XYZ );
	static bool compute_h( const Camera & cam, const Eigen::Vector3d & rW, const Eigen::Matrix3d & qWR_rotation_matrix, const Eigen::VectorXd & yi, Eigen::Vector2d & hi );
	static void compute_H( const Camera & cam, const Eigen::Vector3d & rW, const Eigen::Vector4d & qWR, const Eigen::Matrix3d & qWR_rotation_matrix, const Eigen::VectorXd & x_k_k, const Eigen::VectorXd & yi, const int yi_start_pos, const Eigen::Vector2d & hi, Eigen::MatrixXd & Hi);
};

#endif
