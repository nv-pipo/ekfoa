#ifndef CAMERA_H_
#define CAMERA_H_

#include "print.hpp" //print_txt

#include <Eigen/Dense> //Matrix3d
#include <iostream> //cout

/*
 * Notation:
 * uvu = undistorted image coordinate
 * uvd = distorted image coordinate
 * xyu = undistorted coordinates with origin in the Z axis
 * xyd = distorted coordinates with origin in the Z axis
 * p = point in cartesian coordinates with origin on the current camera position
 */

class Camera {
private:
	Eigen::Matrix3d K_;
	double fx_;
	double fy_;
	double cx_;
	double cy_;
	double k1_;
	double k2_;
	Eigen::Matrix<double, 3, 2> dgc_dhu_;

public:
	Camera(double fx, double fy, double cx, double cy, double k1, double k2) :
		fx_(fx),
		fy_(fy),
		cx_(cx),
		cy_(cy),
		k1_(k1),
		k2_(k2){
		K_ << fx,  0, cx,
		       0, fy, cy,
		 	   0,  0,  1;

		dgc_dhu_ << 1/fx,    0,
				       0, 1/fy,
				       0,    0;
	}


	/*
	 * undistort:
	 * Remove the distortion of an image coordinate: uvd -> uvu
	 */
	void undistort( const Eigen::Vector2d & uvd, Eigen::Vector2d & uvu) const;

	/*
	 * Jacobian of undistort function wrt. distorted image coords. jacobian(uvu, uvd): UVU_uvd
	 */
	void jacob_undistort(const Eigen::Vector2d & uvd, Eigen::Matrix2d & UVU_uvd) const;

	/*
	 * distort:
	 * Apply the radial distortion to an image coordinate: uvu -> uvd
	 */
	void distort( const Eigen::Vector2d & uvu, Eigen::Vector2d & uvd ) const;

	/*
	 * Takes a distorted image pixel (uvd) and transforms it to homogeneous coordinates (undistorted and centered aroundZ axis)
	 */
	void uvd_to_homogeneous( const Eigen::Vector2d & uvd, Eigen::Vector3d & homogeneous_projection )  const;

	/*
	 * Takes an undistorted image pixel (uvu) and transforms it to homogeneous coordinates (undistorted and centered aroundZ axis)
	 */
	void uvu_to_homogeneous( const Eigen::Vector2d & uvu, Eigen::Vector3d & homogeneous_projection )  const;

	/*
	 * % Jacobian of undistorted image point to homogeneous function wrt. uvu. jacobian(uvu_to_homogeneous, uvu) = UVUTOHOMOGENEOUS_uvu
	 */
	void jacob_uvu_to_homogeneous(Eigen::Matrix<double, 3, 2> & UVUTOHOMOGENEOUS_uvu) const;

	/*
	 * Point p projection to undistorted image coordinates?: p to uvu
	 */
	void project_p_to_uvu( const Eigen::Vector3d & p, Eigen::Vector2d & uvu ) const;

	/*
	 * Jacobian of point projection function wrt. to point p: jacobian(UVU_p)
	 */
	void jacob_project_p_to_uvu(const Eigen::Vector3d & p, Eigen::Matrix<double, 2, 3> & UVU_p) const;

};

#endif
