#include "camera.hpp"

#include <stdio.h>
#include <math.h>

void Camera::undistort_fm( const Eigen::Vector2d & uvd, Eigen::Vector2d & uvu) const{
	//Center the coordinates:
	double xd = ( uvd(0) - Cx_ )*d_;
	double yd = ( uvd(1) - Cy_ )*d_;

	//Distance from the center to the point:
	double rd = sqrt(xd*xd + yd*yd);

	//Distortion factor:
	double D = 1 + k1_*rd*rd + k2_*rd*rd*rd*rd;

	//Undistort:
	double xu = xd*D;
	double yu = yd*D;

	//return the coordinates moved back the image coordinate system
	uvu(0) = xu/d_ + Cx_; //d = dx
	uvu(1) = yu/d_ + Cy_; //d = dy
}

void Camera::distort_fm( const Eigen::Vector2d & uv, Eigen::Vector2d & uvd ) const{
	double xu=(uv(0)-Cx_)*d_; //d = dx
	double yu=(uv(1)-Cy_)*d_; //d = dy

	double ru=sqrt(xu*xu+yu*yu);
	double rd=ru/(1+k1_*ru*ru+k2_*ru*ru*ru*ru);

	for (int i=0 ; i<10 ; i++) {
	    double f = rd + k1_*rd*rd*rd + k2_*rd*rd*rd*rd*rd - ru;
	    double f_p = 1 + 3*k1_*rd*rd + 5*k2_*rd*rd*rd*rd;
	    rd = rd - f/f_p;
	}

	double D = 1 + k1_*rd*rd + k2_*rd*rd*rd*rd;
	double xd = xu/D;
	double yd = yu/D;

	uvd(0) = xd/d_ + Cx_; //d = dx
	uvd(1) = yd/d_ + Cy_; //d = dy
}

/*
 * Takes an image pixel (uvd) and transforms it's homogeneous coordinates (undistorted and centered)
 */
void Camera::undistort_center( const Eigen::Vector2d & uv_d, Eigen::Vector3d & xy_u ) const{
	Eigen::Vector2d uv_u;
	undistort_fm(uv_d, uv_u);

	xy_u(0)=-(Cx_-uv_u(0))/K_(0,0); //K_(0,0) = fku,
	xy_u(1)=-(Cy_-uv_u(1))/K_(1,1); //K_(1,1) = fkv,
	xy_u(2)=1;
}
