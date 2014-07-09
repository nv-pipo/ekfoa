#include "camera.hpp"

#include <stdio.h>
#include <math.h>

void Camera::undistort(const Eigen::Vector2d & uvd, Eigen::Vector2d & uvu) const{
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

void Camera::jacob_undistort(const Eigen::Vector2d & uvd, Eigen::Matrix2d & UVU_uvd) const{
	double ud=uvd(0);
	double vd=uvd(1);
	double xd=(uvd(0)-Cx_)*d_; //d_ = dx
	double yd=(uvd(1)-Cy_)*d_; //d_ = dy

	double rd2=xd*xd+yd*yd;
	double rd4=rd2*rd2;

	double uu_ud=(1+k1_*rd2+k2_*rd4)+(ud-Cx_)*(k1_+2*k2_*rd2)*(2*(ud-Cx_)*d_*d_);
	double vu_vd=(1+k1_*rd2+k2_*rd4)+(vd-Cy_)*(k1_+2*k2_*rd2)*(2*(vd-Cy_)*d_*d_);

	double uu_vd=(ud-Cx_)*(k1_+2*k2_*rd2)*(2*(vd-Cy_)*d_*d_);
	double vu_ud=(vd-Cy_)*(k1_+2*k2_*rd2)*(2*(ud-Cx_)*d_*d_);

	UVU_uvd << uu_ud,uu_vd,
			   vu_ud,vu_vd;
}

void Camera::distort(const Eigen::Vector2d & uvu, Eigen::Vector2d & uvd) const{
	double xu=(uvu(0)-Cx_)*d_; //d = dx
	double yu=(uvu(1)-Cy_)*d_; //d = dy

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

void Camera::uvd_to_homogeneous(const Eigen::Vector2d & uvd, Eigen::Vector3d & homogeneous) const{
	Eigen::Vector2d uvu;
	undistort(uvd, uvu);

	uvu_to_homogeneous(uvu, homogeneous);
}

void Camera::uvu_to_homogeneous(const Eigen::Vector2d & uvu, Eigen::Vector3d & homogeneous) const{
	homogeneous(0)=-(Cx_-uvu(0))/K_(0,0); //K_(0,0) = fku,
	homogeneous(1)=-(Cy_-uvu(1))/K_(1,1); //K_(1,1) = fkv,
	homogeneous(2)=1;
}

void Camera::jacob_uvu_to_homogeneous(Eigen::Matrix<double, 3, 2> & UVUTOHOMOGENEOUS_uvu) const{
	UVUTOHOMOGENEOUS_uvu = dgc_dhu_;
}

void Camera::project_p_to_uvu(const Eigen::Vector3d & p, Eigen::Vector2d & uvu) const{
//	double ku = 1/dx; // dx=d_
//	double kv = 1/dy; // dy=d_
	double k = 1/d_; // dx = dy = d => ku = 1/d = kv = 1/d
	uvu(0) = Cx_ + (p(0)/p(2))*f_*k;
	uvu(1) = Cy_ + (p(1)/p(2))*f_*k;
}

void Camera::jacob_project_p_to_uvu(const Eigen::Vector3d & p, Eigen::Matrix<double, 2, 3> & UVU_p) const{
	double k = 1/d_; //ku = 1/camera.dx = kv = 1/dy; d = dx = dy;
    double x = p(0);
    double y = p(1);
    double z = p(2);
    UVU_p << f_*k/(z),          0, -x*f_*k/(z*z),
                      0, f_*k/(z), -y*f_*k/(z*z);
}
