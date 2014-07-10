#include "camera.hpp"

#include <stdio.h>
#include <math.h>

void Camera::undistort(const Eigen::Vector2d & uvd, Eigen::Vector2d & uvu) const{
	//convert from image coordinates to the projection plane coordinates (inv(K_) * [uvd ; 1]):
	double xd = (uvd(0) - cx_)/fx_;
	double yd = (uvd(1) - cy_)/fy_;

	//calculate the squared distance to the optical center:
	double r2 = xd*xd + yd*yd;
	double r4 = r2*r2;

	//Distortion factor:
	double distortion_factor = 1 + k1_*r2 + k2_*r4;

	//Undistort (xyd -> xyu):
	double xu = xd*distortion_factor;
	double yu = yd*distortion_factor;

	//return from the linear projection plane coordinate system to the image coordinate system (K_ * [xu ; yu ; 1]):
	uvu << xu*fx_ + cx_,
           yu*fy_ + cy_;
}

void Camera::jacob_undistort(const Eigen::Vector2d & uvd, Eigen::Matrix2d & UVU_uvd) const{
	double ud=uvd(0);
	double vd=uvd(1);

	//convert from image coordinates to the projection plane coordinates (inv(K_) * [uvd ; 1]):
	double xd=(uvd(0) - cx_)/fx_;
	double yd=(uvd(1) - cy_)/fy_;

	//calculate the squared distance to the optical center:
	double r2=xd*xd + yd*yd;
	double r4=r2*r2;

	double fx2 = fx_*fx_;
	double fy2 = fy_*fy_;

	double uu_ud=k1_*r2 + k2_*r4 + ((k1_*(2*cx_ - 2*ud))/fx2 + (2*k2_*r2*(2*cx_ - 2*ud))/fx2)*(cx_ - ud) + 1;
	double vu_vd=k1_*r2 + k2_*r4 + ((k1_*(2*cy_ - 2*vd))/fy2 + (2*k2_*r2*(2*cy_ - 2*vd))/fy2)*(cy_ - vd) + 1;

	double uu_vd=((k1_*(2*cy_ - 2*vd))/fy2 + (2*k2_*r2*(2*cy_ - 2*vd))/fy2)*(cx_ - ud);
	double vu_ud=((k1_*(2*cx_ - 2*ud))/fx2 + (2*k2_*r2*(2*cx_ - 2*ud))/fx2)*(cy_ - vd);

	UVU_uvd << uu_ud,uu_vd,
               vu_ud,vu_vd;
}

void Camera::distort(const Eigen::Vector2d & uvu, Eigen::Vector2d & uvd) const{
	//convert from image coordinates to the projection plane coordinates (inv(K_) * [uvu ; 1]):
	double xu=(uvu(0) - cx_)/fx_;
	double yu=(uvu(1) - cy_)/fy_;

	//calculate the squared distance to the optical center:
	double r2 = xu*xu+yu*yu;
	double r4 = r2*r2;

	double r=sqrt(r2);
	double rd=r/(1+k1_*r2+k2_*r4);

	double f, f_p;
	for (int i=0 ; i<10 ; i++){
		f = rd + k1_*rd*rd*rd + k2_*rd*rd*rd*rd*rd - r;
		f_p = 1 + 3*k1_*rd*rd + 5*k2_*rd*rd*rd*rd;
		rd = rd - f/f_p;
	}

	double rd2 = rd*rd;

	double distortion_factor = 1 + k1_*rd2 + k2_*rd2*rd2;
	double xd = xu/distortion_factor;
	double yd = yu/distortion_factor;

	//return from the linear projection plane coordinate system to the image coordinate system (K_ * [xd ; yd ; 1]):
	uvd << xd*fx_ + cx_,
           yd*fy_ + cy_;
}

void Camera::uvd_to_homogeneous(const Eigen::Vector2d & uvd, Eigen::Vector3d & homogeneous) const{
	Eigen::Vector2d uvu;
	undistort(uvd, uvu);

	uvu_to_homogeneous(uvu, homogeneous);
}

void Camera::uvu_to_homogeneous(const Eigen::Vector2d & uvu, Eigen::Vector3d & homogeneous) const{
	////inv(K_) * [uvu ; 1] = homogeneous
	homogeneous(0)=(uvu(0) - cx_)/fx_;
	homogeneous(1)=(uvu(1) - cy_)/fy_;
	homogeneous(2)=1;
}

void Camera::jacob_uvu_to_homogeneous(Eigen::Matrix<double, 3, 2> & UVUTOHOMOGENEOUS_uvu) const{
	UVUTOHOMOGENEOUS_uvu = dgc_dhu_;
}

void Camera::project_p_to_uvu(const Eigen::Vector3d & p, Eigen::Vector2d & uvu) const{
	//K_ * p = uvu
	uvu(0) = cx_ + (p(0)/p(2))*fx_;
	uvu(1) = cy_ + (p(1)/p(2))*fx_;
}

void Camera::jacob_project_p_to_uvu(const Eigen::Vector3d & p, Eigen::Matrix<double, 2, 3> & UVU_p) const{
	double x = p(0);
	double y = p(1);
	double z = p(2);
	UVU_p << fx_/z,     0, -(x*fx_)/(z*z),
                 0, fy_/z, -(y*fy_)/(z*z);
}
