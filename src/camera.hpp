#ifndef CAMERA_H_
#define CAMERA_H_

#include "print.hpp" //print_txt

#include <Eigen/Dense> //Matrix3d
#include <iostream> //cout

class Camera {
private:
	Eigen::Matrix3d K_;
	double d_;
	double Cx_;
	double Cy_;
	double k1_;
	double k2_;
	double f_;
	Eigen::Matrix<double, 3, 2> dgc_dhu_;

public:
	Camera(double d, double Cx, double Cy, double k1, double k2, double f) :
		d_(d),
		Cx_(Cx),
		Cy_(Cy),
		k1_(k1),
		k2_(k2),
		f_(f){
		K_ << f/d,   0, Cx,
		        0, f/d, Cy,
		 	    0,   0,  1;

		dgc_dhu_ << +1/K_(0,0), 0, //fku =  K_(1,1);
				    0, +1/K_(1,1), //fkv =  K_(2,2);
				    0, 0;
	}


	void undistort_fm( const Eigen::Vector2d & uvd, Eigen::Vector2d & uvu) const;
	void distort_fm( const Eigen::Vector2d & uv, Eigen::Vector2d & uvd ) const;
	void undistort_center( const Eigen::Vector2d & uvd, Eigen::Vector3d & homogeneous_projection )  const;
	void jacob_undistor_fm( const Eigen::Vector2d & uvd, Eigen::Matrix2d & m) const{
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

		m << uu_ud,uu_vd,
			 vu_ud,vu_vd;
	}
	void get_dgc_dhu(Eigen::Matrix<double, 3, 2> & dgc_dhu) const{
		dgc_dhu = dgc_dhu_;
	}

	void get_dhu_dhrl(const Eigen::Vector3d & hC, Eigen::Matrix<double, 2, 3> & dhu_dhrl) const{
		double k = 1/d_; //ku = 1/camera.dx = kv = 1/dy; d = dx = dy;
	    double hCx = hC(0);
	    double hCy = hC(1);
	    double hCz = hC(2);
	    dhu_dhrl << +f_*k/(hCz),            0, -hCx*f_*k/(hCz*hCz),
	                          0,  +f_*k/(hCz), -hCy*f_*k/(hCz*hCz);
	}
	void hc_to_undistorted( const Eigen::Vector3d & homogeneous, Eigen::Vector2d & uvu ) const{
//		double ku = 1/dx; // dx=d_
//		double kv = 1/dy; // dy=d_
		double k = 1/d_; // dx = dy = d => ku = 1/d = kv = 1/d
	    uvu(0) = Cx_ + (homogeneous(0)/homogeneous(2))*f_*k;
	    uvu(1) = Cy_ + (homogeneous(1)/homogeneous(2))*f_*k;
	}
};

#endif
