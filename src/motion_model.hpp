#ifndef MOTION_MODEL_H_
#define MOTION_MODEL_H_

#include <Eigen/Core>  //Derived
#include <Eigen/Eigen> //math/quaternion
#include <math.h> //math

#include "print.hpp" //print_txt

class MotionModel {
private:
	static void dR_by_dqw(const double w, const double x, const double y, const double z, Eigen::Matrix3d & m){
		m << 2*w, -2*z,  2*y,
	 		 2*z,  2*w, -2*x,
			-2*y,  2*x,  2*w;
	}


	static void dR_by_dqx(const double w, const double x, const double y, const double z, Eigen::Matrix3d & m){
		m << 2*x,  2*y,   2*z,
			 2*y, -2*x,  -2*w,
			 2*z,  2*w,  -2*x;
	}


	static void dR_by_dqy(const double w, const double x, const double y, const double z, Eigen::Matrix3d & m){
		m << -2*y, 2*x,  2*w,
			  2*x, 2*y,  2*z,
			 -2*w, 2*z, -2*y;
	}

	static void dR_by_dqz(const double w, const double x, const double y, const double z, Eigen::Matrix3d & m){
		m << -2*z, -2*w, 2*x,
			  2*w, -2*z, 2*y,
			  2*x,  2*y, 2*z;
	}

	static void qprod(const Eigen::Vector4d & q, const Eigen::Vector4d & p, Eigen::Vector4d & prod_result){
		double a=q(0);
		Eigen::Vector3d v = q.segment(1, 3); //coefficients q
		double x=p(0);
		Eigen::Vector3d u = p.segment(1, 3); //coefficients p

		prod_result << a*x-v.transpose()*u, (a*u+x*v) + v.cross(u);
	}
public:
	static void update_xv_and_compute_F(const double delta_t, Eigen::VectorXd & x_k_k, Eigen::MatrixXd & F);

	static void prediction_step(const double delta_t, const double std_a, const double std_alpha, Eigen::VectorXd & x_k_k, Eigen::MatrixXd & F, Eigen::MatrixXd & Q);

	static void compute_F(const double delta_t, const Eigen::Vector4d & qWR_old, const Eigen::Vector3d & wW_old, const Eigen::Vector4d & qWT, Eigen::MatrixXd & F);

	static void compute_Q(const double delta_t, const Eigen::Vector4d & qWR_old, const Eigen::VectorXd & wW_old, const double std_a, const double std_alpha, Eigen::MatrixXd & Q);

	static void quaternion_matrix(const Eigen::Vector4d & q, Eigen::Matrix3d & q_rotation_matrix);
	static void quaternion_from_angular_velocity(const Eigen::Vector3d & av, Eigen::Vector4d & q);

	static void dq3_by_dq2(const Eigen::Vector4d & q2, Eigen::Matrix4d & m);
	static void dq3_by_dq1(const Eigen::Vector4d & q1, Eigen::Matrix4d & m);

	static void dqomegadt_by_domega(const Eigen::Vector3d &omega,
			const double delta_t,
			Eigen::MatrixXd &dqomegadt_by_domega);

	static double dq0_by_domegaA(const double omegaA, const double omega,
			const double delta_t);

	static double dqA_by_domegaA(const double omegaA, const double omega,
			const double delta_t);

	static double dqA_by_domegaB(const double omegaA, const double omegaB,
			const double omega, double delta_t);

	static void dposw_dq(const Eigen::Vector3d & xyz, const Eigen::Vector4d & q, Eigen::Matrix<double, 3, 4> & dposw_dq);
	static void qconj(const Eigen::Vector4d & q, Eigen::Vector4d & qconj){
		qconj(0) =  q(0); //w
		qconj(1) = -q(1); //x
		qconj(2) = -q(2); //y
		qconj(3) = -q(3); //z
	}

};
#endif
