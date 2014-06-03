#include "motion_model.hpp"

void MotionModel::prediction_step(const double delta_t, const double std_a, const double std_alpha, Eigen::VectorXd & x_k_k, Eigen::MatrixXd & F, Eigen::MatrixXd & Q){

	/****************************************
	 * Camera motion prediction
	 ****************************************/
	Eigen::Vector3d rW_old = x_k_k.segment(0,3); //Extract (last step) position from state vector
	Eigen::Vector4d qWR_old = x_k_k.segment(3,4); //Extract (last step) orientation quaterion from state vector
	Eigen::Vector3d vW_old = x_k_k.segment(7, 3);  //Extract (last step) velocity from state vector
	Eigen::Vector3d wW_old = x_k_k.segment(10, 3); //Extract (last step) angular velocity from state vector

	Eigen::Vector4d qWT;
	MotionModel::quaternion_from_angular_velocity(wW_old * delta_t, qWT);

	Eigen::Vector4d qWR_new;
	qprod(qWR_old,  qWT, qWR_new);

	///////// compute F /////////
	MotionModel::compute_F(delta_t, qWR_old, wW_old, qWT, F);

	///////// compute Q /////////
	MotionModel::compute_Q(delta_t, qWR_old, wW_old, std_a, std_alpha, Q);

	//Update XV:
	//Motion model (constant speed), estimated position is the current position plus the velocity * delta_time:
	//New position:
	x_k_k.segment(0, 3) = rW_old + vW_old*delta_t;

	//New orientation
	x_k_k.segment(3, 4) = qWR_new;

	//Linear and angular velocities are updated by the kalman filter update step
}

void MotionModel::compute_F(const double delta_t, const Eigen::Vector4d & qWR_old, const Eigen::Vector3d & wW_old, const Eigen::Vector4d & qWT, Eigen::MatrixXd & F){
	// Now on to the Jacobian...
	// Identity is a good place to start since overall structure is like this
	// I       0             dxnew_by_dv   0
	// 0       dqnew_by_dq   0             dqnew_by_domega
	// 0       0             I             0
	// 0       0             0             I

	//Start with an identity matrix
	F.setIdentity(13, 13);

	// dxnew_by_dv = I * delta_t
	Eigen::Matrix3d dxnew_by_dv = Eigen::Matrix3d::Identity() * delta_t;

	// Fill in dqnew_by_dq
	Eigen::Matrix4d dqnew_by_dq;
	MotionModel::dq3_by_dq2(qWT, dqnew_by_dq);

	// Fill in dqnew_by_domega = d(q x qwt)_by_dqwt . dqwt_by_domega
	Eigen::Matrix4d dqXqwt_by_dqwt;
	MotionModel::dq3_by_dq1(qWR_old, dqXqwt_by_dqwt);

	// Use function below for dqwt_by_domega
	Eigen::MatrixXd dqwt_by_domega(4,3);
	MotionModel::dqomegadt_by_domega(wW_old, delta_t, dqwt_by_domega);

	// And plug it in
	F.block<3, 3>(0, 7) = dxnew_by_dv;
	F.block<4, 4>(3, 3) = dqnew_by_dq;
	F.block<4, 3>(3, 10) = dqXqwt_by_dqwt * dqwt_by_domega;
}

void MotionModel::compute_Q(const double delta_t, const Eigen::Vector4d & qWR_old, const Eigen::VectorXd & wW_old, const double std_a, const double std_alpha, Eigen::MatrixXd & Q){
	// Fill noise covariance matrix Pnn: this is the covariance of
	// the noise vector (V)
	//                  (Omega)
	// that gets added to the state.
	// Form of this could change later, but for now assume that
	// V and Omega are independent, and that each of their components is
	// independent...
	double linear_velocity_noise_variance = std_a * std_a *
			delta_t * delta_t;
	double angular_velocity_noise_variance = std_alpha * std_alpha *
			delta_t * delta_t;

	// Independence means that the matrix is diagonal
	Eigen::MatrixXd Pnn(6,6);
	Pnn.setZero();
	Pnn(0,0) = linear_velocity_noise_variance;
	Pnn(1,1) = linear_velocity_noise_variance;
	Pnn(2,2) = linear_velocity_noise_variance;
	Pnn(3,3) = angular_velocity_noise_variance;
	Pnn(4,4) = angular_velocity_noise_variance;
	Pnn(5,5) = angular_velocity_noise_variance;

	// Form Jacobian dxnew_by_dn
	// Is like this:
	// I * delta_t     0
	// 0               dqnew_by_dOmega
	// I               0
	// 0               I

	// Start by zeroing
	Eigen::MatrixXd dxnew_by_dn(13,6);
	dxnew_by_dn.setZero();

	Eigen::Matrix4d dqXqwt_by_dqwt;
	MotionModel::dq3_by_dq1(qWR_old, dqXqwt_by_dqwt);

	Eigen::MatrixXd dqwt_by_domega(4,3);
	MotionModel::dqomegadt_by_domega(wW_old, delta_t, dqwt_by_domega);

	dxnew_by_dn.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * delta_t;
	dxnew_by_dn.block<3, 3>(7, 0) = Eigen::Matrix3d::Identity();
	dxnew_by_dn.block<3, 3>(10, 3) = Eigen::Matrix3d::Identity();
	dxnew_by_dn.block<4, 3>(3, 3) = dqXqwt_by_dqwt*dqwt_by_domega;

	// Finally do Q = dxnew_by_dn . Pnn . dxnew_by_dnT
	Q = dxnew_by_dn * Pnn * dxnew_by_dn.transpose();
}

void MotionModel::quaternion_from_angular_velocity(const Eigen::Vector3d & av, Eigen::Vector4d & q) {
	double angle=av.norm();
	if (angle > 0) {
	    const double c = cos(angle/2.0);
	    const double s = sin(angle/2.0) / angle;

	    q(0) = c;
	    q(1) = s * av(0);
	    q(2) = s * av(1);
	    q(3) = s * av(2);
	} else {
	    q << 1, 0, 0, 0;
	}
}


/*
 * JACOBIAN related functions:
 */
void MotionModel::dq3_by_dq2(const Eigen::Vector4d & q2, Eigen::Matrix4d & m) {
	double  w = q2(0);
	double  x = q2(1);
	double  y = q2(2);
	double  z = q2(3);

	m << w, -x, -y, -z,
			x,  w,  z, -y,
			y, -z,  w,  x,
			z,  y, -x,  w;
}

void MotionModel::dq3_by_dq1(const Eigen::Vector4d & q1, Eigen::Matrix4d & m) {
	double  w = q1(0);
	double  x = q1(1);
	double  y = q1(2);
	double  z = q1(3);

	m << w, -x, -y, -z,
			x,  w, -z,  y,
			y,  z,  w, -x,
			z, -y,  x,  w;

}

void MotionModel::dqomegadt_by_domega(const Eigen::Vector3d &omega,
		const double delta_t,
		Eigen::MatrixXd & dqomegadt_by_domega) {
	// Modulus
	double omegamod = sqrt(omega(0) * omega(0) + omega(1) * omega(1) +
			omega(2) * omega(2));

	// Use generic ancillary functions to calculate components of Jacobian
	dqomegadt_by_domega(0, 0) = dq0_by_domegaA(omega(0), omegamod, delta_t);
	dqomegadt_by_domega(0, 1) = dq0_by_domegaA(omega(1), omegamod, delta_t);
	dqomegadt_by_domega(0, 2) = dq0_by_domegaA(omega(2), omegamod, delta_t);
	dqomegadt_by_domega(1, 0) = dqA_by_domegaA(omega(0), omegamod, delta_t);
	dqomegadt_by_domega(1, 1) = dqA_by_domegaB(omega(0), omega(1), omegamod, delta_t);
	dqomegadt_by_domega(1, 2) = dqA_by_domegaB(omega(0), omega(2), omegamod, delta_t);
	dqomegadt_by_domega(2, 0) = dqA_by_domegaB(omega(1), omega(0), omegamod, delta_t);
	dqomegadt_by_domega(2, 1) = dqA_by_domegaA(omega(1), omegamod, delta_t);
	dqomegadt_by_domega(2, 2) = dqA_by_domegaB(omega(1), omega(2), omegamod, delta_t);
	dqomegadt_by_domega(3, 0) = dqA_by_domegaB(omega(2), omega(0), omegamod, delta_t);
	dqomegadt_by_domega(3, 1) = dqA_by_domegaB(omega(2), omega(1), omegamod, delta_t);
	dqomegadt_by_domega(3, 2) = dqA_by_domegaA(omega(2), omegamod, delta_t);
}

//
// DQ0 BY DOMEGAA
// Ancillary function to calculate part of Jacobian \f$ \partial q / \partial
// \omega \f$ which is repeatable due to symmetry. Here omegaA is one of omegax,
// omegay, omegaz.
double MotionModel::dq0_by_domegaA(const double omegaA, const double omega,
		const double delta_t) {
	return (-delta_t / 2.0) * (omegaA / omega) * sin(omega * delta_t / 2.0);
}

//
// DQA BY DOMEGAA
// Ancillary function to calculate part of Jacobian \f$ \partial q / \partial
// \omega \f$ which is repeatable due to symmetry. Here omegaA is one of omegax,
// omegay, omegaz and similarly with qA.
double MotionModel::dqA_by_domegaA(const double omegaA, const double omega,
		const double delta_t) {
	return (delta_t / 2.0) * omegaA * omegaA / (omega * omega)
			* cos(omega * delta_t / 2.0)
			+ (1.0 / omega) * (1.0 - omegaA * omegaA / (omega * omega))
			* sin(omega * delta_t / 2.0);
}

//
// DQA BY DOMEGAB
// Ancillary function to calculate part of Jacobian \f$ \partial q / \partial
// \omega \f$ which is repeatable due to symmetry. Here omegaB is one of omegax,
// omegay, omegaz and similarly with qA.
double MotionModel::dqA_by_domegaB(const double omegaA, const double omegaB,
		const double omega, double delta_t) {
	return (omegaA * omegaB / (omega * omega)) *
			( (delta_t / 2.0) * cos(omega * delta_t / 2.0)
					- (1.0 / omega) * sin(omega * delta_t / 2.0) );
}

/*
 * Compute the derivative of a 3d position against a quaterion:
 */
void MotionModel::dposw_dq(const Eigen::Vector3d & xyz, const Eigen::Vector4d & q, Eigen::Matrix<double, 3, 4> & dposw_dq){
	dposw_dq.setZero();

	Eigen::Matrix3d t33;
	MotionModel::dR_by_dqw(q(0), q(1), q(2), q(3), t33);
	dposw_dq.block<3, 1>(0, 0) = t33 * xyz;
	MotionModel::dR_by_dqx(q(0), q(1), q(2), q(3), t33);
	dposw_dq.block<3, 1>(0, 1) = t33 * xyz;
	MotionModel::dR_by_dqy(q(0), q(1), q(2), q(3), t33);
	dposw_dq.block<3, 1>(0, 2) = t33 * xyz;
	MotionModel::dR_by_dqz(q(0), q(1), q(2), q(3), t33);
	dposw_dq.block<3, 1>(0, 3) = t33 * xyz;

}

/*
 * From a quaternion to its orthogonal rotation matrix. Note that inv(q_rotation_matrix) = q_rotation_matrix' !! (cheap inverse)
 *
 */
void MotionModel::quaternion_matrix(const Eigen::Vector4d & q, Eigen::Matrix3d & q_rotation_matrix){
	double w=q(0);
	double x=q(1);
	double y=q(2);
	double z=q(3);

	q_rotation_matrix << w*w+x*x-y*y-z*z,    2*(x*y -w*z),     2*(z*x+w*y),
					     2*(x*y+w*z)    , w*w-x*x+y*y-z*z,     2*(y*z-w*x),
					     2*(z*x-w*y)    ,     2*(y*z+w*x), w*w-x*x-y*y+z*z;
}
