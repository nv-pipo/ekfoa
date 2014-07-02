#include "feature.hpp"


/*
 * compute_h:
 * Using the predicted state of the feature (yi), compute 'h'. Where 'h' is the state interpretation in image coordinates.
 * Basically it projects the predicted 3D point into the image.
 */
bool Feature::compute_h(const Camera & cam, const Eigen::Vector3d & rW, const Eigen::Matrix3d & qWR_rotation_matrix, const Eigen::VectorXd & yi, Eigen::Vector2d & hi){
	//TODO: provide a better integration of the tracking algorithm. 'h' can be used to significantly reduce the search space of the tracker:

	Eigen::Vector3d hrl;
	hrl = qWR_rotation_matrix.transpose() * Feature::compute_unshifted_3d_position(rW, yi);

	Eigen::Vector2d uvu;
	cam.hc_to_undistorted(hrl, uvu);

	Eigen::Vector2d uvd;
	cam.distort_fm(uvu, uvd);

	hi = uvd;

	return true;
}

Eigen::Vector3d Feature::compute_cartesian(const Eigen::VectorXd & yi){
	//TODO: Asserts

	const Eigen::VectorXd & yi_rW = yi.head(3); //camera position when it was first seen.
	double theta = yi(3);
	double phi = yi(4);
	double rho = yi(5);

	Eigen::Vector3d mi;
	Feature::compute_m( theta, phi, mi );

	return yi_rW + (1/rho)*mi;
}

Eigen::Vector3d Feature::compute_unshifted_3d_position(const Eigen::Vector3d & rW, const Eigen::VectorXd & yi){

	const Eigen::Vector3d & yi_rW = yi.head<3>(); //camera orientation when it was first seen.
	double theta = yi(3);
	double phi = yi(4);
	double rho = yi(5);

	Eigen::Vector3d mi;
	Feature::compute_m( theta, phi, mi );

	return ((yi_rW - rW)*rho + mi);
}

/*
 * compute_H:
 * Computes the derivative of the function h with respect to x, a Jacobian of dh/dx = H.
 */
void Feature::compute_H(const Camera & cam, const Eigen::Vector3d & rW, const Eigen::Vector4d & qWR, const Eigen::Matrix3d & qWR_rotation_matrix, const Eigen::VectorXd & x_k_k, const Eigen::VectorXd & yi, const int yi_start_pos, const Eigen::Vector2d & hi, Eigen::MatrixXd & Hi){

	Eigen::Matrix3d qWR_rotation_matrix_inverse = qWR_rotation_matrix.inverse();

	//Initialize the Jacobian 'H' column for this feature:
	Hi.setZero(2, x_k_k.size());

	/*
	 * Set the derivative of this feature against the camera position:
	 */
	//dh_drw: predicted state in image coordinates(hi) against position (rW)
	Eigen::Matrix2d dhd_dhu;
	cam.jacob_undistor_fm( hi, dhd_dhu );
	dhd_dhu = dhd_dhu.inverse().eval();

	Eigen::Matrix<double, 2, 3>  dhu_dhrl;
	Eigen::Vector3d hC;
	hC = qWR_rotation_matrix.inverse() * Feature::compute_unshifted_3d_position(rW, yi);

	cam.get_dhu_dhrl(hC, dhu_dhrl);

	Eigen::Matrix3d dhrl_drw = - qWR_rotation_matrix_inverse * yi(5);

	Eigen::Matrix<double, 2, 3> dh_dhrl = dhd_dhu*dhu_dhrl;

	Hi.block<2, 3>(0, 0) = dh_dhrl * dhrl_drw; //dh_drw

	//dh_dqwr: predicted state in image coordinates(hi) against orientation (qWR)
	Eigen::Matrix<double, 3, 4> dhrl_dqwr;
	Eigen::Matrix<double, 3, 4> dRq_times_a_by_dq;
	Eigen::Vector4d qbar;
	MotionModel::qconj(qWR, qbar);
	MotionModel::dposw_dq(Feature::compute_unshifted_3d_position(rW, yi), qbar, dRq_times_a_by_dq);
	Eigen::Matrix4d dqbar_by_dq = Eigen::Vector4d(1,-1,-1,-1).asDiagonal();

	dhrl_dqwr = dRq_times_a_by_dq * dqbar_by_dq;

	Hi.block<2, 4>(0, 3) = dh_dhrl * dhrl_dqwr; //dh_dqwr


	/*
	 * Set dh_dy: predicted state in image coordinates(hi) against feature state (yi)
	 */
	//dhrl_dy:
	Eigen::Matrix<double, 3, 6> dhrl_dy;
	Feature::compute_dhrl_dy(rW, qWR_rotation_matrix_inverse, yi, dhrl_dy);

	Hi.block<2, 6>(0, yi_start_pos) = dh_dhrl * dhrl_dy; //dh_dy
}


//compute the ray directional vector
void Feature::compute_m( const double theta, const double phi, Eigen::Vector3d & mi ){
    double cphi = cos(phi);
    mi(0) = cphi*sin(theta);
    mi(1) = -sin(phi);
	mi(2) = cphi*cos(theta);
}

void Feature::compute_dhrl_dy(const Eigen::Vector3d & rW, const Eigen::Matrix3d & qWR_rotation_matrix_inverse, const Eigen::VectorXd & yi, Eigen::Matrix<double, 3, 6> & dhrl_dy){
    double theta = yi(3);
    double phi = yi(4);
    double lambda = yi(5);

    Eigen::Vector3d dmi_dthetai = qWR_rotation_matrix_inverse * Eigen::Vector3d(cos(phi)*cos(theta), 0, -cos(phi)*sin(theta));

    Eigen::Vector3d dmi_dphii = qWR_rotation_matrix_inverse * Eigen::Vector3d(-sin(phi)*sin(theta), -cos(phi), -sin(phi)*cos(theta));

    dhrl_dy.block<3, 3>(0, 0) = lambda*qWR_rotation_matrix_inverse;
    dhrl_dy.block<3, 1>(0, 3) = dmi_dthetai;
    dhrl_dy.block<3, 1>(0, 4) = dmi_dphii;
    dhrl_dy.block<3, 1>(0, 5) = qWR_rotation_matrix_inverse*(yi.segment<3>(0)-rW);
}

