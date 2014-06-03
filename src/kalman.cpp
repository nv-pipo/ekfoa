#include "kalman.hpp"

Kalman::Kalman(double v_0, double std_v_0, double w_0, double std_w_0, double sigma_a, double sigma_alpha, double sigma_image_noise){

	//Init with zeros the state vector and covariance matrice:
	x_k_k_.setZero(13);
	p_k_k_.setZero(13, 13);

	x_k_k_ << 0, //Camera X position
			0, //Camera Y position
			0, //Camera Z position
			1, //Orientation quaternion 'w'
			0, //Orientation quaternion 'x'
			0, //Orientation quaternion 'y'
			0, //Orientation quaternion 'z'
			v_0, //Velocity X axis
			v_0, //Velocity Y axis
			v_0, //Velocity Z axis
			w_0, //Angular Velocity X
			w_0, //Angular Velocity Y
			w_0; //Angular Velocity Z

	double eps = 1/pow(2, 52); //TODO: I'm guessing that EPS is used to refer to the smallest number that can be produced and is not zero.
	//	p_k_k(0,0) = std::numeric_limits<double>::min();
	//	p_k_k(1,1) = std::numeric_limits<double>::min();
	//	p_k_k(2,2) = std::numeric_limits<double>::min();
	//	p_k_k(3,3) = std::numeric_limits<double>::min();
	//	p_k_k(4,4) = std::numeric_limits<double>::min();
	//	p_k_k(5,5) = std::numeric_limits<double>::min();
	//	p_k_k(6,6) = std::numeric_limits<double>::min();
	p_k_k_(0,0) = eps;
	p_k_k_(1,1) = eps;
	p_k_k_(2,2) = eps;
	p_k_k_(3,3) = eps;
	p_k_k_(4,4) = eps;
	p_k_k_(5,5) = eps;
	p_k_k_(6,6) = eps;
	p_k_k_(7,7) = std_v_0*std_v_0;
	p_k_k_(8,8) = std_v_0*std_v_0;
	p_k_k_(9,9) = std_v_0*std_v_0;
	p_k_k_(10,10) = std_w_0*std_w_0;
	p_k_k_(11,11) = std_w_0*std_w_0;
	p_k_k_(12,12) = std_w_0*std_w_0;

	std_a_ = sigma_a;
	std_alpha_ = sigma_alpha;
	std_z_ = sigma_image_noise;
}

/*
 * Construct a calman filter with the given state, covariance matrix and deviations. Mainly used for testing.
 */
Kalman::Kalman(const Eigen::VectorXd & x_k_k, const Eigen::MatrixXd & p_k_k, double sigma_a, double sigma_alpha, double sigma_image_noise){
	x_k_k_ = x_k_k;
	p_k_k_ = p_k_k;

	std_a_ = sigma_a;
	std_alpha_ = sigma_alpha;
	std_z_ = sigma_image_noise;
}

void Kalman::delete_features(std::vector<int> & delete_list){
	if (delete_list.size()==0)
		return;

	int new_size = x_k_k_.rows();

	//Start deleting from the last feature to be deleted (simplifies and optimizes the way it is done):
	for(int i = delete_list.size()-1; i >= 0; i--) {
		unsigned int start_dst = 13 + delete_list[i]*6; //the feature to be delete starts at: xv_size + feature_index*6.
		unsigned int start_src = start_dst + 6; //next feature starts at 'start_dst + 6'
		unsigned int size_rest = x_k_k_.rows() - start_src; //relevant size minus space that does not need to change


		//shift rows up. Of state and covariance matrix:
		x_k_k_.segment(start_dst, size_rest) = x_k_k_.segment(start_src, size_rest);
		p_k_k_.block(start_dst, 0, size_rest, new_size) = p_k_k_.block(start_src, 0, size_rest, new_size);//reducing new_size before here would result in missing one column

		//Now we can reduce the size:
		new_size -= 6;

		//shift columns left. Of covariance matrix:
		p_k_k_.block(0, start_dst, new_size, size_rest) = p_k_k_.block(0, start_src, new_size, size_rest);
	}

	x_k_k_.conservativeResize(new_size);
	p_k_k_.conservativeResize(new_size,new_size);
}

void Kalman::predict_state_and_covariance(const double delta_t){
	//Return if no features are observed:
	if (x_k_k_.rows() == 13)
		return;

	Eigen::MatrixXd F;
	Eigen::MatrixXd Q;
	MotionModel::prediction_step(delta_t, std_a_, std_alpha_, x_k_k_, F, Q);


	//Update the covariance matrix as follows:
	//  size_P_k = size(P_k,1);
	//	p_k_k = [ F*p_k_k(1:13,1:13)*F' + Q         F*p_k_k(1:13,14:size_P_k);
	//	          p_k_k(14:size_P_k,1:13)*F'        p_k_k(14:size_P_k,14:size_P_k)];

	int size_p_k_k_minus_xv = p_k_k_.rows()-13;
	p_k_k_.block(0, 0, 13, 13) = (F*p_k_k_.block(0, 0, 13, 13)*F.transpose() + Q).eval();
	p_k_k_.block(0, 13, 13, size_p_k_k_minus_xv) = (F*p_k_k_.block(0, 13, 13, size_p_k_k_minus_xv)).eval();
	p_k_k_.block(13, 0, size_p_k_k_minus_xv, 13) = (p_k_k_.block(13, 0, size_p_k_k_minus_xv, 13)*F.transpose()).eval();
}

void Kalman::add_features_inverse_depth( const Camera & cam, const Eigen::MatrixXd & list_uvd ){
	if (list_uvd.cols() == 0)
		return;

	//num new features
	int new_features = list_uvd.cols();
	//Where next feature state should start:
	int insert_point = x_k_k_.rows();

	//resize the state and covariance estimate:
	x_k_k_.conservativeResize(x_k_k_.rows() + 6*new_features);
	p_k_k_.conservativeResize(p_k_k_.rows() + 6*new_features, p_k_k_.cols() + 6*new_features);

	for (int p=0 ; p<new_features ; p++){
		const Eigen::Vector2d & uv_d = list_uvd.col(p);
		Eigen::Vector3d xy_u;
		cam.undistort_center(uv_d, xy_u);

		//Extract orientation quaterion from state vector. Used to calculate the
		Eigen::Vector4d qWR(x_k_k_(3), x_k_k_(4), x_k_k_(5), x_k_k_(6));

		Eigen::Matrix3d qWR_rotation_matrix;
		MotionModel::quaternion_matrix(qWR, qWR_rotation_matrix);

		Eigen::Vector3d XYZ_w = qWR_rotation_matrix*xy_u;

		//Add point information to the state:
		add_a_feature_state_inverse_depth( XYZ_w, insert_point );

		//Add point information to the covariance matrix:
		add_a_feature_covariance_inverse_depth( cam, uv_d, xy_u, qWR, qWR_rotation_matrix, XYZ_w, insert_point );

		insert_point += 6;
	}
}

void Kalman::add_a_feature_covariance_inverse_depth( const Camera & cam, const Eigen::Vector2d & uv_d, const Eigen::Vector3d & xy_u, const Eigen::Vector4d & qWR, const Eigen::Matrix3d & qWR_rotation_matrix , const Eigen::Vector3d & XYZ_w, const int insert_point ){

	double X_w = XYZ_w(0);
	double Y_w = XYZ_w(1);
	double Z_w = XYZ_w(2);

	// Derivatives
	Eigen::RowVector3d dtheta_dgw;
	dtheta_dgw << Z_w/(X_w*X_w+Z_w*Z_w), 0, -X_w/(X_w*X_w+Z_w*Z_w);

	Eigen::RowVector3d dphi_dgw;
	dphi_dgw << (X_w*Y_w)/((X_w*X_w+Y_w*Y_w+Z_w*Z_w)*sqrt(X_w*X_w+Z_w*Z_w)), -sqrt(X_w*X_w+Z_w*Z_w)/(X_w*X_w+Y_w*Y_w+Z_w*Z_w), (Z_w*Y_w)/((X_w*X_w+Y_w*Y_w+Z_w*Z_w)*sqrt(X_w*X_w+Z_w*Z_w));

	Eigen::Matrix<double, 3, 4> dgw_dqwr;
	MotionModel::dposw_dq(xy_u, qWR, dgw_dqwr);

	Eigen::RowVector4d dtheta_dqwr = dtheta_dgw*dgw_dqwr;
	Eigen::RowVector4d dphi_dqwr = dphi_dgw*dgw_dqwr;

	Eigen::Matrix<double, 6, 4> dy_dqwr;
	dy_dqwr.setZero();
	dy_dqwr.block<1, 4>(3, 0) = dtheta_dqwr;
	dy_dqwr.block<1, 4>(4, 0) = dphi_dqwr;

	Eigen::MatrixXd dy_drw(6, 3);
	dy_drw.setZero();
	dy_drw.block<3, 3>(0, 0).setIdentity();

	Eigen::Matrix<double, 6, 13> dy_dxv;
	dy_dxv.setZero();
	dy_dxv.block<6, 3>(0, 0) = dy_drw;
	dy_dxv.block<6, 4>(0, 3) = dy_dqwr;

	Eigen::Matrix<double, 5, 3> dyprima_dgw;
	dyprima_dgw.setZero();
	dyprima_dgw.block<1, 3>(3, 0) = dtheta_dgw;
	dyprima_dgw.block<1, 3>(4, 0) = dphi_dgw;

	Eigen::Matrix<double, 3, 2> dgc_dhu;
	cam.get_dgc_dhu(dgc_dhu);

	Eigen::Matrix2d dhu_dhd;
	cam.jacob_undistor_fm( uv_d, dhu_dhd );

	Eigen::Matrix<double, 5, 2> dyprima_dhd = dyprima_dgw*qWR_rotation_matrix*dgc_dhu*dhu_dhd; //dgw_dgc = R_wc = qWR.toRotationMatrix();

	Eigen::Matrix<double, 6, 3> dy_dhd;
	dy_dhd.setZero();
	dy_dhd.block<5, 2>(0, 0) = dyprima_dhd;
	dy_dhd(5,2) = 1;

	Eigen::Matrix3d Padd;
	Padd.setIdentity(); //Initial std_pxl = 1, and std_rho = 1. Block(0,0,1,1) = I * std_pxl^2, pos(2, 2) = std_rho^2


	//	P_xv = P( 1:13, 1:13 ); //Correlation of Camera with itself
	//	P_yxv = P( 14:end, 1:13 ); //Correlation of features with camera
	//	P_y = P( 14:end, 14:end ); //Correlation of features with features
	//	P_xvy = P( 1:13, 14:end ); //Correlation of camera with features
	//	p_k_k = [ P_xv          P_xvy                       P_xv*dy_dxv';
	//	          P_yxv         P_y                         P_yxv*dy_dxv';
	//	          dy_dxv*P_xv   dy_dxv*P_xvy                dy_dxv*P_xv*dy_dxv'+...
	//	                                                    dy_dhd*Padd*dy_dhd'];


	//Note: p_x_x might be large, so using dynamic allocation to modify it.
	// Correlation of the new feature with the camera state info (first 13 positions) and viceversa:
	p_k_k_.block(0, insert_point, 13, 6) = p_k_k_.topLeftCorner(13,13)*dy_dxv.transpose(); //P_xv*dy_dxv'
	p_k_k_.block(insert_point, 0, 6, 13) = dy_dxv*p_k_k_.topLeftCorner(13,13); //dy_dxv*P_xv

	//dy_dxv*P_xv*dy_dxv'+dy_dhd*Padd*dy_dhd' = correlation between this feature properties:
	p_k_k_.block(insert_point, insert_point, 6, 6) = dy_dxv*p_k_k_.topLeftCorner(13,13)*dy_dxv.transpose() + dy_dhd*Padd*dy_dhd.transpose();

//	//If other features already existed, compute the correlation of this feature with the old features
	if (insert_point > 13) {
		//Correlation of old features with camera:
		Eigen::MatrixXd P_yxv = p_k_k_.block(13, 0, insert_point-13, 13);
		Eigen::MatrixXd P_xvy = p_k_k_.block(0, 13, 13, insert_point-13);

		p_k_k_.block(13, insert_point, insert_point-13, 6) = P_yxv*dy_dxv.transpose(); //P_yxv*dy_dxv' = correlation of the new feature with the already existing features (pos 14-> insert_point)
		p_k_k_.block(insert_point, 13, 6, insert_point-13) = dy_dxv*P_xvy; //dy_dxv*P_xvy = correlation of the new feature with the already existing features (pos 14-> insert_point)
	}
}

void Kalman::add_a_feature_state_inverse_depth( const Eigen::VectorXd & XYZ_w, const int insert_point){
	Eigen::VectorXd newFeature(6);

	//A projected point is expressed in terms of the camera position when it was first seen.
	//So the first 3 positions are equal to the first 3 positions of the state vector (the translation):
	newFeature.head(3) = x_k_k_.head(3);

	//The last three are the azimuth, elevation and ray length (distance to the camera position). It is calculated from the quaternion that describes the orientation of the camera:
	// XYZ_w is the undistorted homogeneous coordinates rotated by the orientation (qWR). In other words XYZ_w is the direction vector of the ray.
	double nx=XYZ_w(0);
	double ny=XYZ_w(1);
	double nz=XYZ_w(2);

	//TODO: convert azimuth and elevation to homogeneous coordinates not from angles but from the positions in the image! (Anchored homogeneous points)
	newFeature(3) = std::atan2(nx,nz); //azimuth
	newFeature(4) = std::atan2(-ny,sqrt(nx*nx+nz*nz)); //elevation
	newFeature(5) = 1 ; //Initially guessed ray length (a positive number since it has to be in front of the camera, the EKF takes care to later improve this guess)

	x_k_k_.segment(insert_point, 6) = newFeature;
}

/*
 * update:
 * With the camera parameters and observations (mapped to the current state features) it corrects the state and covariance matrix of the EKF.
 */
void Kalman::update( const Camera & cam, const std::vector<Observation> & observations ){
	//Return if there were no observations:
	if (observations.size() == 0)
		return;

	Eigen::Vector3d rW = x_k_k_.head(3); //current camera position
	Eigen::Vector4d qWR = x_k_k_.segment(3, 4);//current camera orientation
	Eigen::Matrix3d qWR_rotation_matrix;
	MotionModel::quaternion_matrix(qWR, qWR_rotation_matrix);

	std::vector<Eigen::Vector2d> list_h;
	std::vector<Eigen::MatrixXd> list_H;
	int num_observations = 0;

	//compute 'h' and its Jacobian 'H':
	for(size_t i = 0; i != observations.size(); i++) {
		int yi_start_pos = observations[i].feature_idx * 6 + 13;
		Eigen::VectorXd yi = x_k_k_.segment(yi_start_pos, 6); //feature_state
		Eigen::Vector2d hi; //this feature state estimation represented in image coordinates
		Eigen::MatrixXd Hi; //this feature derivative against the current state (x_k_k)

		Feature::compute_h( cam, rW, qWR_rotation_matrix, yi, hi );
		Feature::compute_H( cam, rW, qWR, qWR_rotation_matrix, x_k_k_, yi, yi_start_pos, hi, Hi );
		num_observations++;
		list_h.push_back(hi);
		list_H.push_back(Hi);
	}

	Eigen::VectorXd z(num_observations*2); //each observation uses 2 doubles, for U and V.
	Eigen::VectorXd h(num_observations*2); //each observation uses 2 doubles, for the predicted U and V.
	Eigen::MatrixXd H(num_observations*2, x_k_k_.rows());
	Eigen::MatrixXd R;
	R.setIdentity(num_observations*2, num_observations*2);
	for (size_t i = 0; i != list_h.size(); i++) {
		z.segment(i*2, 2) = observations[i].uvd;
		h.segment(i*2, 2) = list_h[i];
		H.block(i*2, 0, 2, x_k_k_.rows()) = list_H[i];
	}

	//TODO: Optimize in a per feature basis and maybe parallelize, as Joan Sola's "SLAM course.pdf" suggest.
	//filter gain
	Eigen::MatrixXd S = H*p_k_k_*H.transpose() + R;
	Eigen::MatrixXd K = p_k_k_*H.transpose()*S.inverse();

	//updated state and covariance
	x_k_k_ += K*( z - h );
	p_k_k_ -= K*S*K.transpose();

	//normalize the quaternion
	Eigen::Matrix4d Jnorm;
	Kalman::normalize_jac( x_k_k_.segment<4>(3), Jnorm );


	//////It is updated as follows:
	//	    p_k_k = [          p_k_k(1:3,1:3)              p_k_k(1:3,4:7)*Jnorm'               p_k_k(1:3,8:size_p_k_k);
	//	                 Jnorm*p_k_k(4:7,1:3)        Jnorm*p_k_k(4:7,4:7)*Jnorm'         Jnorm*p_k_k(4:7,8:size_p_k_k);
	//	              p_k_k(8:size_p_k_k,1:3)     p_k_k(8:size_p_k_k,4:7)*Jnorm'      p_k_k(8:size_p_k_k,8:size_p_k_k)];

	//Update the covariance matrix that are related to the quaternion (qWC):
	//cols:
	p_k_k_.block(3, 0, 4, p_k_k_.cols()).applyOnTheLeft(Jnorm); // p_k_k(4:7, :) = Jnorm * p_k_k(4:7, :)

	//rows:
	p_k_k_.block(0, 3, p_k_k_.rows(), 4).applyOnTheRight(Jnorm.transpose()); // p_k_k(:, 4:7) = p_k_k(:, 4:7) * Jnorm'
}
