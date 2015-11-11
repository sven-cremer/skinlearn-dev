#include <ice_robot_controllers/outer_loop.h>

outer_loop::outer_loop(){
	bool result=true;

	outerLoopWk.resize(8,1);
	outerLoopWk_flexi_1.resize(8,1);
	outerLoopWk_flexi_2.resize(8,1);
	outerLoopWk_flexi_3.resize(8,1);
	outerLoopWk_flexi_4.resize(8,1);


	if( useFIRmodel || useARMAmodel || useCTARMAmodel)
	{
		outerLoopWk(0,0) = filtW0 ; outerLoopWk_flexi_1(0,0) = flex_1_filtW0 ; outerLoopWk_flexi_2(0,0) = flex_2_filtW0 ; outerLoopWk_flexi_3(0,0) = flex_3_filtW0 ; outerLoopWk_flexi_4(0,0) = flex_4_filtW0 ;
		outerLoopWk(1,0) = filtW1 ; outerLoopWk_flexi_1(1,0) = flex_1_filtW1 ; outerLoopWk_flexi_2(1,0) = flex_2_filtW1 ; outerLoopWk_flexi_3(1,0) = flex_3_filtW1 ; outerLoopWk_flexi_4(1,0) = flex_4_filtW1 ;
		outerLoopWk(2,0) = filtW2 ; outerLoopWk_flexi_1(2,0) = flex_1_filtW2 ; outerLoopWk_flexi_2(2,0) = flex_2_filtW2 ; outerLoopWk_flexi_3(2,0) = flex_3_filtW2 ; outerLoopWk_flexi_4(2,0) = flex_4_filtW2 ;
		outerLoopWk(3,0) = filtW3 ; outerLoopWk_flexi_1(3,0) = flex_1_filtW3 ; outerLoopWk_flexi_2(3,0) = flex_2_filtW3 ; outerLoopWk_flexi_3(3,0) = flex_3_filtW3 ; outerLoopWk_flexi_4(3,0) = flex_4_filtW3 ;
		outerLoopWk(4,0) = filtW4 ; outerLoopWk_flexi_1(4,0) = flex_1_filtW4 ; outerLoopWk_flexi_2(4,0) = flex_2_filtW4 ; outerLoopWk_flexi_3(4,0) = flex_3_filtW4 ; outerLoopWk_flexi_4(4,0) = flex_4_filtW4 ;
		outerLoopWk(5,0) = filtW5 ; outerLoopWk_flexi_1(5,0) = flex_1_filtW5 ; outerLoopWk_flexi_2(5,0) = flex_2_filtW5 ; outerLoopWk_flexi_3(5,0) = flex_3_filtW5 ; outerLoopWk_flexi_4(5,0) = flex_4_filtW5 ;
		outerLoopWk(6,0) = filtW6 ; outerLoopWk_flexi_1(6,0) = flex_1_filtW6 ; outerLoopWk_flexi_2(6,0) = flex_2_filtW6 ; outerLoopWk_flexi_3(6,0) = flex_3_filtW6 ; outerLoopWk_flexi_4(6,0) = flex_4_filtW6 ;
		outerLoopWk(7,0) = filtW7 ; outerLoopWk_flexi_1(7,0) = flex_1_filtW7 ; outerLoopWk_flexi_2(7,0) = flex_2_filtW7 ; outerLoopWk_flexi_3(7,0) = flex_3_filtW7 ; outerLoopWk_flexi_4(7,0) = flex_4_filtW7 ;
	}else
	{
		outerLoopWk(0,0) = 0.0 ; outerLoopWk_flexi_1(0,0) = 0.0 ; outerLoopWk_flexi_2(0,0) = 0.0 ; outerLoopWk_flexi_3(0,0) = 0.0 ; outerLoopWk_flexi_4(0,0) = 0.0 ;
		outerLoopWk(1,0) = 0.0 ; outerLoopWk_flexi_1(1,0) = 0.0 ; outerLoopWk_flexi_2(1,0) = 0.0 ; outerLoopWk_flexi_3(1,0) = 0.0 ; outerLoopWk_flexi_4(1,0) = 0.0 ;
		outerLoopWk(2,0) = 0.0 ; outerLoopWk_flexi_1(2,0) = 0.0 ; outerLoopWk_flexi_2(2,0) = 0.0 ; outerLoopWk_flexi_3(2,0) = 0.0 ; outerLoopWk_flexi_4(2,0) = 0.0 ;
		outerLoopWk(3,0) = 0.0 ; outerLoopWk_flexi_1(3,0) = 0.0 ; outerLoopWk_flexi_2(3,0) = 0.0 ; outerLoopWk_flexi_3(3,0) = 0.0 ; outerLoopWk_flexi_4(3,0) = 0.0 ;
		outerLoopWk(4,0) = 0.0 ; outerLoopWk_flexi_1(4,0) = 0.0 ; outerLoopWk_flexi_2(4,0) = 0.0 ; outerLoopWk_flexi_3(4,0) = 0.0 ; outerLoopWk_flexi_4(4,0) = 0.0 ;
		outerLoopWk(5,0) = 0.0 ; outerLoopWk_flexi_1(5,0) = 0.0 ; outerLoopWk_flexi_2(5,0) = 0.0 ; outerLoopWk_flexi_3(5,0) = 0.0 ; outerLoopWk_flexi_4(5,0) = 0.0 ;
		outerLoopWk(6,0) = 0.0 ; outerLoopWk_flexi_1(6,0) = 0.0 ; outerLoopWk_flexi_2(6,0) = 0.0 ; outerLoopWk_flexi_3(6,0) = 0.0 ; outerLoopWk_flexi_4(6,0) = 0.0 ;
		outerLoopWk(7,0) = 0.0 ; outerLoopWk_flexi_1(7,0) = 0.0 ; outerLoopWk_flexi_2(7,0) = 0.0 ; outerLoopWk_flexi_3(7,0) = 0.0 ; outerLoopWk_flexi_4(7,0) = 0.0 ;
	}


	numIrlSamples = 100;
	numIrlLsIter = 10;
	numCartDof = 1;
	irlOneshot = true;
	rls_lambda = 0.98 ;
	rls_sigma  = 1000 ;

	mrac_gamma_1 = 1 ;
	mrac_gamma_2 = 1 ;
	mrac_gamma_3 = 1 ;
	mrac_gamma_4 = 1 ;
	mrac_gamma_5 = 1 ;



	mrac_P_m = 1 ;
	mrac_P_h = 1 ;


	//TODO The only place I can tell that KDL is used. To get joint_names
	for (int i = 0; i < num_Joints; ++i){
		nh_.param("saturation/" + chain_.getJoint(i)->joint_->name, saturation_[i], 0.0);
	}

	delT = 0.001;


	// initial conditions
	ode_init_x[0 ] = 0.0;
	ode_init_x[1 ] = 0.0;
	ode_init_x[2 ] = 0.0;
	ode_init_x[3 ] = 0.0;

	/////////////////////////
	// System Model

	// FIXME remove below stuff
	num_Inputs  = 44 ;
	num_Outputs = 6  ; // 6 for 6 cart dof
	//  num_Hidden  = 100;
	num_Error   = 6  ;
	num_Joints  = 7  ;

	kdl_temp_joint_.resize( num_Joints );
	eigen_temp_joint.resize( num_Joints,1 );

	q       .resize( num_Joints ) ;
	qd      .resize( num_Joints ) ;
	qdd     .resize( num_Joints ) ;

	q_m     .resize( num_Outputs ) ;
	qd_m    .resize( num_Outputs ) ;
	qdd_m   .resize( num_Outputs ) ;

	// desired Cartesian states
	X_m     .resize( num_Outputs ) ;
	Xd_m    .resize( num_Outputs ) ;
	Xdd_m   .resize( num_Outputs ) ;

	// Prev desired Cartesian states
	p_X_m     .resize( num_Outputs ) ;
	p_Xd_m    .resize( num_Outputs ) ;
	p_Xdd_m   .resize( num_Outputs ) ;

	// Cartesian states
	X       .resize( num_Outputs ) ;
	Xd      .resize( num_Outputs ) ;

	t_r     .resize( num_Outputs ) ;
	task_ref.resize( num_Outputs ) ;
	task_refModel_output.resize( num_Outputs ) ;
	tau     .resize( num_Outputs ) ;

	q        = Eigen::VectorXd::Zero( num_Joints ) ;
	qd       = Eigen::VectorXd::Zero( num_Joints ) ;
	qdd      = Eigen::VectorXd::Zero( num_Joints ) ;
	q_m      = Eigen::VectorXd::Zero( num_Joints ) ;
	qd_m     = Eigen::VectorXd::Zero( num_Joints ) ;
	qdd_m    = Eigen::VectorXd::Zero( num_Joints ) ;

	X_m      = Eigen::VectorXd::Zero( num_Outputs ) ;
	Xd_m     = Eigen::VectorXd::Zero( num_Outputs ) ;
	Xdd_m    = Eigen::VectorXd::Zero( num_Outputs ) ;
	X        = Eigen::VectorXd::Zero( num_Outputs ) ;
	Xd       = Eigen::VectorXd::Zero( num_Outputs ) ;

	p_X_m    = Eigen::VectorXd::Zero( num_Outputs ) ;
	p_Xd_m   = Eigen::VectorXd::Zero( num_Outputs ) ;
	p_Xdd_m  = Eigen::VectorXd::Zero( num_Outputs ) ;

	X_m(0)   = cartIniX     ;
	X_m(1)   = cartIniY     ;
	X_m(2)   = cartIniZ     ;
	X_m(3)   = cartIniRoll  ;
	X_m(4)   = cartIniPitch ;
	X_m(5)   = cartIniYaw   ;

	p_X_m    = X_m   ;
	p_Xd_m   = Xd_m  ;
	p_Xdd_m  = Xdd_m ;

	transformed_force = Eigen::Vector3d::Zero();
	r_acc_data          = Eigen::Vector3d::Zero();

	t_r                  = Eigen::VectorXd::Zero( num_Outputs ) ;
	task_ref             = Eigen::VectorXd::Zero( num_Outputs ) ;
	task_refModel_output = Eigen::VectorXd::Zero( num_Outputs ) ;
	tau                  = Eigen::VectorXd::Zero( num_Outputs ) ;
	force                = Eigen::VectorXd::Zero( num_Outputs ) ;
	// FIXME remove this hardcoded 4 value
	flexiForce           = Eigen::VectorXd::Zero( 4 ) ;

	// Initial Reference
	task_ref = X_m ;

	Jacobian         = Eigen::MatrixXd::Zero( num_Outputs, kdl_chain_.getNrOfJoints() ) ;
	JacobianPinv     = Eigen::MatrixXd::Zero( kdl_chain_.getNrOfJoints(), 6 ) ;
	JacobianTrans    = Eigen::MatrixXd::Zero( kdl_chain_.getNrOfJoints(), 6 ) ;
	JacobianTransPinv= Eigen::MatrixXd::Zero( num_Outputs, kdl_chain_.getNrOfJoints() ) ;
	nullSpace        = Eigen::MatrixXd::Zero( kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfJoints() ) ;

	cartControlForce = Eigen::VectorXd::Zero( num_Outputs ) ;
	nullspaceTorque  = Eigen::VectorXd::Zero( kdl_chain_.getNrOfJoints() ) ;
	controlTorque    = Eigen::VectorXd::Zero( kdl_chain_.getNrOfJoints() ) ;

	/////////////////////////
	// Outer Loop Init

	// MRAC
	outerLoopMRACmodelX.updateDelT( outerLoopTime );
	outerLoopMRACmodelX.updateAB( task_mA,
			task_mB );
	outerLoopMRACmodelX.updateIni( cartIniX,
			cartIniX );
	outerLoopMRACmodelX.updateSimHuman( useSimHuman );
	outerLoopMRACmodelX.updateSimHuman( simHuman_a,
			simHuman_b );
	outerLoopMRACmodelX.updateGamma( mrac_gamma_1,
			mrac_gamma_2,
			mrac_gamma_3,
			mrac_gamma_4,
			mrac_gamma_5 ) ;
	outerLoopMRACmodelX.updateCov( mrac_P_m,
			mrac_P_h ) ;

	outerLoopMRACmodelY.updateDelT( outerLoopTime );
	outerLoopMRACmodelY.updateAB( task_mA,
			task_mB );
	outerLoopMRACmodelY.updateIni( cartIniY,
			cartIniY );
	outerLoopMRACmodelY.updateSimHuman( useSimHuman );
	outerLoopMRACmodelY.updateSimHuman( simHuman_a,
			simHuman_b );
	outerLoopMRACmodelY.updateGamma( mrac_gamma_1,
			mrac_gamma_2,
			mrac_gamma_3,
			mrac_gamma_4,
			mrac_gamma_5 ) ;
	outerLoopMRACmodelY.updateCov( mrac_P_m,
			mrac_P_h ) ;

	// RLS

	outerLoopRLSmodelX.updateDelT( outerLoopTime );
	outerLoopRLSmodelX.updateAB( task_mA,
			task_mB );
	outerLoopRLSmodelX.initRls( rls_lambda, rls_sigma );
	//  outerLoopRLSmodelX.initPos( cartIniX );


	outerLoopRLSmodelY.updateDelT( outerLoopTime );
	outerLoopRLSmodelY.updateAB( task_mA,
			task_mB );
	outerLoopRLSmodelY.initRls( rls_lambda, rls_sigma );
	//  outerLoopRLSmodelY.initPos( cartIniY );

	// CT RLS
	outerLoopCTRLSmodelX.updateDelT( outerLoopTime );
	outerLoopCTRLSmodelX.updateAB( task_mA,
			task_mB );

	outerLoopCTRLSmodelY.updateDelT( outerLoopTime );
	outerLoopCTRLSmodelY.updateAB( task_mA,
			task_mB );

	// MSD
	outerLoopMSDmodelX.updateDelT( outerLoopTime );
	outerLoopMSDmodelX.updateMsd( m_M,
			m_S,
			m_D );

	outerLoopMSDmodelY.updateDelT( outerLoopTime );
	outerLoopMSDmodelY.updateMsd( m_M,
			m_S,
			m_D );

	// IRL
	outerLoopIRLmodelX.init( numCartDof, numIrlSamples, numIrlLsIter, irlOneshot );
	outerLoopIRLmodelX.updateDelT( outerLoopTime );
	outerLoopIRLmodelX.updateMsd( m_M,
			m_S,
			m_D );
	outerLoopIRLmodelX.updateAB( task_mA,
			task_mB );

	outerLoopIRLmodelY.init( numCartDof, numIrlSamples, numIrlLsIter, irlOneshot );
	outerLoopIRLmodelY.updateDelT( outerLoopTime );
	outerLoopIRLmodelY.updateMsd( m_M,
			m_S,
			m_D );
	outerLoopIRLmodelY.updateAB( task_mA,
			task_mB );


	/////////////////////////

	// System Model END
	/////////////////////////


	/****return result;****/
}

bool outer_loop::load_parameters()
{
	std::string para_m_M = "/m_M" ;
	std::string para_m_S = "/m_S" ;
	std::string para_m_D = "/m_D" ;

	if (!nh_.getParam( para_m_M , m_M )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_m_M .c_str()) ; return false; }
	if (!nh_.getParam( para_m_S , m_S )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_m_S .c_str()) ; return false; }
	if (!nh_.getParam( para_m_D , m_D )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_m_D .c_str()) ; return false; }

	std::string para_task_mA = "/task_mA" ;
	std::string para_task_mB = "/task_mB" ;

	if (!nh_.getParam( para_task_mA , task_mA )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_task_mA .c_str()) ; return false; }
	if (!nh_.getParam( para_task_mB , task_mB )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_task_mB .c_str()) ; return false; }



	useCurrentCartPose = false ;
	std::string para_useCurrentCartPose     = "/useCurrentCartPose";
	if (!nh_.getParam( para_useCurrentCartPose, useCurrentCartPose )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useCurrentCartPose.c_str()) ; return false; }

	useNullspacePose = true ;
	std::string para_useNullspacePose     = "/useNullspacePose";
	if (!nh_.getParam( para_useNullspacePose, useNullspacePose )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useNullspacePose.c_str()) ; return false; }

	useFTinput = false ;
	std::string para_useFTinput   = "/useFTinput";
	if (!nh_.getParam( para_useFTinput, useFTinput )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useFTinput.c_str()) ; return false; }

	useARMAmodel = false ;
	std::string para_useARMAmodel = "/useARMAmodel";
	if (!nh_.getParam( para_useARMAmodel, useARMAmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useARMAmodel.c_str()) ; return false; }

	useCTARMAmodel = false ;
	std::string para_useCTARMAmodel = "/useCTARMAmodel";
	if (!nh_.getParam( para_useCTARMAmodel, useCTARMAmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useCTARMAmodel.c_str()) ; return false; }

	useFIRmodel = false ;
	std::string para_useFIRmodel = "/useFIRmodel";
	if (!nh_.getParam( para_useFIRmodel, useFIRmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useFIRmodel.c_str()) ; return false; }

	useMRACmodel = false ;
	std::string para_useMRACmodel = "/useMRACmodel";
	if (!nh_.getParam( para_useMRACmodel, useMRACmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useMRACmodel.c_str()) ; return false; }

	useMSDmodel = false ;
	std::string para_useMSDmodel = "/useMSDmodel";
	if (!nh_.getParam( para_useMSDmodel, useMSDmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useMSDmodel.c_str()) ; return false; }

	useIRLmodel = false ;
	std::string para_useIRLmodel = "/useIRLmodel";
	if (!nh_.getParam( para_useIRLmodel, useIRLmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useIRLmodel.c_str()) ; return false; }

	useDirectmodel = false ;
	std::string para_useDirectmodel = "/useDirectmodel";
	if (!nh_.getParam( para_useDirectmodel, useDirectmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useDirectmodel.c_str()) ; return false; }


	externalRefTraj = true ;
	std::string para_externalRefTraj = "/externalRefTraj";
	if (!nh_.getParam( para_externalRefTraj, externalRefTraj )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_externalRefTraj.c_str()) ; return false; }

	intentEst_delT = 0.1 ;
	std::string para_intentEst_delT = "/intentEst_delT";
	if (!nh_.getParam( para_intentEst_delT, intentEst_delT )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_intentEst_delT.c_str()) ; return false; }

	intentEst_M = 1.0 ;
	std::string para_intentEst_M = "/intentEst_M";
	if (!nh_.getParam( para_intentEst_M, intentEst_M )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_intentEst_M.c_str()) ; return false; }

	std::string para_forceCutOffX = "/forceCutOffX";
	std::string para_forceCutOffY = "/forceCutOffY";
	std::string para_forceCutOffZ = "/forceCutOffZ";

	if (!nh_.getParam( para_forceCutOffX , forceCutOffX )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_forceCutOffX.c_str()) ; return false; }
	if (!nh_.getParam( para_forceCutOffY , forceCutOffY )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_forceCutOffY.c_str()) ; return false; }
	if (!nh_.getParam( para_forceCutOffZ , forceCutOffZ )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_forceCutOffZ.c_str()) ; return false; }

	std::string para_filtW0        = "/filtW0"        ;
	std::string para_filtW1        = "/filtW1"        ;
	std::string para_filtW2        = "/filtW2"        ;
	std::string para_filtW3        = "/filtW3"        ;
	std::string para_filtW4        = "/filtW4"        ;
	std::string para_filtW5        = "/filtW5"        ;
	std::string para_filtW6        = "/filtW6"        ;
	std::string para_filtW7        = "/filtW7"        ;

	std::string para_flex_1_filtW0 = "/flex_1_filtW0" ;
	std::string para_flex_1_filtW1 = "/flex_1_filtW1" ;
	std::string para_flex_1_filtW2 = "/flex_1_filtW2" ;
	std::string para_flex_1_filtW3 = "/flex_1_filtW3" ;
	std::string para_flex_1_filtW4 = "/flex_1_filtW4" ;
	std::string para_flex_1_filtW5 = "/flex_1_filtW5" ;
	std::string para_flex_1_filtW6 = "/flex_1_filtW6" ;
	std::string para_flex_1_filtW7 = "/flex_1_filtW7" ;

	std::string para_flex_2_filtW0 = "/flex_2_filtW0" ;
	std::string para_flex_2_filtW1 = "/flex_2_filtW1" ;
	std::string para_flex_2_filtW2 = "/flex_2_filtW2" ;
	std::string para_flex_2_filtW3 = "/flex_2_filtW3" ;
	std::string para_flex_2_filtW4 = "/flex_2_filtW4" ;
	std::string para_flex_2_filtW5 = "/flex_2_filtW5" ;
	std::string para_flex_2_filtW6 = "/flex_2_filtW6" ;
	std::string para_flex_2_filtW7 = "/flex_2_filtW7" ;

	std::string para_flex_3_filtW0 = "/flex_3_filtW0" ;
	std::string para_flex_3_filtW1 = "/flex_3_filtW1" ;
	std::string para_flex_3_filtW2 = "/flex_3_filtW2" ;
	std::string para_flex_3_filtW3 = "/flex_3_filtW3" ;
	std::string para_flex_3_filtW4 = "/flex_3_filtW4" ;
	std::string para_flex_3_filtW5 = "/flex_3_filtW5" ;
	std::string para_flex_3_filtW6 = "/flex_3_filtW6" ;
	std::string para_flex_3_filtW7 = "/flex_3_filtW7" ;

	std::string para_flex_4_filtW0 = "/flex_4_filtW0" ;
	std::string para_flex_4_filtW1 = "/flex_4_filtW1" ;
	std::string para_flex_4_filtW2 = "/flex_4_filtW2" ;
	std::string para_flex_4_filtW3 = "/flex_4_filtW3" ;
	std::string para_flex_4_filtW4 = "/flex_4_filtW4" ;
	std::string para_flex_4_filtW5 = "/flex_4_filtW5" ;
	std::string para_flex_4_filtW6 = "/flex_4_filtW6" ;
	std::string para_flex_4_filtW7 = "/flex_4_filtW7" ;

	filtW0 = 0.0 ;  flex_1_filtW0 = 0.0 ;  flex_2_filtW0 = 0.0 ;  flex_3_filtW0 = 0.0 ;  flex_4_filtW0 = 0.0 ;
	filtW1 = 0.0 ;  flex_1_filtW1 = 0.0 ;  flex_2_filtW1 = 0.0 ;  flex_3_filtW1 = 0.0 ;  flex_4_filtW1 = 0.0 ;
	filtW2 = 0.0 ;  flex_1_filtW2 = 0.0 ;  flex_2_filtW2 = 0.0 ;  flex_3_filtW2 = 0.0 ;  flex_4_filtW2 = 0.0 ;
	filtW3 = 0.0 ;  flex_1_filtW3 = 0.0 ;  flex_2_filtW3 = 0.0 ;  flex_3_filtW3 = 0.0 ;  flex_4_filtW3 = 0.0 ;
	filtW4 = 0.0 ;  flex_1_filtW4 = 0.0 ;  flex_2_filtW4 = 0.0 ;  flex_3_filtW4 = 0.0 ;  flex_4_filtW4 = 0.0 ;
	filtW5 = 0.0 ;  flex_1_filtW5 = 0.0 ;  flex_2_filtW5 = 0.0 ;  flex_3_filtW5 = 0.0 ;  flex_4_filtW5 = 0.0 ;
	filtW6 = 0.0 ;  flex_1_filtW6 = 0.0 ;  flex_2_filtW6 = 0.0 ;  flex_3_filtW6 = 0.0 ;  flex_4_filtW6 = 0.0 ;
	filtW7 = 0.0 ;  flex_1_filtW7 = 0.0 ;  flex_2_filtW7 = 0.0 ;  flex_3_filtW7 = 0.0 ;  flex_4_filtW7 = 0.0 ;

	if (!nh_.getParam( para_filtW0        , filtW0        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW0       .c_str()) ; return false; }
	if (!nh_.getParam( para_filtW1        , filtW1        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW1       .c_str()) ; return false; }
	if (!nh_.getParam( para_filtW2        , filtW2        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW2       .c_str()) ; return false; }
	if (!nh_.getParam( para_filtW3        , filtW3        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW3       .c_str()) ; return false; }
	if (!nh_.getParam( para_filtW4        , filtW4        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW4       .c_str()) ; return false; }
	if (!nh_.getParam( para_filtW5        , filtW5        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW5       .c_str()) ; return false; }
	if (!nh_.getParam( para_filtW6        , filtW6        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW6       .c_str()) ; return false; }
	if (!nh_.getParam( para_filtW7        , filtW7        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW7       .c_str()) ; return false; }

	if (!nh_.getParam( para_flex_1_filtW0 , flex_1_filtW0 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW0.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_1_filtW1 , flex_1_filtW1 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW1.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_1_filtW2 , flex_1_filtW2 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW2.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_1_filtW3 , flex_1_filtW3 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW3.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_1_filtW4 , flex_1_filtW4 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW4.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_1_filtW5 , flex_1_filtW5 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW5.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_1_filtW6 , flex_1_filtW6 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW6.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_1_filtW7 , flex_1_filtW7 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW7.c_str()) ; return false; }

	if (!nh_.getParam( para_flex_2_filtW0 , flex_2_filtW0 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW0.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_2_filtW1 , flex_2_filtW1 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW1.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_2_filtW2 , flex_2_filtW2 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW2.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_2_filtW3 , flex_2_filtW3 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW3.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_2_filtW4 , flex_2_filtW4 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW4.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_2_filtW5 , flex_2_filtW5 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW5.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_2_filtW6 , flex_2_filtW6 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW6.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_2_filtW7 , flex_2_filtW7 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW7.c_str()) ; return false; }

	if (!nh_.getParam( para_flex_3_filtW0 , flex_3_filtW0 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW0.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_3_filtW1 , flex_3_filtW1 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW1.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_3_filtW2 , flex_3_filtW2 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW2.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_3_filtW3 , flex_3_filtW3 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW3.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_3_filtW4 , flex_3_filtW4 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW4.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_3_filtW5 , flex_3_filtW5 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW5.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_3_filtW6 , flex_3_filtW6 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW6.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_3_filtW7 , flex_3_filtW7 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW7.c_str()) ; return false; }

	if (!nh_.getParam( para_flex_4_filtW0 , flex_4_filtW0 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW0.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_4_filtW1 , flex_4_filtW1 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW1.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_4_filtW2 , flex_4_filtW2 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW2.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_4_filtW3 , flex_4_filtW3 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW3.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_4_filtW4 , flex_4_filtW4 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW4.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_4_filtW5 , flex_4_filtW5 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW5.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_4_filtW6 , flex_4_filtW6 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW6.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_4_filtW7 , flex_4_filtW7 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW7.c_str()) ; return false; }

	std::string para_numIrlSamples = "/numIrlSamples";
	if (!nh_.getParam( para_numIrlSamples , numIrlSamples )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_numIrlSamples.c_str()) ; return false; }
	std::string para_numIrlLsIter = "/numIrlLsIter";
	if (!nh_.getParam( para_numIrlLsIter , numIrlLsIter )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_numIrlLsIter.c_str()) ; return false; }
	std::string para_numCartDof = "/numCartDof";
	if (!nh_.getParam( para_numCartDof , numCartDof )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_numCartDof.c_str()) ; return false; }
	std::string para_irlOneshot = "/irlOneshot";
	if (!nh_.getParam( para_irlOneshot , irlOneshot )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_irlOneshot.c_str()) ; return false; }
	std::string para_fixedFilterWeights = "/fixedFilterWeights";
	if (!nh_.getParam( para_fixedFilterWeights , useFixedWeights )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_fixedFilterWeights.c_str()) ; return false; }

	std::string para_rls_lambda = "/rls_lambda";
	std::string para_rls_sigma  = "/rls_sigma";

	if (!nh_.getParam( para_rls_lambda , rls_lambda )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_rls_lambda.c_str()) ; return false; }
	if (!nh_.getParam( para_rls_sigma  , rls_sigma  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_rls_sigma .c_str()) ; return false; }

	std::string para_mrac_gamma_1 = "/mrac_gamma_1";
	std::string para_mrac_gamma_2 = "/mrac_gamma_2";
	std::string para_mrac_gamma_3 = "/mrac_gamma_3";
	std::string para_mrac_gamma_4 = "/mrac_gamma_4";
	std::string para_mrac_gamma_5 = "/mrac_gamma_5";

	if (!nh_.getParam( para_mrac_gamma_1 , mrac_gamma_1 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_gamma_1.c_str()) ; return false; }
	if (!nh_.getParam( para_mrac_gamma_2 , mrac_gamma_2 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_gamma_2.c_str()) ; return false; }
	if (!nh_.getParam( para_mrac_gamma_3 , mrac_gamma_3 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_gamma_3.c_str()) ; return false; }
	if (!nh_.getParam( para_mrac_gamma_4 , mrac_gamma_4 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_gamma_4.c_str()) ; return false; }
	if (!nh_.getParam( para_mrac_gamma_5 , mrac_gamma_5 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_gamma_5.c_str()) ; return false; }

	std::string para_mrac_P_m = "/mrac_P_m";
	std::string para_mrac_P_h = "/mrac_P_h";

	if (!nh_.getParam( para_mrac_P_m , mrac_P_m )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_P_m.c_str()) ; return false; }
	if (!nh_.getParam( para_mrac_P_h , mrac_P_h )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_P_h.c_str()) ; return false; }

	std::string para_outerLoopTime = "/outerLoop_time";
	if (!nh_.getParam( para_outerLoopTime , outerLoopTime )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_outerLoopTime.c_str()) ; return false; }

	intentLoopTime = outerLoopTime;
	std::string para_intentLoopTime = "/intentEst_time";
	if (!nh_.getParam( para_intentLoopTime , intentLoopTime )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_intentLoopTime.c_str()) ; return false; }

	useSimHuman = false ;
	std::string para_simHuman = "/useSimHuman";
	if (!nh_.getParam( para_simHuman , useSimHuman )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_simHuman.c_str()) ; return false; }

	std::string para_simHuman_a = "/simHuman_a" ;
	std::string para_simHuman_b = "/simHuman_b" ;

	if (!nh_.getParam( para_simHuman_a , simHuman_a )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_simHuman_a .c_str()) ; return false; }
	if (!nh_.getParam( para_simHuman_b , simHuman_b )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_simHuman_b .c_str()) ; return false; }

}

void outer_loop::calcHumanIntentPos( Eigen::Vector3d & force,
		Eigen::VectorXd & pos,
		double delT,
		double m )
{
	Eigen::Vector3d intentPos = Eigen::Vector3d::Zero();
	Eigen::Vector3d intentVel = Eigen::Vector3d::Zero();
	Eigen::Vector3d intentAcc = Eigen::Vector3d::Zero();

	Eigen::Vector3d M(m, m, m);

	intentAcc = force.cwiseQuotient(M);

	intentVel = intentVel + intentAcc * delT ;
	intentPos = intentPos + intentVel * delT ;

	pos = intentPos;
}

void outer_loop::update()
{
	// OUTER Loop Update

		// Human Intent Estimation
		if( !externalRefTraj )
		{
			//TODO Identify what the robot_state does and how to replace it
			//TODO Identify what intent_elapsed does and how to replace it
			if( ( robot_state_->getTime() - intent_elapsed_ ).toSec() >= intentLoopTime )
			{

				calcHumanIntentPos( transformed_force, task_ref, intentEst_delT, intentEst_M );

				// Transform human intent to torso lift link
				//TODO Identify what is x_gripper_acc_ and how we can replace it
				task_ref.x() = x_gripper_acc_.p.x() + task_ref.x() ;
				task_ref.y() = x_gripper_acc_.p.y() + task_ref.y() ;
				task_ref.z() = x_gripper_acc_.p.z() + task_ref.z() ;

				intent_elapsed_ = robot_state_->getTime() ;
			}
		}

		//TODO Identify what outer_elapsed does and how to replace it
		if( ( robot_state_->getTime() - outer_elapsed_ ).toSec() >= outerLoopTime )
		{
			// RLS ARMA
			if( useARMAmodel )
			{
				//TODO Identify how useFlexiForce is used and how to replace it
				if( useFlexiForce )
				{
					// Set ARMA parameters
					// X axis
					if( flexiForce(0) > flexiForce(2) ){

						outerLoopRLSmodelX.setWeights( outerLoopWk_flexi_1 ) ;
						if( useFixedWeights )
							outerLoopRLSmodelX.setFixedWeights( outerLoopWk_flexi_1 );
						else
							outerLoopRLSmodelX.setUpdatedWeights();

					}else{
						outerLoopRLSmodelX.setWeights( outerLoopWk_flexi_3 ) ;
						if( useFixedWeights )
							outerLoopRLSmodelX.setFixedWeights( outerLoopWk_flexi_3 );
						else
							outerLoopRLSmodelX.setUpdatedWeights();
					}

					// Y axis
					if( flexiForce(1) > flexiForce(3) ){
						outerLoopRLSmodelY.setWeights( outerLoopWk_flexi_2 ) ;
						if( useFixedWeights )
							outerLoopRLSmodelY.setFixedWeights( outerLoopWk_flexi_2 );
						else
							outerLoopRLSmodelY.setUpdatedWeights();
					}else{
						outerLoopRLSmodelY.setWeights( outerLoopWk_flexi_4 ) ;
						if( useFixedWeights )
							outerLoopRLSmodelY.setFixedWeights( outerLoopWk_flexi_4 );
						else
							outerLoopRLSmodelY.setUpdatedWeights();
					}
				}

				// X axis
				outerLoopRLSmodelX.updateARMA( Xd_m                   (0) ,
	                                           Xd                     (0) ,
	                                           X_m                    (0) ,
	                                           X                      (0) ,
	                                           Xdd_m                  (0) ,
	                                           transformed_force      (0) ,
	                                           task_ref               (0) ,
	                                           task_refModel_output   (0)  );

				// Y axis
				outerLoopRLSmodelY.updateARMA( Xd_m                   (1) ,
	                                           Xd                     (1) ,
	                                           X_m                    (1) ,
	                                           X                      (1) ,
	                                           Xdd_m                  (1) ,
	                                           transformed_force      (1) ,
	                                           task_ref               (1) ,
	                                           task_refModel_output   (1)  );

				// ROS_ERROR_STREAM("USING RLS ARMA");

				if( useFlexiForce )
				{
					// X axis
					if( flexiForce(0) > flexiForce(2) )
					{
						outerLoopRLSmodelX.getWeights( outerLoopWk_flexi_1 ) ;
					}else{
						outerLoopRLSmodelX.getWeights( outerLoopWk_flexi_3 ) ;
					}

					// Y axis
					if( flexiForce(1) > flexiForce(3) )
					{
						outerLoopRLSmodelY.getWeights( outerLoopWk_flexi_2 ) ;
					}else{
						outerLoopRLSmodelY.getWeights( outerLoopWk_flexi_4 ) ;
					}

				}else
				{
					outerLoopRLSmodelX.getWeights( outerLoopWk ) ;
					outerLoopRLSmodelY.getWeights( outerLoopWk ) ;
				}
				// Delete this
				//		    if( outerLoopWk_flexi_1.norm() == 0 && outerLoopWk_flexi_3.norm() == 0 )
				//		    {
				//		    	X_m(0) =  cartIniX     ;
				//		    }
				//
				//		    if(  outerLoopWk_flexi_2.norm() == 0 && outerLoopWk_flexi_4.norm() == 0  )
				//		    {
				//		    	X_m(1) =  cartIniY     ;
				//		    }

			}

			// CT RLS ARMA
			if( useCTARMAmodel )
			{
				//      outerLoopCTRLSmodelX.updateARMA( Xd_m              (0) ,
				//                                       Xd                (0) ,
				//                                       X_m               (0) ,
				//                                       X                 (0) ,
				//                                       Xdd_m             (0) ,
				//                                       transformed_force (0) ,
				//                                       task_ref          (0) ,
				//                                       task_refModel     (0)  );

				// Y axis
				outerLoopCTRLSmodelY.updateARMA( Xd_m                   (1) ,
	                                             Xd                     (1) ,
	                                             X_m                    (1) ,
	                                             X                      (1) ,
	                                             Xdd_m                  (1) ,
	                                             transformed_force      (1) ,
	                                             task_ref               (1) ,
	                                             task_refModel_output   (1)  );

				//      ROS_ERROR_STREAM("USING CT RLS ARMA");

				outerLoopCTRLSmodelY.getWeights( outerLoopWk ) ;
				//		outerLoopCTRLSmodelY.setWeights( outerLoopWk ) ;
			}

			// RLS FIR
			if( useFIRmodel )
			{
				//      outerLoopRLSmodelX.updateFIR( Xd_m              (0) ,
				//                                    Xd                (0) ,
				//                                    X_m               (0) ,
				//                                    X                 (0) ,
				//                                    Xdd_m             (0) ,
				//                                    transformed_force (0) ,
				//                                    task_ref          (0) ,
				//                                    task_refModel     (0)  );

				// Y axis
				outerLoopRLSmodelY.updateFIR( Xd_m                   (1) ,
	                                          Xd                     (1) ,
	                                          X_m                    (1) ,
	                                          X                      (1) ,
	                                          Xdd_m                  (1) ,
	                                          transformed_force      (1) ,
	                                          task_ref               (1) ,
	                                          task_refModel_output   (1)  );
				//      ROS_ERROR_STREAM("USING RLS FIR");

				outerLoopRLSmodelY.getWeights( outerLoopWk ) ;
				//		outerLoopRLSmodelY.setWeights( outerLoopWk ) ;
			}
			/*
				// MRAC
				if( useMRACmodel )
				{
			//      outerLoopMRACmodelX.update( Xd_m              (0) ,
			//                                  Xd                (0) ,
			//                                  X_m               (0) ,
			//                                  X                 (0) ,
			//                                  Xdd_m             (0) ,
			//                                  transformed_force (0) ,
			//                                  task_ref          (0) ,
			//                                  task_refModel     (0)  );

				  // Y axis
				  outerLoopMRACmodelY.update( Xd_m              (1) ,
											  Xd                (1) ,
											  X_m               (1) ,
											  X                 (1) ,
											  Xdd_m             (1) ,
											  transformed_force (1) ,
											  task_ref          (1) ,
											  task_refModel     (1)  );

			//      ROS_ERROR_STREAM("USING MRAC");
				}
			 */
			// MSD
			if( useMSDmodel )
			{
				// Cartesian space MSD model
				outerLoopMSDmodelX.update( Xd_m             (0) ,
	                                       Xd               (0) ,
	                                       X_m              (0) ,
	                                       X                (0) ,
	                                       Xdd_m            (0) ,
	                                       transformed_force(0)  );

				outerLoopMSDmodelY.update( Xd_m             (1) ,
	                                       Xd               (1) ,
	                                       X_m              (1) ,
	                                       X                (1) ,
	                                       Xdd_m            (1) ,
	                                       transformed_force(1)  );
				//      ROS_ERROR_STREAM("USING MSD");
			}

			// IRL
			if( useIRLmodel )
			{
				//    // Cartesian space IRL model
				//    outerLoopIRLmodelX.updateIRL( Xd_m              (0) ,
				//   	                            Xd                (0) ,
				//   	                            X_m               (0) ,
				//   	                            X                 (0) ,
				//   	                            Xdd_m             (0) ,
				//   	                            transformed_force (0) ,
				//   	                            task_ref          (0) ,
				//   	                            task_refModel     (0)  );

				outerLoopIRLmodelY.updateIRL( Xd_m                  (1) ,
	                                          Xd                    (1) ,
	                                          X_m                   (1) ,
	                                          X                     (1) ,
	                                          Xdd_m                 (1) ,
	                                          transformed_force     (1) ,
	                                          task_ref              (1) ,
	                                          task_refModel_output  (1)  );


				// IRL
				//			outerLoopIRLmodelX.getMsd( m_M,
				//			                           m_S,
				//			                           m_D );
				outerLoopIRLmodelY.getMsd( m_M,
	                                       m_S,
	                                       m_D );

				//      ROS_ERROR_STREAM("USING MSD");
			}

			// Direct Model
			if( useDirectmodel )
			{
				// q_d
				X_m(1)= task_refModel_output(1)      ;
				Xd_m  = (X_m - p_X_m)/outerLoopTime  ;
				Xdd_m = (Xd_m - p_Xd_m)/outerLoopTime;
			}

			// MRAC
			if( useMRACmodel )
			{
				//      outerLoopMRACmodelX.update( Xd_m              (0) ,
				//                                  Xd                (0) ,
				//                                  X_m               (0) ,
				//                                  X                 (0) ,
				//                                  Xdd_m             (0) ,
				//                                  transformed_force (0) ,
				//                                  task_ref          (0) ,
				//                                  task_refModel     (0)  );

				// Y axis
				outerLoopMRACmodelY.update( Xd_m                 (1) ,
	                                        Xd                   (1) ,
	                                        X_m                  (1) ,
	                                        X                    (1) ,
	                                        Xdd_m                (1) ,
	                                        transformed_force    (1) ,
	                                        task_ref             (1) ,
	                                        task_refModel_output (1)  );

				//      ROS_ERROR_STREAM("USING MRAC");
			}

			p_X_m    = X_m   ;
			p_Xd_m   = Xd_m  ;
			p_Xdd_m  = Xdd_m ;

			outer_elapsed_ = robot_state_->getTime() ;

		}

		// System Model END
		/////////////////////////
}
