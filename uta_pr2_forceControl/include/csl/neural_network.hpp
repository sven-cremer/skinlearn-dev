/**
 *
 * neural_network.hpp: ...
 *
 *
 * @author Isura Ranatunga, University of Texas at Arlington, Copyright (C) 2013.
 * @contact isura.ranatunga@mavs.uta.edu
 * @see ...
 * @created Jan 06, 2014
 * @modified Jan 06, 2014
 *
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace csl {

namespace neural_network {

/*
 * This class will implement a Neural Network Based controler using a two layer NN with sigmoid activation functions
 * TODO add more description
 */
class TwoLayerNeuralNetworkController {

	enum {
		Inputs = 35
	}; // n Size of the inputs
	enum {
		Outputs = 7
	}; // m Size of the outputs
	enum {
		Hidden = 10
	}; // l Size of the hidden layer
	enum {
		Error = 7
	}; // filtered error

	// Declare the number of joints.
	enum { Joints = 7 };

	// Define the joint/cart vector types accordingly (using a fixed
	// size to avoid dynamic allocations and make the code realtime safe).
	typedef Eigen::Matrix<double, Joints, Joints>  SystemMatrix;
	typedef Eigen::Matrix<double, Joints, 1>	   SystemVector;

	Eigen::Matrix<double, Hidden, Inputs + 1> V_trans;
	Eigen::Matrix<double, Outputs, Hidden> W_trans;
	Eigen::Matrix<double, Hidden, Inputs + 1> V_trans_next;
	Eigen::Matrix<double, Outputs, Hidden> W_trans_next;
	Eigen::Matrix<double, Inputs + 1, Inputs + 1> G;
	Eigen::Matrix<double, Hidden, Hidden> F;
	Eigen::Matrix<double, Outputs, Outputs> L;
	Eigen::Matrix<double, Hidden + Inputs + 1, Hidden + Outputs> Z;

	//  V_trans_next
	//  W_trans_next
	//  sigmaPrime

	Eigen::Matrix<double, Inputs + 1, 1> x;
	Eigen::Matrix<double, Outputs, 1> y;
	Eigen::Matrix<double, Hidden, 1> hiddenLayer_out;
	Eigen::Matrix<double, Hidden, Hidden> hiddenLayerIdentity;
	Eigen::Matrix<double, Hidden, 1> hiddenLayer_in;
	Eigen::Matrix<double, Outputs, 1> outputLayer_out;
	Eigen::Matrix<double, Hidden, Hidden> sigmaPrime;
	Eigen::Matrix<double, Error, 1> r;
	Eigen::Matrix<double, Outputs, 1> vRobust;

	double kappa;
	double Kv;
	double lambda;
	double Kz;
	double Zb;
	double nnF;
	double nnG;
	double nn_ON;

	double feedForwardForce;

	double delT; // Time step

public:

	TwoLayerNeuralNetworkController()
	{
		Init( 0.07 ,
                      10   ,
                      0.5  ,
                      0    ,
                      100  ,
                      1    ,
                      100  ,
                      20   ,
                      1     );

		delT = 0.001; /// 1000 Hz by default

	}

	void Init( double p_kappa  ,
		   double p_Kv     ,
		   double p_lambda ,
		   double p_Kz     ,
		   double p_Zb     ,
		   double p_ffForce,
		   double p_nnF    ,
		   double p_nnG    ,
		   double p_nn_ON   )
		{
			kappa            = p_kappa   ;
			Kv               = p_Kv      ; // prop. gain for PID inner loop
			lambda           = p_lambda  ; //*std::sqrt(Kp); // der. gain for PID inner loop
			Kz               = p_Kz      ;
			Zb               = p_Zb      ;
			feedForwardForce = p_ffForce ;
			nnF              = p_nnF     ;
			nnG              = p_nnG     ;
			nn_ON		 = p_nn_ON   ;

			hiddenLayerIdentity.setIdentity();

			W_trans.setZero();
			W_trans_next.setZero();
			V_trans.setZero();
			V_trans_next.setZero();

			F.setIdentity();
			G.setIdentity();
			L.setIdentity();

			// Very important
			Z.setZero();

			F = nnF*F;
			G = nnG*G;
		}

	void Update( SystemVector & qd_m  ,
		     SystemVector & qd    ,
		     SystemVector & q_m   ,
		     SystemVector & q     ,
		     SystemVector & qdd_m ,
		     SystemVector & t_r   ,
		     SystemVector & tau    );

	void UpdateDelT( double p_delT );

	Eigen::Matrix<double, TwoLayerNeuralNetworkController::Hidden, 1>
	sigmoid( Eigen::Matrix<double, TwoLayerNeuralNetworkController::Hidden, 1> & z );

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

}
