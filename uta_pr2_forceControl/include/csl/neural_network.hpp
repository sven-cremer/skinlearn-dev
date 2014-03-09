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

        double num_Inputs  ; // n Size of the inputs
        double num_Outputs ; // m Size of the outputs
        double num_Hidden  ; // l Size of the hidden layer
        double num_Error   ; // filtered error
        double num_Joints  ; // number of joints.

        Eigen::MatrixXd V_trans;
        Eigen::MatrixXd W_trans;
        Eigen::MatrixXd V_trans_next;
        Eigen::MatrixXd W_trans_next;
        Eigen::MatrixXd G;
        Eigen::MatrixXd F;
        Eigen::MatrixXd L;
        Eigen::MatrixXd Z;

        Eigen::MatrixXd x;
        Eigen::MatrixXd y;
        Eigen::MatrixXd hiddenLayer_out;
        Eigen::MatrixXd hiddenLayerIdentity;
        Eigen::MatrixXd hiddenLayer_in;
        Eigen::MatrixXd outputLayer_out;
        Eigen::MatrixXd sigmaPrime;
        Eigen::MatrixXd r;
        Eigen::MatrixXd vRobust;
        Eigen::MatrixXd sigmaPrimeTrans_W_r;

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
          changeNNstructure( 35 ,   // num_Inputs
                             7  ,   // num_Outputs
                             10 ,   // num_Hidden
                             7  ,   // num_Error
                             7   ); // num_Joints

          delT = 0.001; /// 1000 Hz by default

          init( 0.07 ,
                10   ,
                0.5  ,
                0    ,
                100  ,
                1    ,
                100  ,
                20   ,
                1     );
	}

	void changeNNstructure( double para_num_Inputs  ,
	                        double para_num_Outputs ,
	                        double para_num_Hidden  ,
	                        double para_num_Error   ,
	                        double para_num_Joints   )
	{
	  num_Inputs  = para_num_Inputs  ;
	  num_Outputs = para_num_Outputs ;
	  num_Hidden  = para_num_Hidden  ;
	  num_Error   = para_num_Error   ;
	  num_Joints  = para_num_Joints  ;
	}

	void init( double p_kappa  ,
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
			Kv               = p_Kv      ;
			lambda           = p_lambda  ;
			Kz               = p_Kz      ;
			Zb               = p_Zb      ;
			feedForwardForce = p_ffForce ;
			nnF              = p_nnF     ;
			nnG              = p_nnG     ;
			nn_ON		 = p_nn_ON   ;

			V_trans       .resize( num_Hidden                  , num_Inputs + 1           ) ;
			W_trans       .resize( num_Outputs                 , num_Hidden               ) ;
			V_trans_next  .resize( num_Hidden                  , num_Inputs + 1           ) ;
			W_trans_next  .resize( num_Outputs                 , num_Hidden               ) ;
			G             .resize( num_Inputs + 1              , num_Inputs + 1           ) ;
			F             .resize( num_Hidden                  , num_Hidden               ) ;
			L             .resize( num_Outputs                 , num_Outputs              ) ;
			Z             .resize( num_Hidden + num_Inputs + 1 , num_Hidden + num_Outputs ) ;

			x                   .resize( num_Inputs + 1, 1              ) ;
			y                   .resize( num_Outputs   , 1              ) ;
			hiddenLayer_out     .resize( num_Hidden    , 1              ) ;
			hiddenLayerIdentity .resize( num_Hidden    , num_Hidden     ) ;
			hiddenLayer_in      .resize( num_Hidden    , 1              ) ;
			outputLayer_out     .resize( num_Outputs   , 1              ) ;
			sigmaPrime          .resize( num_Hidden    , num_Hidden     ) ;
			r                   .resize( num_Error     , 1              ) ;
			vRobust             .resize( num_Outputs   , 1              ) ;
			sigmaPrimeTrans_W_r .resize( num_Hidden    , 1              ) ;


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

	void updateDelT( double p_delT )
	{
	  delT = p_delT;
	}

	void UpdateCart( Eigen::VectorXd & X     ,
	                 Eigen::VectorXd & Xd    ,
	                 Eigen::VectorXd & X_m   ,
	                 Eigen::VectorXd & Xd_m  ,
	                 Eigen::VectorXd & Xdd_m ,
	                 Eigen::VectorXd & q     ,
	                 Eigen::VectorXd & qd    ,
	                 Eigen::VectorXd & t_r   ,
	                 Eigen::VectorXd & tau    );

        void UpdateJoint( Eigen::VectorXd & q     ,
                          Eigen::VectorXd & qd    ,
                          Eigen::VectorXd & q_m   ,
                          Eigen::VectorXd & qd_m  ,
                          Eigen::VectorXd & qdd_m ,
                          Eigen::VectorXd & t_r   ,
                          Eigen::VectorXd & tau    );

	void Update( Eigen::VectorXd & q    ,
                     Eigen::VectorXd & qd   ,
                     Eigen::VectorXd & q_m  ,
                     Eigen::VectorXd & qd_m ,
                     Eigen::VectorXd & t_r  ,
                     Eigen::VectorXd & tau   );

	Eigen::MatrixXd
	sigmoid( Eigen::MatrixXd & z ) const;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

}
