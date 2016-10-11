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

#ifndef TWOLAYERNEURALNETWORKCONTROLLER_H_
#define TWOLAYERNEURALNETWORKCONTROLLER_H_

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace csl {

namespace neural_network {

/*
 * This class will implement a Neural Network Based controller using a two layer NN with sigmoid activation functions
 * TODO add more description
 */
class TwoLayerNeuralNetworkController
{
private:

	bool updateWeights;
	bool updateInnerWeights;

	int num_Inputs  ; // n Size of the inputs
	int num_Outputs ; // m Size of the outputs
	int num_Hidden  ; // l Size of the hidden layer
	int num_Error   ; // filtered error
	int num_Joints  ; // number of joints.

	Eigen::MatrixXd V_;
	Eigen::MatrixXd W_;
	Eigen::MatrixXd V_next_;
	Eigen::MatrixXd W_next_;

	Eigen::MatrixXd V_trans;
	Eigen::MatrixXd W_trans;
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
	Eigen::MatrixXd r_tran;
	Eigen::MatrixXd vRobust;
	Eigen::MatrixXd sigmaPrimeTrans_W_r;

	double kappa;
	Eigen::MatrixXd Kv;
	Eigen::MatrixXd lambda;
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
		updateWeights = true;
		updateInnerWeights = true;

          changeNNstructure( 35 ,   // num_Inputs
                             7  ,   // num_Outputs
                             10 ,   // num_Hidden
                             7  ,   // num_Error
                             7   ); // num_Joints

		delT = 0.001; /// 1000 Hz by default

		Eigen::MatrixXd p_Kv     ;
		Eigen::MatrixXd p_lambda ;

		p_Kv                  .resize( 7, 1 ) ;
		p_lambda              .resize( 7, 1 ) ;

          p_Kv << 10 ,
                  10 ,
                  10 ,
                  10 ,
                  10 ,
                  10 ,
                  10 ;

          p_lambda << 0.5 ,
                      0.5 ,
                      0.5 ,
                      0.5 ,
                      0.5 ,
                      0.5 ,
                      0.5 ;

          init( 0.07     ,
                p_Kv     ,
                p_lambda ,
                0        ,
                100      ,
                1        ,
                100      ,
                20       ,
                1         );
}

	void changeNNstructure( int para_num_Inputs  ,
			int para_num_Outputs ,
			int para_num_Hidden  ,
			int para_num_Error   ,
			int para_num_Joints   )
	{
		num_Inputs  = para_num_Inputs  ;
		num_Outputs = para_num_Outputs ;
		num_Hidden  = para_num_Hidden  ;
		num_Error   = para_num_Error   ;
		num_Joints  = para_num_Joints  ;

		V_        .resize( num_Inputs + 1              , num_Hidden               ) ;
		W_        .resize( num_Hidden                  , num_Outputs              ) ;
		V_next_   .resize( num_Inputs + 1              , num_Hidden               ) ;
		W_next_   .resize( num_Hidden                  , num_Outputs              ) ;

		V_trans   .resize( num_Hidden                  , num_Inputs + 1           ) ;
		W_trans   .resize( num_Outputs                 , num_Hidden               ) ;
		G         .resize( num_Inputs + 1              , num_Inputs + 1           ) ;
		F         .resize( num_Hidden                  , num_Hidden               ) ;
		L         .resize( num_Outputs                 , num_Outputs              ) ;
		Z         .resize( num_Hidden + num_Inputs + 1 , num_Hidden + num_Outputs ) ;

		x                   .resize( num_Inputs + 1, 1          ) ;
		y                   .resize( num_Outputs   , 1          ) ;
		hiddenLayer_out     .resize( num_Hidden    , 1          ) ;
		hiddenLayerIdentity .resize( num_Hidden    , num_Hidden ) ;
		hiddenLayer_in      .resize( num_Hidden    , 1          ) ;
		outputLayer_out     .resize( num_Outputs   , 1          ) ;
		sigmaPrime          .resize( num_Hidden    , num_Hidden ) ;
		r                   .resize( num_Error     , 1          ) ;
		r_tran              .resize( 1             , num_Error  ) ;
		vRobust             .resize( num_Outputs   , 1          ) ;
		sigmaPrimeTrans_W_r .resize( num_Hidden    , 1          ) ;

		hiddenLayerIdentity.setIdentity();
		F.setIdentity();
		G.setIdentity();
		L.setIdentity();

		W_.setZero();
		W_next_.setZero();
		V_.setZero();
		V_next_.setZero();

		// Very important
		Z.setZero();

	}

	void init( double p_kappa             ,
	           Eigen::MatrixXd & p_Kv     ,
	           Eigen::MatrixXd & p_lambda ,
		   double p_Kz                ,
		   double p_Zb                ,
		   double p_ffForce           ,
		   double p_nnF               ,
		   double p_nnG               ,
		   double p_nn_ON              )
	{
		// Init Kv
		if(p_Kv.rows() == num_Error && p_Kv.cols() == 1 )
		{
			Kv = p_Kv.asDiagonal();
		}
		else if (p_Kv.rows() == num_Error && p_Kv.cols() == num_Error )
		{
			Kv = p_Kv;
		}
		else
		{
			std::cerr<<"Error in TwoLayerNeuralNetworkController::init";
			Kv.setZero();
		}

		// Init Lambda
		if(p_lambda.rows() == num_Error && p_lambda.cols() == 1 )
		{
			lambda = p_lambda.asDiagonal();
		}
		else if (p_lambda.rows() == num_Error && p_lambda.cols() == num_Error )
		{
			lambda = p_lambda;
		}
		else
		{
			std::cerr<<"Error in TwoLayerNeuralNetworkController::init";
			lambda.setZero();
		}

		std::cout<<"Kv:\n"<<Kv<<"\n---\n";
		std::cout<<"Kv*lambda:\n"<<Kv*lambda<<"\n---\n";

		kappa            = p_kappa   ;
		Kz               = p_Kz      ;
		Zb               = p_Zb      ;
		feedForwardForce = p_ffForce ;
		nnF              = p_nnF     ;
		nnG              = p_nnG     ;
		nn_ON            = p_nn_ON   ;

		F = nnF*F;
		G = nnG*G;

	}

	void updateDelT( double p_delT )
	{
		delT = p_delT;
	}

	inline void UpdateCart( Eigen::VectorXd & X     ,
	                 Eigen::VectorXd & Xd    ,
	                 Eigen::VectorXd & X_m   ,
	                 Eigen::VectorXd & Xd_m  ,
	                 Eigen::VectorXd & Xdd_m ,
	                 Eigen::VectorXd & q     ,
	                 Eigen::VectorXd & qd    ,
	                 Eigen::VectorXd & t_r   ,
	                 Eigen::VectorXd & tau    );

	inline void UpdateJoint( Eigen::VectorXd & q     ,
                          Eigen::VectorXd & qd    ,
                          Eigen::VectorXd & q_m   ,
                          Eigen::VectorXd & qd_m  ,
                          Eigen::VectorXd & qdd_m ,
                          Eigen::VectorXd & t_r   ,
                          Eigen::VectorXd & tau    );

	inline void Update( Eigen::VectorXd & q    ,
                     Eigen::VectorXd & qd   ,
                     Eigen::VectorXd & q_m  ,
                     Eigen::VectorXd & qd_m ,
                     Eigen::VectorXd & t_r  ,
                     Eigen::VectorXd & tau   );

	inline Eigen::MatrixXd
	sigmoid( Eigen::MatrixXd & z ) const;

	double getInnerWeightsNorm()
	{
		return V_.norm();
	}
	double getOuterWeightsNorm()
	{
		return W_.norm();
	}
	Eigen::MatrixXd	getInnerWeights()   // TODO return V_ instead
	{
		return V_trans;
	}
	Eigen::MatrixXd	getOuterWeights()   // TODO return W_ instead
	{
		return W_trans;
	}
	void setInnerWeights(Eigen::MatrixXd V_trans_)	// TODO check size
	{
		V_next_ = V_trans_.transpose();
		V_ = V_trans_.transpose();
	}
	void setOuterWeights(Eigen::MatrixXd W_trans_)
	{
		W_next_ = W_trans_.transpose();
		W_ = W_trans_.transpose();
	}
	void setUpdateWeights(bool p_updateWeights)
	{
		updateWeights = p_updateWeights;
	}
	void setUpdateInnerWeights(bool p_updateInnerWeights)
	{
		updateInnerWeights = p_updateInnerWeights;
	}


	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void TwoLayerNeuralNetworkController::UpdateCart( Eigen::VectorXd & X     ,
                                                  Eigen::VectorXd & Xd    ,
                                                  Eigen::VectorXd & X_m   ,
                                                  Eigen::VectorXd & Xd_m  ,
                                                  Eigen::VectorXd & Xdd_m ,
                                                  Eigen::VectorXd & q     ,
                                                  Eigen::VectorXd & qd    ,
                                                  Eigen::VectorXd & t_r   ,
                                                  Eigen::VectorXd & tau    )
{
        // NN Input Vector
        x <<           1 ,
            (  X_m -  X) , //   q( 0 ) ;
            ( Xd_m - Xd) , //  qd( 0 ) ;
                    X_m  ,
                   Xd_m  ,
                  Xdd_m  ,
                      q  ,
                     qd  ;

        // x is global so not passed
        Update( X    ,
                Xd   ,
                X_m  ,
                Xd_m ,
                t_r  ,
                tau   );
}

void TwoLayerNeuralNetworkController::UpdateJoint( Eigen::VectorXd & q     ,
					           Eigen::VectorXd & qd    ,
					           Eigen::VectorXd & q_m   ,
                                                   Eigen::VectorXd & qd_m  ,
					           Eigen::VectorXd & qdd_m ,
					           Eigen::VectorXd & t_r   ,
					           Eigen::VectorXd & tau    )
{
	// NN Input Vector
        x <<           1 ,
            (  q_m -  q) , //   q( 0 ) ;
            ( qd_m - qd) , //  qd( 0 ) ;
                    q_m  ,
                   qd_m  ,
                  qdd_m  ;

        Update( q    ,
                qd   ,
                q_m  ,
                qd_m ,
                t_r  ,
                tau   );
}

void TwoLayerNeuralNetworkController::Update( Eigen::VectorXd & q    ,
                                              Eigen::VectorXd & qd   ,
                                              Eigen::VectorXd & q_m  ,
                                              Eigen::VectorXd & qd_m ,
                                              Eigen::VectorXd & t_r  ,
                                              Eigen::VectorXd & tau   )
{
	W_ = W_next_;
	V_ = V_next_;

	W_trans = W_.transpose();
	V_trans = V_.transpose();

	// Filtered error
	r = (qd_m - qd) + lambda*(q_m - q);
	r_tran = r.transpose();

	// Robust term
	Z.block(0,0,num_Hidden,num_Outputs) = W_;
	Z.block(num_Hidden,num_Outputs,num_Inputs+1,num_Hidden) = V_;
	vRobust = - Kz*(Z.norm() + Zb)*r;

	hiddenLayer_in = V_trans*x;
	hiddenLayer_out = sigmoid(hiddenLayer_in);
	outputLayer_out = W_trans*hiddenLayer_out;

	y = outputLayer_out;

	// control torques
	tau = Kv*r + nn_ON*( y - vRobust ) - feedForwardForce*t_r ;
	//	tau = (qd_m - qd) + 100*(q_m - q);

	if(!updateWeights)
		return;

	//
	sigmaPrime = hiddenLayer_out.asDiagonal()*( hiddenLayerIdentity - hiddenLayerIdentity*hiddenLayer_out.asDiagonal() );

	// Wk+1                  = Wk                  +  Wkdot                                                                                                          * dt
	W_next_ = W_ + (F*hiddenLayer_out*r_tran - F*sigmaPrime*V_trans*x*r_tran - kappa*F*r.norm()*W_) * delT;

	if(!updateInnerWeights)
		return;

	sigmaPrimeTrans_W_r = sigmaPrime.transpose()*W_*r;      // make sigmaPrimeTrans_W_r_tran = r_tran*sigmaPrime*W_trans

	// Vk+1                  = Vk                  +  Vkdot                                                                                      			 * dt
	V_next_ = V_ + (G*x*sigmaPrimeTrans_W_r.transpose() - kappa*G*r.norm()*V_) * delT;

}

Eigen::MatrixXd
TwoLayerNeuralNetworkController::sigmoid( Eigen::MatrixXd & z ) const
{
	// FIXME improve this
	for(uint i=0;i<z.size();i++)
	{
		z(i) = 1.0/(1.0 + exp(-(double)z(i)));
	}
	return z;
}

}

}

#endif /* TWOLAYERNEURALNETWORKCONTROLLER_H_ */
