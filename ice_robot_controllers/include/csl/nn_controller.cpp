/*
 * nn_controller.cpp
 *
 *  Created on: Mar 23, 2017
 *      Author: Sven Cremer
 *
 *      Neuroadaptive controller based on Isura Ranatunga's TwoLayerNeuralNetworkController class.
 */

#ifndef NNCONTROLLER_H_
#define NNCONTROLLER_H_

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace csl {

namespace neural_network {

/*
 * This class will implement a Neural Network Based controller using a two layer NN with sigmoid activation functions
 * TODO add more description
 */
class NNController
{

public:

	enum ActFcn {Sigmoid, RBF};
	enum Layers {Single, Two};

private:

	NNController::ActFcn actF;

	int bias;		// Bias unit, 0 (none) or 1
	bool updateWeights;
	bool updateInnerWeights;

	int num_Inputs  ; // n Size of the inputs
	int num_Outputs ; // m Size of the outputs
	int num_Hidden  ; // l Size of the hidden layer
	int num_Error   ; // filtered error
	int num_Joints  ; // number of joints.
	int num_Dim		; // size of input and output signals, i.e. number of Cartesian coordinates

	Eigen::MatrixXd V_;			// Inner layer
	Eigen::MatrixXd W_;			// Outer layer
	Eigen::MatrixXd V_next_;
	Eigen::MatrixXd W_next_;
	Eigen::MatrixXd V_trans;
	Eigen::MatrixXd W_trans;

	Eigen::MatrixXd Z;	// Combined weight matrix

	Eigen::MatrixXd G;	// Positive definite design matrix for inner layer (V)
	Eigen::MatrixXd F;	// Positive definite design matrix for outer layer (W)

	Eigen::VectorXd x;	// NN input vector
	Eigen::VectorXd y;	// NN output vector

	Eigen::VectorXd sigma;		// Activation function output
	Eigen::MatrixXd sigmaPrime;	// Derivative of activation function
	Eigen::VectorXd hiddenLayerIn; // Input to the hidden layer

	Eigen::VectorXd sigmaPrimeTrans_W_r; // For temporary storing result

	Eigen::VectorXd r;	// Sliding mode error
	Eigen::VectorXd vRobust;

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

	/*** Constructor methods ***/

	NNController(int nJoints, int nDim, int nHidden, NNController::ActFcn a = NNController::Sigmoid)
	{

		actF = a;

		bias = 1;
		updateWeights = true;
		updateInnerWeights = true;

		num_Joints  = nJoints;
		num_Dim     = nDim;
		num_Hidden  = nHidden;
		num_Outputs = nDim;

		if(true) // Determine size of NN input vector
		{
			num_Inputs = num_Joints*2 + num_Dim*5;
		}

		initNN();

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

	void initNN()	// TODO make private
	{

		V_        .resize( num_Inputs + bias           , num_Hidden               ) ;
		W_        .resize( num_Hidden + bias           , num_Outputs              ) ;
		V_next_   .resize( num_Inputs + bias           , num_Hidden               ) ;
		W_next_   .resize( num_Hidden + bias           , num_Outputs              ) ;
		V_trans   .resize( num_Hidden                  , num_Inputs + bias        ) ;
		W_trans   .resize( num_Outputs                 , num_Hidden + bias        ) ;

		Z         .resize( num_Hidden + num_Inputs + 2*bias , num_Hidden + num_Outputs ) ;

		G         .resize( num_Inputs + bias           , num_Inputs + bias        ) ;
		F         .resize( num_Hidden + bias           , num_Hidden + bias        ) ;


		x                   .resize( num_Inputs + bias) ;
		y                   .resize( num_Outputs      ) ;

		hiddenLayerIn       .resize( num_Hidden       ) ;
		sigma               .resize( num_Hidden       ) ;
		sigmaPrime          .resize( num_Hidden, num_Hidden ) ;

		sigmaPrimeTrans_W_r .resize( num_Hidden     ) ;

		r                   .resize( num_Error      ) ;
		vRobust             .resize( num_Outputs    ) ;

		F.setIdentity();
		G.setIdentity();

		// TODO initialize weights randomly
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
			std::cerr<<"Error in NNController::init";
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
			std::cerr<<"Error in NNController::init";
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

	Eigen::VectorXd activation( Eigen::VectorXd z);
	Eigen::VectorXd activationPrime( Eigen::VectorXd z);

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

void NNController::UpdateCart( Eigen::VectorXd & X     ,
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

void NNController::UpdateJoint( Eigen::VectorXd & q     ,
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

void NNController::Update( Eigen::VectorXd & q    ,
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
	//r_tran = r.transpose();

	// Robust term
	Z.block(0,0,num_Hidden,num_Outputs) = W_;
	Z.block(num_Hidden,num_Outputs,num_Inputs+1,num_Hidden) = V_;
	vRobust = - Kz*(Z.norm() + Zb)*r;

	hiddenLayerIn = V_trans*x;
	sigma = activation(hiddenLayerIn);
	y = W_trans*sigma;

	// control torques
	tau = Kv*r + nn_ON*( y - vRobust ) - feedForwardForce*t_r ;
	//	tau = (qd_m - qd) + 100*(q_m - q);

	if(!updateWeights)
		return;

	//
	sigmaPrime = activationPrime(hiddenLayerIn);

	// Wk+1                  = Wk                  +  Wkdot                                                                                                          * dt
	W_next_ = W_ + (F*sigma*r.transpose() - F*sigmaPrime*V_trans*x*r.transpose() - kappa*F*r.norm()*W_) * delT;

	if(!updateInnerWeights)
		return;

	sigmaPrimeTrans_W_r = sigmaPrime.transpose()*W_*r;      // make sigmaPrimeTrans_W_r_tran = r_tran*sigmaPrime*W_trans

	// Vk+1                  = Vk                  +  Vkdot                                                                                      			 * dt
	V_next_ = V_ + (G*x*sigmaPrimeTrans_W_r.transpose() - kappa*G*r.norm()*V_) * delT;

}

Eigen::VectorXd NNController::activation( Eigen::VectorXd z)
{
	int N = z.size();

	Eigen::VectorXd y;
	y.resize(N);

	switch(actF)
	{
	case NNController::Sigmoid:
	{
		for(int i=0;i<N;i++)
		{
			y(i) = 1.0/(1.0 + exp(-(double)z(i)));
		}
		break;
	}
	case NNController::RBF:
//	{
//		break;
//	}
	default:
		std::cerr<<"NNController: activation function not implemented!\n";
		break;
	}

	return y;
}

Eigen::VectorXd NNController::activationPrime( Eigen::VectorXd z)
{
	int N = z.size();

	Eigen::VectorXd y;
	y.resize(N);

	switch(actF)
	{
	case NNController::Sigmoid:
	{
		Eigen::VectorXd s = activation(z);
		Eigen::MatrixXd S = s.asDiagonal();
		y = S - S*S;
		break;
	}
	case NNController::RBF:
//	{
//		break;
//	}
	default:
		std::cerr<<"NNController: activationPrime function not implemented!\n";
		break;
	}

	return y;
}

} // end namespace neural_network
} // end namespace csl

#endif /* NNCONTROLLER_H_ */
