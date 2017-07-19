/*
 * nn_estimator.cpp
 *
 *  Created on: Jun 22, 2017
 *      Author: Sven Cremer
 */

#ifndef NNESTIMATOR_H_
#define NNESTIMATOR_H_

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace csl {
namespace neural_network {

class NNEstimator
{

public:

	enum ActFcn {Sigmoid, RBF};

private:

	double updateRate;			// Controller update rate in seconds

	// Flags
	bool nn_ON;					// Use fhat from NN
	bool updateWeightsV;
	bool updateWeightsU;
	bool useLimits;				// Use artificial limits for Phat
	NNEstimator::ActFcn actF;	// Selected activation function

	// NN variables
	int bias;					// Use bias unit; 0 (none) or 1
	int num_Inputs  ; 			// Number of input nodes
	int num_Outputs ; 			// Number of output nodes
	int num_Hidden  ; 			// Size of the hidden layer
	int num_Dim		; 			// Size of input and output signals, i.e. number of Cartesian coordinates

	Eigen::MatrixXd V_;			// Human intent layer (estimates trajectory)
	Eigen::MatrixXd U_;			// Human gain layer (estimates gains)
	Eigen::MatrixXd V_next_;
	Eigen::MatrixXd U_next_;
	Eigen::MatrixXd V_trans;
	Eigen::MatrixXd U_trans;

	Eigen::VectorXd phi;		// NN input vector
	Eigen::VectorXd xhat;		// NN output vector (trajectory)
	Eigen::VectorXd Phat;		// NN output vector (gains)
	Eigen::VectorXd Phat_min;   // Artificial limits
	Eigen::VectorXd Phat_max;   // Artificial limits

	Eigen::VectorXd sigma;		// Activation function output
	Eigen::MatrixXd J;	        // Identity matrix  [I6, I6]

	// Controller variables
	Eigen::VectorXd e;			// Error signal
	Eigen::VectorXd ed;			// Error signal derivative
	Eigen::VectorXd ea_;		// Approximated human dynamics error signal
	Eigen::VectorXd ea_dot;
	Eigen::VectorXd s;			// Sliding mode error
	Eigen::MatrixXd Dh;			// Derivative term
	Eigen::MatrixXd Kh;		    // Proportional term
	Eigen::MatrixXd G;			// Positive definite design matrix for layer V
	Eigen::MatrixXd H;			// Positive definite design matrix for layer U
	double kappa;				// Gain of e-modification terms
	double alpha;				// Learning rate for testing

	Eigen::VectorXd fh;			// Human force

	// RBF variables
	double rbf_beta;			// RBF beta = 1/(2*sigma^2); fixed for each node instead of Eigen::VectorXd
	Eigen::MatrixXd rbf_mu;		// RBF center for each hidden layer node [num_Hidden x num_Hidden]

public:

	/*** Constructor methods ***/

	NNEstimator( int nDim,
                 NNEstimator::ActFcn a = NNEstimator::RBF)
	{

		actF = a;

		updateRate = 0.01; // 100 Hz

		nn_ON              = true;
		updateWeightsV     = true;
		updateWeightsU     = true;
		useLimits          = false;

		bias = 1;

		num_Dim     = nDim;		// Usually 6
		num_Inputs  = nDim*3;
		num_Outputs = nDim*2;
		num_Hidden  = num_Inputs;

		paramResize();

		// Set default parameter values
		Eigen::VectorXd p_G;
		Eigen::VectorXd p_H;
		p_G.setOnes(num_Hidden + bias);
		p_H.setOnes(num_Hidden + bias);
		p_G *= 0.1;
		p_H *= 0.1;

		paramInit(p_G, p_H, 0.01, 0.01);

		alpha = 1.0;

		// RBF
		rbf_beta = 1.0;
    	rbf_mu.resize(num_Hidden, num_Hidden);
    	rbf_mu.setRandom();								// Uniform dist between (-1,1)
    	Eigen::MatrixXd pos(num_Hidden,num_Hidden);
    	Eigen::MatrixXd neg(num_Hidden,num_Hidden);
    	pos.setOnes();
    	neg = -pos;
    	rbf_mu = (rbf_mu.array() < 0).select(neg,pos);	// Make elements -1 or +1
	}

	void paramResize()	// TODO make private
	{

		V_        .resize( num_Hidden + bias           , num_Outputs              ) ;
		U_        .resize( num_Hidden + bias           , num_Outputs              ) ;
		V_next_   .resize( num_Hidden + bias           , num_Outputs              ) ;
		U_next_   .resize( num_Hidden + bias           , num_Outputs              ) ;
		V_trans   .resize( num_Outputs                 , num_Hidden + bias        ) ;
		U_trans   .resize( num_Outputs                 , num_Hidden + bias        ) ;

		G         .resize( num_Hidden + bias           , num_Hidden + bias        ) ;
		H         .resize( num_Hidden + bias           , num_Hidden + bias        ) ;

		phi                 .resize( num_Inputs ) ;

		//activationOut       .resize( num_Inputs + bias) ;
		sigma               .resize( num_Inputs + bias  ) ;

		e                   .resize( num_Dim          ) ;
		ed                  .resize( num_Dim          ) ;
		ea_                 .resize( num_Dim          ) ;
		ea_dot              .resize( num_Dim          ) ;
		s                   .resize( num_Dim          ) ;
		fh                  .resize( num_Dim          ) ;

		Kh                  .resize( num_Dim, num_Dim ) ;
		Dh                  .resize( num_Dim, num_Dim ) ;

		Eigen::MatrixXd Iden;
		Iden.setIdentity(num_Dim,num_Dim);
		J.resize( num_Dim, num_Dim*2 );
		J.block(0,0,num_Dim,num_Dim)       = Iden;
		J.block(0,num_Dim,num_Dim,num_Dim) = Iden;

	}

	void resetWeights(double weightsLimit)
	{
		V_.setRandom();		// Range is [-1:1]
		U_.setRandom();

		V_ *= weightsLimit;
		U_ *= weightsLimit;

		V_next_ = V_;
		U_next_ = U_;
		V_trans = V_.transpose();
		U_trans = U_.transpose();
	}

	void paramInit( Eigen::VectorXd & p_G,
                    Eigen::VectorXd & p_H,
                    double p_kappa,
					double weightsLimit)
	{
		if(p_G.size() != num_Hidden + bias || p_H.size() != num_Hidden + bias)
		{
			std::cerr<<"Failed to initialize G or H!\n";
			p_G.setOnes(num_Hidden + bias);
			p_H.setOnes(num_Hidden + bias);
			p_G *= 10;
			p_H *= 10;
			//return;	// TODO
		}
		Eigen::MatrixXd p_G_mat = p_G.asDiagonal();
		Eigen::MatrixXd p_H_mat = p_H.asDiagonal();
		paramInit(p_G_mat, p_H_mat, p_kappa, weightsLimit);
	}

	void paramInit( Eigen::MatrixXd & p_G,
                    Eigen::MatrixXd & p_H,
                    double p_kappa,
					double weightsLimit)
	{
		G = p_G;
		H = p_H;

		kappa = p_kappa;

		// Initialize weights randomly
		resetWeights(weightsLimit);

		ea_.setZero();

		//std::cout<<"G:\n"<<G<<"\n---\n";
		//std::cout<<"H:\n"<<H<<"\n---\n";
	}

	/*** Get and set methods ***/

	int getNumInputs()					{	return num_Inputs;	}
	double getWeightsNormV()			{	return V_.norm();	}
	double getWeightsNormU()			{	return U_.norm();	}
	Eigen::MatrixXd	getWeightsV()		{	return V_;			}
	Eigen::MatrixXd	getWeightsU()		{	return U_;		}
	Eigen::VectorXd	getNNoutputU()		{	return Phat;		}
	Eigen::VectorXd	getNNoutputV()		{	return xhat;		}
	Eigen::VectorXd	getRBFmu()			{	return rbf_mu;		}
	Eigen::VectorXd	getG()				{	return G;			}
	Eigen::VectorXd	getH()				{	return H;			}
	Eigen::VectorXd	getKh()				{	return Kh.diagonal();	}
	Eigen::VectorXd	getDh()				{	return Dh.diagonal();	}
	Eigen::VectorXd	getS()				{	return s;			}
	Eigen::VectorXd	getEa()				{	return ea_;			}

	void setPhatMin(Eigen::VectorXd v)  {	Phat_min = v;		}	// TODO check values
	void setPhatMax(Eigen::VectorXd v)  {	Phat_max = v;		}
	void setUseLimits(bool v)  			{	useLimits = v;		}
	void setParamAlpha(double p)		{	alpha = p;			}
	void setParamKappa(double p)		{	kappa = p;			}
	void setParamG(double p)
	{
		G.setIdentity(num_Hidden + bias,num_Hidden + bias);
		G *= p;
	}
	void setParamH(double p)
	{
		H.setIdentity(num_Hidden + bias,num_Hidden + bias);
		H *= p;
	}
	bool setParamG(Eigen::VectorXd v)
	{
		if(v.rows() == num_Dim && v.cols() == 1 )
		{
			G = v.asDiagonal();
		}
		else if (v.rows() == num_Dim && v.cols() == num_Dim )
		{
			G = v;
		}
		else
		{
			std::cerr<<"Failed to initialize G!\n";
			return false;
		}
		return true;
	}
	bool setParamH(Eigen::VectorXd v)
	{
		// Initialize Lambda
		if(v.rows() == num_Dim && v.cols() == 1 )
		{
			H = v.asDiagonal();
		}
		else if (v.rows() == num_Dim && v.cols() == num_Dim )
		{
			H = v;
		}
		else
		{
			std::cerr<<"Failed to initialize H!\n";
			return false;
		}
		return true;
	}
	bool setParamG(double p, int index)
	{
		if(0<index && index<num_Dim)
		{
			G(index,index) = p;
			return true;
		}
		return false;
	}
	bool setParaH(double p, int index)
	{
		if(0<index && index<num_Dim)
		{
			H(index,index) = p;
			return true;
		}
		return false;
	}

	void setFlagNN(bool b)				{	nn_ON = b;			}
	void setUpdateRate(double r)		{	updateRate = r;		}
	void setRBFbeta(double b)			{	nn_ON = b;			}
	bool setWeightsV(Eigen::MatrixXd p_V)
	{
		if( p_V.rows() == (num_Inputs + bias) && p_V.cols() == num_Hidden )
		{
			V_next_ = p_V;
			return true;
		}
		return false;
	}
	bool setWeightsU(Eigen::MatrixXd p_W)
	{
		if( p_W.rows() == (num_Hidden + bias) && p_W.cols() == num_Outputs )
		{
			U_next_ = p_W;
			return true;
		}
		return false;
	}
	void setUpdateWeightsV(bool p_updateWeights)
	{
		updateWeightsV = p_updateWeights;
	}
	void setUpdateWeightsU(bool p_updateWeights)
	{
		updateWeightsU = p_updateWeights;
	}

	/*** Computational methods ***/

	Eigen::VectorXd activation( Eigen::VectorXd z);
	Eigen::MatrixXd activationPrime( Eigen::VectorXd z);

	// Estimates trajectory and and updates NN
	void Update( Eigen::VectorXd & x,
                 Eigen::VectorXd & xd,
                 Eigen::VectorXd & f_h,
                 double dt,
                 Eigen::VectorXd & x_hat,
                 Eigen::VectorXd & xd_hat);

	template<typename T>
	void printVar(std::string name, T var)
	{
		std::cout<<name<<"=\n"<<var<<"\n---\n";
	}
	void printSize(std::string name, Eigen::MatrixXd var)
	{
		std::cout<<name<<" is ["<<var.rows()<<"x"<<var.cols()<<"]\n";
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void NNEstimator::Update( Eigen::VectorXd & x,
                          Eigen::VectorXd & xd,
                          Eigen::VectorXd & f_h,
                          double dt,
                          Eigen::VectorXd & x_hat,
                          Eigen::VectorXd & xd_hat)
{
	U_ = U_next_;
	V_ = V_next_;

	U_trans = U_.transpose();
	V_trans = V_.transpose();

	// Update NN input vector
//	if(bias == 1)
//		phi << 1, x, xd, f_h;
//	else
		phi << x.head(num_Dim), xd.head(num_Dim), f_h.head(num_Dim);

	fh = f_h.head(num_Dim);

	// NN output
	sigma = activation(phi);
	Phat = U_trans*sigma;
	xhat = V_trans*sigma;

	if(useLimits)
	{
		Phat = (Phat.array() > Phat_max.array()).select(Phat_max, Phat);	// (Phat > Phat_max ? Phat_max : Phat)
		Phat = (Phat.array() < Phat_min.array()).select(Phat_min, Phat);
	}

	Kh = Phat.head(num_Dim).asDiagonal();
	Dh = Phat.tail(num_Dim).asDiagonal();
	x_hat.head(num_Dim)  = xhat.head(num_Dim);	// Note: this assumes x_hat has been initialized
	xd_hat.head(num_Dim) = xhat.tail(num_Dim);

	// Update error signals
	e  = x_hat.head(num_Dim) - x.head(num_Dim);
	ed = xd_hat.head(num_Dim) - xd.head(num_Dim);

	// Update filtered error
	ea_dot = Dh.inverse()*(fh - Kh*ea_);
	ea_ = ea_ + ea_dot*dt;

	// Sliding mode
	s = e - ea_;

	// NN update
	if(updateWeightsU) // Gains
	{
		// Uk+1 = Uk +  Ukdot * dt
		U_next_ = U_ - alpha*(H*sigma*s.transpose()*J*xhat.asDiagonal() + kappa*s.norm()*H*U_) * dt;	// TODO sign?
	}

	if(updateWeightsV) // Trajectory
	{
		// Vk+1 = Vk +  Vkdot * dt
		V_next_ = V_ - alpha*(G*sigma*s.transpose()*J*Phat.asDiagonal() + kappa*s.norm()*G*V_) * dt;
	}

}

Eigen::VectorXd NNEstimator::activation( Eigen::VectorXd z)
{
	if( z.size() != num_Hidden)
	{
		std::cerr<<"Activation input vector has the wrong dimensions!\n";
	}

	Eigen::VectorXd y;
	y.resize(num_Hidden+bias);

	switch(actF)
	{
	case NNEstimator::Sigmoid:
	{
		for(int i=0;i<num_Hidden;i++)
		{
			y(i) = 1.0/(1.0 + exp(-(double)z(i)));
		}
		break;
	}
	case NNEstimator::RBF:
	{
		Eigen::VectorXd d;
		double x;
		for(int i=0;i<num_Hidden;i++)
		{
			d = z - rbf_mu.col(i);
			x = d.transpose() * d;
			y(i) = exp( - rbf_beta * x );
		}
		break;
	}
	default:
		std::cerr<<"NNController: activation function not implemented!\n";
		break;
	}

	if(bias == 1)
	{
		y(num_Hidden) = 1;	// num_Hidden + bias - 1
	}

	return y;
}

Eigen::MatrixXd NNEstimator::activationPrime( Eigen::VectorXd z)
{
	if( z.size() != num_Hidden)
	{
		std::cerr<<"ActivationPrime input vector has the wrong dimensions!\n";
	}

	Eigen::MatrixXd y;
	y.setZero(num_Hidden+bias,num_Hidden);	// The last row will be zero if there is a bias unit.

	switch(actF)
	{
	case NNEstimator::Sigmoid:
	{
		Eigen::VectorXd s = activation(z);
		Eigen::MatrixXd S = s.asDiagonal();
		y.block(0,0,num_Hidden,num_Hidden) = S - S*S;
		break;
	}
	case NNEstimator::RBF:
	{
		Eigen::VectorXd d;
		double x;
		for(int i=0;i<num_Hidden;i++)
		{
			d = z - rbf_mu.col(i);
			x = d.transpose() * d;
			d = d.cwiseAbs();
			y.block(0,i,num_Hidden,1) =  -2*rbf_beta*d* exp( - rbf_beta * x );
		}
		break;
	}
	default:
		std::cerr<<"NNController: activationPrime function not implemented!\n";
		break;
	}

	return y;
}

} // end namespace neural_network
} // end namespace csl

#endif /* NNESTIMATOR_H_ */


