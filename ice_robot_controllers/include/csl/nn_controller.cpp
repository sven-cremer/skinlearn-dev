/*
 * nn_controller.cpp
 *
 *  Created on: Mar 23, 2017
 *      Author: Sven Cremer
 *
 *      Neuroadaptive controller based on Isura Ranatunga's TwoLayerNeuralNetworkController class.
 *      This class implements both one and two layer NNs with different activation functions.
 */

#ifndef NNCONTROLLER_H_
#define NNCONTROLLER_H_

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace csl {
namespace neural_network {

class NNController
{

public:

	enum ActFcn {Sigmoid, RBF};
	enum Layers {One, Two};		// TODO implement One layer

private:

	double updateRate;			// Controller update rate in seconds

	// Flags
	bool nn_ON;					// Use fhat from NN
	bool robust_ON;				// Use robustifying term
	bool PED_ON;				// Use prescribed error dynamics
	bool updateWeights;
	bool updateInnerWeights;
	NNController::ActFcn actF;	// Selected activation function
	NNController::Layers lay;	// Number of NN layers

	// NN variables
	int bias;					// Use bias unit; 0 (none) or 1
	int num_Inputs  ; 			// Number of input nodes
	int num_Outputs ; 			// Number of output nodes
	int num_Hidden  ; 			// Size of the hidden layer
	int num_Joints  ; 			// Number of joints
	int num_Dim		; 			// Size of input and output signals, i.e. number of Cartesian coordinates

	Eigen::MatrixXd V_;			// Inner layer
	Eigen::MatrixXd W_;			// Outer layer
	Eigen::MatrixXd V_next_;
	Eigen::MatrixXd W_next_;
	Eigen::MatrixXd V_trans;
	Eigen::MatrixXd W_trans;

	Eigen::MatrixXd Z;			// Combined weight matrix

	Eigen::VectorXd phi;		// NN input vector
	Eigen::VectorXd fhat;		// NN output vector

	Eigen::VectorXd sigma;		// Activation function output
	Eigen::MatrixXd sigmaPrime;	// Derivative of activation function
	Eigen::VectorXd hiddenLayerIn; // Input to the hidden layer

	Eigen::VectorXd sigmaPrimeTrans_W_r; // For temporary storing result

	// Controller variables
	Eigen::VectorXd e;			// Error signal
	Eigen::VectorXd ed;			// Error signal derivative
	Eigen::VectorXd r;			// Sliding mode error
	Eigen::VectorXd v;	        // Robustifying term
	Eigen::MatrixXd Kv;			// "Derivative" term, i.e. Kv*r= Kv*ed + Kv*lam*e
	Eigen::MatrixXd La;		    // "Proportional" term
	Eigen::MatrixXd G;			// Positive definite design matrix for inner layer (V)
	Eigen::MatrixXd F;			// Positive definite design matrix for outer layer (W)
	double kappa;				// Gain of e-modification terms
	double Kz;					// Gain of robustifying term
	double Zb;					// Bound for NN weight error part of robustifying term

	// Prescribed error dynamics
	Eigen::MatrixXd Kd;			// Desired stiffness
	Eigen::MatrixXd Dd;			// Desired damping
	Eigen::MatrixXd Ga;			// Gamma
	//Eigen::MatrixXd lambda;
	Eigen::VectorXd fl;			// Filtered force
	Eigen::VectorXd fh;			// Human force
	Eigen::MatrixXd La_dot;
	Eigen::VectorXd fl_dot;

	// RBF variables
	double rbf_beta;			// RBF beta = 1/(2*sigma^2); fixed for each node instead of Eigen::VectorXd
	Eigen::MatrixXd rbf_mu;		// RBF center for each hidden layer node [num_Hidden x num_Hidden]

public:

	/*** Constructor methods ***/

	NNController(int nJoints, int nDim, int nHidden,
			     bool PED = false,
			     NNController::ActFcn a = NNController::RBF,
				 NNController::Layers l = NNController::Two)
	{

		actF = a;
		lay  = l;

		updateRate = 0.001; // 1000 Hz

		PED_ON             = PED;
		nn_ON              = true;
		robust_ON          = true;
		updateWeights      = true;
		updateInnerWeights = true;

		bias = 1;

		num_Joints  = nJoints;
		num_Dim     = nDim;
		num_Hidden  = nHidden;
		num_Outputs = nDim;

		if(PED_ON) // Determine size of NN input vector (Joint: 5*num_Joints)
		{
			num_Inputs = num_Joints*2 + num_Dim*9;
		}
		else
		{
			num_Inputs = num_Joints*2 + num_Dim*5;
		}
		std::cout<<"Number of joints:  "<<num_Joints<<"\n";
		std::cout<<"Number of outputs: "<<num_Outputs<<"\n";
		std::cout<<"Number of inputs:  "<<num_Inputs<<"\n";

		paramResize();

		// Set default parameter values
		Eigen::VectorXd p_Kv;
		Eigen::VectorXd p_La;

		p_Kv.setOnes(num_Dim);
		p_La.setOnes(num_Dim);

		p_Kv *= 2;
		p_La *= 20;

		paramInit(p_Kv, p_La, 0.01, 0.05, 100, 100, 10, 0.01);

		// Prescribed error dynamics
		Kd .setIdentity(num_Dim,num_Dim);
		Dd .setIdentity(num_Dim,num_Dim);
		Ga .setIdentity(num_Dim,num_Dim);
		La_dot.setZero(num_Dim,num_Dim);
		fl_dot.setZero(num_Dim);
		fl.setZero(num_Dim);
		fh.setZero(num_Dim);
		Kd *= 20.0;				// TODO use parameters
		Dd *= 10.0;
		Ga  *= 9.5;
		if(PED_ON)
		{
			La.setIdentity(num_Dim,num_Dim);
			La *= 0.5;
		}

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

		V_        .resize( num_Inputs + bias           , num_Hidden               ) ;
		W_        .resize( num_Hidden + bias           , num_Outputs              ) ;
		V_next_   .resize( num_Inputs + bias           , num_Hidden               ) ;
		W_next_   .resize( num_Hidden + bias           , num_Outputs              ) ;
		V_trans   .resize( num_Hidden                  , num_Inputs + bias        ) ;
		W_trans   .resize( num_Outputs                 , num_Hidden + bias        ) ;

		Z         .resize( num_Hidden + num_Inputs + 2*bias , num_Hidden + num_Outputs ) ;

		G         .resize( num_Inputs + bias           , num_Inputs + bias        ) ;
		F         .resize( num_Hidden + bias           , num_Hidden + bias        ) ;

		phi                 .resize( num_Inputs + bias) ;
		fhat                .resize( num_Outputs      ) ;

		hiddenLayerIn       .resize( num_Hidden       ) ;
		sigma               .resize( num_Hidden       ) ;
		sigmaPrime          .resize( num_Hidden, num_Hidden ) ;

		sigmaPrimeTrans_W_r .resize( num_Hidden       ) ;

		e                   .resize( num_Dim          ) ;
		ed                  .resize( num_Dim          ) ;
		r                   .resize( num_Dim          ) ;
		v                   .resize( num_Dim          ) ;

		Kv                  .resize( num_Dim, num_Dim ) ;
		La                  .resize( num_Dim, num_Dim ) ;

	}

	void resetWeights(double weightsLimit)
	{
		W_.setRandom();		// Range is [-1:1]
		V_.setRandom();

		W_ *= weightsLimit;
		V_ *= weightsLimit;

		W_next_ = W_;
		V_next_ = V_;
		W_trans = W_.transpose();
		V_trans = V_.transpose();
	}

	void paramInit( Eigen::VectorXd & p_Kv,
                    Eigen::VectorXd & p_La,
                    double p_kappa,
                    double p_Kz,
                    double p_Zb,
					double p_G,
					double p_F,
					double weightsLimit)
	{

		Eigen::MatrixXd p_Kv_mat = p_Kv;
		Eigen::MatrixXd p_La_mat = p_La;
		paramInit(p_Kv_mat, p_La_mat, p_kappa, p_Kz, p_Zb, p_G, p_F, weightsLimit);

	}

	void paramInit( Eigen::MatrixXd & p_Kv,
                    Eigen::MatrixXd & p_La,
                    double p_kappa,
                    double p_Kz,
                    double p_Zb,
					double p_G,
					double p_F,
					double weightsLimit)
	{
		// Initialize Kv
		if( !setParamKv(p_Kv) )
		{
			Kv.setZero();
		}
		// Initialize Lambda
		if( !setParamLa(p_La) )
		{
			La.setZero();
		}

		kappa = p_kappa;
		Kz    = p_Kz;
		Zb    = p_Zb;

		G.setIdentity();
		F.setIdentity();
		G *= p_G;
		F *= p_F;

		// Initialize weights randomly
		resetWeights(weightsLimit);

		Z.setZero();	// Important since the update function only modifies a part of the matrix
		v.setZero();

		//std::cout<<"Kv:\n"<<Kv<<"\n---\n";
		//std::cout<<"Kv*lambda:\n"<<Kv*La<<"\n---\n";
	}

	/*** Get and set methods ***/

	int getNumInputs()					{	return num_Inputs;	}
	double getInnerWeightsNorm()		{	return V_.norm();	}
	double getOuterWeightsNorm()		{	return W_.norm();	}
	Eigen::MatrixXd	getInnerWeights()	{	return V_;			}
	Eigen::MatrixXd	getOuterWeights()	{	return W_trans;		}
	Eigen::VectorXd	getNNoutput()		{	return fhat;		}
	Eigen::VectorXd	getRBFmu()			{	return rbf_mu;		}
	Eigen::VectorXd	getKv()				{	return Kv.diagonal();}
	Eigen::VectorXd	getLa()				{	return La.diagonal();}
	Eigen::VectorXd	getGa()				{	return Ga.diagonal();}
	Eigen::VectorXd	getKd()				{	return Kd.diagonal();}
	Eigen::VectorXd	getDd()				{	return Dd.diagonal();}

	void setParamKz(double p)			{	Kz = p;				}
	void setParamZb(double p)			{	Zb = p;				}
	void setParamKappa(double p)		{	kappa = p;			}
	void setParamF(double p)
	{
		F.setIdentity();
		F *= p;
	}
	void setParamG(double p)
	{
		G.setIdentity();
		G *= p;
	}
	bool setParamKd(Eigen::MatrixXd p)	// TODO make a function
	{
		if(p.rows() == num_Dim && p.cols() == 1 )
		{
			Kd = p.asDiagonal();
		}
		else if (p.rows() == num_Dim && p.cols() == num_Dim )
		{
			Kd = p;
		}
		else
		{
			std::cerr<<"Failed to initialize Kd!\n";
			return false;
		}
		return true;
	}
	bool setParamDd(Eigen::MatrixXd p)
	{
		if(p.rows() == num_Dim && p.cols() == 1 )
		{
			Dd = p.asDiagonal();
		}
		else if (p.rows() == num_Dim && p.cols() == num_Dim )
		{
			Dd = p;
		}
		else
		{
			std::cerr<<"Failed to initialize Kd!\n";
			return false;
		}
		return true;
	}
	bool setParamKv(Eigen::VectorXd p_Kv)
	{
		if(p_Kv.rows() == num_Dim && p_Kv.cols() == 1 )
		{
			Kv = p_Kv.asDiagonal();
		}
		else if (p_Kv.rows() == num_Dim && p_Kv.cols() == num_Dim )
		{
			Kv = p_Kv;
		}
		else
		{
			std::cerr<<"Failed to initialize Kv!\n";
			return false;
		}
		return true;
	}
	bool setParamLa(Eigen::VectorXd p_La)
	{
		// Initialize Lambda
		if(p_La.rows() == num_Dim && p_La.cols() == 1 )
		{
			La = p_La.asDiagonal();
		}
		else if (p_La.rows() == num_Dim && p_La.cols() == num_Dim )
		{
			La = p_La;
		}
		else
		{
			std::cerr<<"Failed to initialize Lambda!\n";
			return false;
		}
		return true;
	}
	bool setParamKv(double p, int index)
	{
		if(0<index && index<num_Dim)
		{
			Kv(index,index) = p;
			return true;
		}
		return false;
	}
	bool setParamLa(double p, int index)
	{
		if(0<index && index<num_Dim)
		{
			La(index,index) = p;
			return true;
		}
		return false;
	}
	bool setParamKd(double p, int index)
	{
		if(0<index && index<num_Dim)
		{
			Kd(index,index) = p;
			return true;
		}
		return false;
	}
	bool setParamDd(double p, int index)
	{
		if(0<index && index<num_Dim)
		{
			Dd(index,index) = p;
			return true;
		}
		return false;
	}

	void setFlagNN(bool b)				{	nn_ON = b;			}
	void setFlagRobust(bool b)			{	robust_ON = b;		}
	void setFlagPED(bool b)				{	PED_ON = b;			}
	void setUpdateRate(double r)		{	updateRate = r;		}
	void setRBFbeta(double b)			{	nn_ON = b;			}
	bool setInnerWeights(Eigen::MatrixXd p_V)
	{
		if( p_V.rows() == (num_Inputs + bias) && p_V.cols() == num_Hidden )
		{
			V_next_ = p_V;
			return true;
		}
		return false;
	}
	bool setOuterWeights(Eigen::MatrixXd p_W)
	{
		if( p_W.rows() == (num_Hidden + bias) && p_W.cols() == num_Outputs )
		{
			W_next_ = p_W;
			return true;
		}
		return false;
	}
	void setUpdateWeights(bool p_updateWeights)
	{
		updateWeights = p_updateWeights;
	}
	void setUpdateInnerWeights(bool p_updateInnerWeights)
	{
		updateInnerWeights = p_updateInnerWeights;
	}

	/*** Computational methods ***/

	Eigen::VectorXd activation( Eigen::VectorXd z);
	Eigen::MatrixXd activationPrime( Eigen::VectorXd z);

	// Computes fc = -Kv*r + fhat and updates NN
    void Update( double dt,
                 Eigen::VectorXd & fc);

	void UpdateCart( Eigen::VectorXd & q,
                     Eigen::VectorXd & qd,
                     Eigen::VectorXd & x,
                     Eigen::VectorXd & xd,
                     Eigen::VectorXd & x_m,
                     Eigen::VectorXd & xd_m,
                     Eigen::VectorXd & xdd_m,
                     double dt,
                     Eigen::VectorXd & f_h,
                     Eigen::VectorXd & fc);

	void UpdateJoint( Eigen::VectorXd & q,
                      Eigen::VectorXd & qd,
                      Eigen::VectorXd & q_m,
                      Eigen::VectorXd & qd_m,
                      Eigen::VectorXd & qdd_m,
					  double dt,
                      Eigen::VectorXd & fc);


	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void NNController::UpdateCart( Eigen::VectorXd & q,
                               Eigen::VectorXd & qd,
                               Eigen::VectorXd & x,
                               Eigen::VectorXd & xd,
                               Eigen::VectorXd & x_m,
                               Eigen::VectorXd & xd_m,
                               Eigen::VectorXd & xdd_m,
                               double dt,
                               Eigen::VectorXd & f_h,
                               Eigen::VectorXd & fc)
{
	// Update error signals
	e  = x_m  - x;
	ed = xd_m - xd;

	// Update NN input vector
	if(PED_ON)
	{
		phi << 1, fl, fl_dot, La.diagonal(), La_dot.diagonal(), q, qd, e, ed, x_m, xd_m, xdd_m;
	}
	else
	{
		if(bias == 1)
			phi << 1, q, qd, e, ed, x_m, xd_m, xdd_m;
		else
			phi << q, qd, e, ed, x_m, xd_m, xdd_m;
	}

	fh = f_h;

	Update(dt, fc);
}

void NNController::UpdateJoint( Eigen::VectorXd & q,
                                Eigen::VectorXd & qd,
                                Eigen::VectorXd & q_m,
                                Eigen::VectorXd & qd_m,
                                Eigen::VectorXd & qdd_m,
                                double dt,
                                Eigen::VectorXd & fc)
{
	// Update error signals
	e  = q_m  - q;
	ed = qd_m - qd;

	// Update NN input vector
	if(bias == 1)
		phi << 1, e, ed, q_m, qd_m, qdd_m;
	else
		phi << e, ed, q_m, qd_m, qdd_m;		// TODO check if q, qd are needed

	Update(dt, fc);
}

void NNController::Update( double dt,
                           Eigen::VectorXd & fc)
{
	W_ = W_next_;
	V_ = V_next_;

	W_trans = W_.transpose();
	V_trans = V_.transpose();

	// Prescribed error dynamics
	if(PED_ON)
	{
		Ga = Dd - La;

		La_dot = Kd - Ga*La;
		La += La_dot*dt;

		fl_dot = fh - Ga*fl;
		fl = fl + fl_dot*dt;

		r = ed + La*e - fl;
	}
	else
	{
		// Sliding mode
		r = ed + La*e;
	}

	// Robustifying term
	if(robust_ON)
	{
		Z.block(0,0,num_Hidden+bias,num_Outputs) = W_;
		Z.block(num_Hidden+bias,num_Outputs,num_Inputs+bias,num_Hidden) = V_;
		v = - Kz*(Z.norm() + Zb)*r;
	}

	hiddenLayerIn = V_trans*phi;
	sigma = activation(hiddenLayerIn);
	fhat = W_trans*sigma;

	// Control signal
	if(nn_ON)
		fc = Kv*r + fhat - v - fh;
	else
		fc = Kv*r;

	if(!updateWeights)	// TODO check if it's time to update
		return;
	//
	sigmaPrime = activationPrime(hiddenLayerIn);

	// Wk+1 = Wk +  Wkdot * dt
	W_next_ = W_ + (F*sigma*r.transpose() - F*sigmaPrime*V_trans*phi*r.transpose() - kappa*F*r.norm()*W_) * dt;

	if(!updateInnerWeights)
		return;

	sigmaPrimeTrans_W_r = sigmaPrime.transpose()*W_*r;      // make sigmaPrimeTrans_W_r_tran = r_tran*sigmaPrime*W_trans

	// Vk+1 = Vk +  Vkdot * dt
	V_next_ = V_ + (G*phi*sigmaPrimeTrans_W_r.transpose() - kappa*G*r.norm()*V_) * dt;

}

Eigen::VectorXd NNController::activation( Eigen::VectorXd z)
{
	if( z.size() != num_Hidden)
	{
		std::cerr<<"Activation input vector has the wrong dimensions!\n";
	}

	Eigen::VectorXd y;
	y.resize(num_Hidden+bias);

	switch(actF)
	{
	case NNController::Sigmoid:
	{
		for(int i=0;i<num_Hidden;i++)
		{
			y(i) = 1.0/(1.0 + exp(-(double)z(i)));
		}
		break;
	}
	case NNController::RBF:
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

Eigen::MatrixXd NNController::activationPrime( Eigen::VectorXd z)
{
	if( z.size() != num_Hidden)
	{
		std::cerr<<"ActivationPrime input vector has the wrong dimensions!\n";
	}

	Eigen::MatrixXd y;
	y.setZero(num_Hidden+bias,num_Hidden);	// The last row will be zero if there is a bias unit.

	switch(actF)
	{
	case NNController::Sigmoid:
	{
		Eigen::VectorXd s = activation(z);
		Eigen::MatrixXd S = s.asDiagonal();
		y.block(0,0,num_Hidden,num_Hidden) = S - S*S;
		break;
	}
	case NNController::RBF:
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

#endif /* NNCONTROLLER_H_ */
