/*
 * rbf_nn.hpp
 *
 *   Created on: Jan 26, 2016
 *       Author: Sven Cremer
 *  Description: Implementation of a RBF NN
 */

#ifndef ICE_ROBOT_CONTROLLERS_INCLUDE_CSL_RBF_NN_HPP_
#define ICE_ROBOT_CONTROLLERS_INCLUDE_CSL_RBF_NN_HPP_

#include <Eigen/Core>


namespace csl {

namespace neural_network {


class RBFNeuralNetworkController {

	// Definitions
	enum { Joints = 7 };
	enum { NNinput = Joints*4 };
	typedef Eigen::Matrix<double, Joints , 1> JointVec;
	typedef Eigen::Matrix<double, Joints , Joints> JointMat;
	typedef Eigen::Matrix<double, NNinput, 1> NNVec;

	bool updateWeights;

	int numNodes;			// Number of NN nodes, index n
	int numOutputs;			// Number of outputs (joints), index j

	Eigen::MatrixXd W_;		// Current NN weights for each output [numOutputs x numNodes]
	Eigen::MatrixXd W_nxt;  // Next NN weights
	Eigen::MatrixXd W_dot;	// Derivative of NN weights
	Eigen::VectorXd S_;		// Activation function output

	double sigma;			// Modification term for weight update
	double rbf_var;			// RBF variance = width^2
	double rbf_mag;			// RBF amplitude of center
	JointMat Gamma_;		// For updating NN Weight [numNodes x numNodes]
	double gamma;
	Eigen::MatrixXd Mu_;	// RBF center for each node n [NNinput x numNodes]

//	JointVec x1;			// Joint position
//	JointVec x2;			// Joint velocity
	JointVec e1;			// Position error
	JointVec e1_dot;		// Velocity error
	JointVec e2;			// Error variable
	JointVec e2_dot;
	JointVec a1;			// alpha
	JointVec a2;
	JointVec a1_dot;		// alpha derivative
	JointVec a2_dot;
	JointMat K1;			// Gains
	JointMat K2;			// Gains
	NNVec Z_;				// Input vector for NN

	double dT;				// Time step

	// Hysteresis nonlinearity NN
	bool enableHysteresisNN;
	JointMat beta;
	JointMat beta_nxt;
	JointMat beta_dot;
	JointMat Gamma_beta_;
	double gamma_beta;
	double sigma_beta;
	double b;
	Eigen::VectorXd tanh_;
	Eigen::VectorXd e2b_;

public:

	RBFNeuralNetworkController()
	{
		updateWeights = true;

        dT = 0.001;				// 1000 Hz by default

        numNodes 	= 16;
        numOutputs	= Joints;	// 7 by default

        // Initialize Parameters
        W_.   resize(numOutputs, numNodes);
        W_dot.resize(numOutputs, numNodes);
        W_nxt.resize(numOutputs, numNodes);
        W_.   setZero();
        W_dot.setZero();
        W_nxt.setZero();

        S_.resize(numNodes);

        rbf_var = 1.0;
        rbf_mag = 1.0;

        sigma = 0.01;
        gamma = 50;
        Gamma_.setIdentity();
        Gamma_ *= gamma;

        // If the centers are -1 or 1, then there are 2^(numOutputs*4)=2^28 possible combinations.
        // This would require too many nodes.
        Mu_.resize(NNinput,numNodes);
        Mu_.setRandom();								// uniform dist between (-1,1)
        Eigen::MatrixXd pos(NNinput,numNodes);
        Eigen::MatrixXd neg(NNinput,numNodes);
        pos.setOnes();
        neg = -pos;
        Mu_ = (Mu_.array() < 0).select(neg,pos);		// make elements -1 or +1

        // Set Gains
        K1.setIdentity();
        K1 *= 5;
        K2.setIdentity();
        K2 *= 20;

    	// Hysteresis nonlinearity
        enableHysteresisNN = true;
        beta.setZero();
        beta_dot.setZero();
        beta_nxt.setZero();
    	b = 1;
    	sigma_beta = 0.01;
    	gamma_beta = 50;
    	Gamma_beta_.setIdentity();
    	Gamma_beta_ *= gamma_beta;
	}

	void init( double p_K1, double p_K2,
			   double p_sigma_w, double p_gamma_w,
			   double p_sigma_b, double p_gamma_b,
			   double b)
	{

	}

	void updateDelT( double p_dT )
	{
	  dT = p_dT;
	}

	void setUpdateWeights(bool updateWeights_)
	{
		updateWeights = updateWeights_;
	}

	void hyperbolicTangent(const Eigen::VectorXd & Z,	// input
		                     	 Eigen::VectorXd & S )	// output
	{
		int N = Z.size();
		S.resize(N);
		for(int n=0;n<N;n++)
		{
			S(n) = tanh( Z(n) );
		}

	}

	void activationFcn(const Eigen::MatrixXd & Z,	// input
		                     Eigen::VectorXd & S )	// output
	{
		NNVec x;
		double tmp;
		for(int n=0;n<numNodes;n++)
		{
			x = Z-Mu_.col(n);
			tmp = x.transpose() * x;
			S(n) = exp( -( tmp ) / rbf_var );
		}
	}


	void update( double delT			 ,		// time step
				 Eigen::VectorXd & q     ,		// x1
                 Eigen::VectorXd & qd    ,		// x2
                 Eigen::VectorXd & q_m   ,		// xd
                 Eigen::VectorXd & qd_m  ,		// vd
                 Eigen::VectorXd & qdd_m ,		// ad
                 Eigen::VectorXd & tau   )
	{

		dT = delT;
		W_ = W_nxt;				// Update weight
		beta = beta_nxt;

		e1 = q - q_m;			// Position error (15)
		e1_dot = qd - qd_m;		// Velocity error (16)

		a1 = qd_m - K1*e1;		// Virtual control (21)
		e2 = qd - a1;			// Error variable (17)
		a1_dot = qdd_m - K1*e1_dot;

		Z_ << q, qd, a1, a1_dot;	// Create NN input vector

	    activationFcn(Z_,S_);		// Compute S using RBF

	    W_dot = Gamma_*(e2*S_.transpose()-sigma*W_);	// TODO: use gamma and sigma matricies instead

	    W_nxt = W_dot*dT + W_;		// Compute next weight update

	    tau = -e1 - K2*e2 + W_*S_;	// Control signal

    	// Hysteresis nonlinearity
	    if(!enableHysteresisNN)
	    	return;
	    e2b_ = e2 / b;
	    hyperbolicTangent(e2b_,tanh_);
    	beta_dot = Gamma_beta_*(tanh_*e2.transpose() - sigma_beta*beta);
    	beta_nxt = beta_dot*dT + beta;
    	tau = tau - beta*tanh_;

	}


	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} // end neural_network namespace

} // end csl namespace




#endif /* ICE_ROBOT_CONTROLLERS_INCLUDE_CSL_RBF_NN_HPP_ */
