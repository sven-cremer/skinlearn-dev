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

	//JointVec sigma_;		// Modification term for weight update
	double sigma;
	double rbf_var;			// RBF variance = width^2
	double rbf_mag;			// RBF amplitude of center
	//Eigen::MatrixXd gamma_;	// For updating NN Weight [numNodes x numNodes]
	double gamma;
	Eigen::MatrixXd Mu_;	// RBF center for each node n [NNinput x numNodes]

	JointVec x1;			// Joint position
	JointVec x2;			// Joint velocity
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

        sigma = 0.02;
        //sigma_.setOnes();
        //sigma_ *= sigma;

        rbf_var = 1.0;
        rbf_mag = 1.0;

        gamma = 10;
        //gamma_.resize(numNodes,numNodes);
        //gamma_.setIdentity();
        //gamma_ *= gamma;

        // If the centers are -1 or 1, then there are 2^(numOutputs*4)=2^28 possible combinations.
        // This would require too many nodes.
        Mu_.resize(NNinput,numNodes);
        Mu_.setRandom();								// uniform dist between (-1,1)
        Eigen::MatrixXd pos(NNinput,numNodes);
        Eigen::MatrixXd neg(NNinput,numNodes);
        pos.setOnes();
        neg = -pos;
        Mu_ = (Mu_.array() < 0).select(neg,pos);		// make elements -1 or +1

        // Initialize states
        // x1
        x2.setZero();

        // Set Gains
        K1.setIdentity();
        K1 *= 50;
        K2.setIdentity();
        K2 *= 50;

	}

//	void init( double p_variable )
//	{
//
//	}

	void updateDelT( double p_dT )
	{
	  dT = p_dT;
	}

	void setUpdateWeights(bool updateWeights_)
	{
		updateWeights = updateWeights_;
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

		e1 = q - q_m;			// Position error (15)
		e1_dot = qd - qd_m;		// Velocity error (16)

		a1 = qd_m - K1*e1;		// Virtual control (21)
		e2 = qd - a1;			// Error variable (17)
		a1_dot = qdd_m - K1*e1_dot;

		Z_ << q, qd, a1, a1_dot;	// Create NN input vector

	    activationFcn(Z_,S_);		// Compute S using RBF

	    W_dot = gamma*(e2*S_.transpose()-sigma*W_);	// TODO: use gamma and sigma matricies instead

	    W_nxt = W_dot*dT + W_;		// Compute next weight update

	    tau = -e1 - K2*e2 + W_*S_;	// Control signal

	}


	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} // end neural_network namespace

} // end csl namespace




#endif /* ICE_ROBOT_CONTROLLERS_INCLUDE_CSL_RBF_NN_HPP_ */
