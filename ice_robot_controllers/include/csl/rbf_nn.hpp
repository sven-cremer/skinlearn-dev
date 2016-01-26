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
#include <Eigen/Eigenvalues>

namespace csl {

namespace neural_network {


class RBFNeuralNetworkController {

	// Definitions
	enum { Joints = 7 };
	enum { NNinput = Joints*4 };
	typedef Eigen::Matrix<double, Joints , 1> JointVec;
	typedef Eigen::Matrix<double, NNinput, 1> NNVec;

	bool updateWeights;

	int numNodes;			// Number of NN nodes, index n
	int numOutputs;			// Number of outputs (joints), index j

	Eigen::MatrixXd W_;		// Current NN weights for each output [numOutputs x numNodes]

	JointVec sigma;			// Modification term for weight update
	double rbf_var;			// RBF variance = width^2
	double rbf_mag;			// RBF amplitude of center
	Eigen::MatrixXd gamma;	// For updating NN Weight [numNodes x numNodes]

	Eigen::MatrixXd Mu_;	// RBF center for each node n [NNinput x numNodes]

	JointVec x1;			// Joint position
	JointVec x2;			// Joint velocity
	JointVec a1;			// alpha
	JointVec a2;
	JointVec a1_dot;		// alpha derivative
	JointVec a2_dot;

	double dT;				// Time step

public:

	RBFNeuralNetworkController()
	{
		updateWeights = true;

        dT = 0.001;				// 1000 Hz by default

        numNodes 	= 16;
        numOutputs	= Joints;	// 7 by default

        // Initialize Parameters

        W_.resize(numOutputs, numNodes);
        W_.setZero();

        sigma.setOnes();
        sigma *= 0.02;

        rbf_var = 1.0;
        rbf_mag = 1.0;

        gamma.resize(numNodes,numNodes);
        gamma.setIdentity();
        gamma = 10*gamma;

        // If the centers are -1 or 1, then there are 2^(numOutputs*4)=2^28 possible combinations.
        // This would require too many nodes.
        Mu_.resize(NNinput,numNodes);
        Mu_.setRandom();	// TODO make elements -1 or +1

//        NNVec randVec;
//        randVec = (Eigen::VectorXi::Random() & 2) -1;

        // Initialize state
        // x1
        x2.setZero();

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
		                        Eigen::MatrixXd & S )	// output
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


	void Update( Eigen::VectorXd & q    ,
                  Eigen::VectorXd & qd   ,
                  Eigen::VectorXd & q_m  ,
                  Eigen::VectorXd & qd_m ,
                  Eigen::VectorXd & t_r  ,
                  Eigen::VectorXd & tau   )
	{

		// TODO

		// Create Z
		// Compute S
		// dW_j
		// Wk+1 = Wk +  Wkdot*dt
		// tau

	}


	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

} // end neural_network namespace

} // end csl namespace




#endif /* ICE_ROBOT_CONTROLLERS_INCLUDE_CSL_RBF_NN_HPP_ */
