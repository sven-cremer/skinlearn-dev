/*
 * rbf_nn.hpp
 *
 *   Created on: Jan 26, 2016
 *       Author: Sven Cremer
 *  Description: Implementation of a RBF NN
 */

#ifndef ICE_ROBOT_CONTROLLERS_INCLUDE_CSL_RBF_NN_HPP_
#define ICE_ROBOT_CONTROLLERS_INCLUDE_CSL_RBF_NN_HPP_

#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/StdVector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace csl {

namespace neural_network {


class RBFNeuralNetworkController {

	// Definitions
	enum { Joints = 7 };
	typedef Eigen::Matrix<double, Joints  , 1> JointVec;
	typedef Eigen::Matrix<double, Joints*4, 1> NNVec;

	bool updateWeights;

	int numNodes;			// Number of NN nodes, index n
	int numOutputs;			// Number of outputs (joints), index j

	std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > W_j;	// Current NN weights for each output j

	JointVec sigma;			// Modification term for weight update
	double rbf_var;			// RBF variance = width^2
	double rbf_mag;			// RBF amplitude of center
	Eigen::MatrixXd gamma;	// For updating NN Weight [numNodes x numNodes]

	std::vector<NNVec, Eigen::aligned_allocator<NNVec> > Mu_;	// RBF center for each node n

	JointVec x1;			// Joint position
	JointVec x2;			// Joint velocity
	JointVec a1;			// alpha
	JointVec a2;
	JointVec a1_dot;		// alpha derivative
	JointVec a2_dot;

	double dT;				// Time step

//
//        double num_Inputs  ; // n Size of the inputs
//        double num_Outputs ; // m Size of the outputs
//        double num_Hidden  ; // l Size of the hidden layer
//        double num_Error   ; // filtered error
//        double num_Joints  ; // number of joints.
//
//
//    Eigen::MatrixXd V_;
//    Eigen::MatrixXd W_;
//    Eigen::MatrixXd V_next_;
//    Eigen::MatrixXd W_next_;
//
//        Eigen::MatrixXd V_trans;
//        Eigen::MatrixXd W_trans;
//        Eigen::MatrixXd G;
//        Eigen::MatrixXd F;
//        Eigen::MatrixXd L;
//        Eigen::MatrixXd Z;
//
//        Eigen::MatrixXd x;
//        Eigen::MatrixXd y;
//        Eigen::MatrixXd hiddenLayer_out;
//        Eigen::MatrixXd hiddenLayerIdentity;
//        Eigen::MatrixXd hiddenLayer_in;
//        Eigen::MatrixXd outputLayer_out;
//        Eigen::MatrixXd sigmaPrime;
//        Eigen::MatrixXd r;
//        Eigen::MatrixXd r_tran;
//        Eigen::MatrixXd vRobust;
//        Eigen::MatrixXd sigmaPrimeTrans_W_r;
//
//	double kappa;
//	Eigen::MatrixXd Kv;
//	Eigen::MatrixXd lambda;
//	double Kz;
//	double Zb;
//	double nnF;
//	double nnG;
//	double nn_ON;
//
//	double feedForwardForce;


public:

	RBFNeuralNetworkController()
	{
		updateWeights = true;

        dT = 0.001;				// 1000 Hz by default

        numNodes 	= 16;
        numOutputs	= Joints;	// 7 by default

        // Initialize Parameters
        Eigen::MatrixXd zeroMat;
        zeroMat.resize(numOutputs,1);
        zeroMat.setZero();
        double sigma_ = 0.02;

        for(int i=0;i<numOutputs;i++)
        {
        	W_j.push_back( zeroMat );
        	sigma(i) = sigma_;
        }

        rbf_var = 1.0;
        rbf_mag = 1.0;

        gamma.resize(numNodes,numNodes);
        gamma.setIdentity();
        gamma = 10*gamma;


        NNVec randVec;						// If the centers are -1 or 1, then there are 2^(numOutputs*4)=2^28 possible combinations.
        for(int n=0;n<numNodes;n++)			// This would require too many nodes.
        {
        	randVec.Random();
        	//randVec = (Eigen::VectorXi::Random() & 2) -1;
        	Mu_.push_back(randVec);
        }

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
			x = Z-Mu_[n];
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
