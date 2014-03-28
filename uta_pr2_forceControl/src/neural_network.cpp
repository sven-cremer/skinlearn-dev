/*
 * neural_network.cpp
 *
 *
 * @author Isura Ranatunga, University of Texas at Arlington, Copyright (C) 2013.
 * @contact isura.ranatunga@mavs.uta.edu
 * @see ...
 * @created Jan 06, 2014
 * @modified Jan 06, 2014
 *
 */

#include "csl/neural_network.hpp"

namespace csl
{

namespace neural_network
{

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

//// Debug
//  X(3) =  0;
//  X(4) =  0;
//  X(5) =  0;
//
// Xd(3) =  0;
// Xd(4) =  0;
// Xd(5) =  0;
//
// X_m(3) =  0;
// X_m(4) =  0;
// X_m(5) =  0;
//
// Xd_m(3) =  0;
// Xd_m(4) =  0;
// Xd_m(5) =  0;
//
// Xdd_m(3) =  0;
// Xdd_m(4) =  0;
// Xdd_m(5) =  0;
//// Debug

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
        W_trans = W_trans_next;
	V_trans = V_trans_next;

	// Filtered error
	r = (qd_m - qd) + lambda.asDiagonal()*(q_m - q);

	// Robust term
	Z.block(0,0,num_Hidden,num_Outputs) = W_trans.transpose();
	Z.block(num_Hidden,num_Outputs,num_Inputs+1,num_Hidden) = V_trans.transpose();
	vRobust = - Kz*(Z.norm() + Zb)*r;

	hiddenLayer_in = V_trans*x;
	hiddenLayer_out = sigmoid(hiddenLayer_in);
	outputLayer_out = W_trans*hiddenLayer_out;

	y = outputLayer_out;

	// control torques
	tau = Kv.asDiagonal()*r + nn_ON*( y - vRobust ) - feedForwardForce*t_r ;
//	tau = (qd_m - qd) + 100*(q_m - q);

	//
	sigmaPrime = hiddenLayer_out.asDiagonal()*( hiddenLayerIdentity - hiddenLayerIdentity*hiddenLayer_out.asDiagonal() );

	// Wk+1                  = Wk                  +  Wkdot                                                                                                          * dt
	W_trans_next.transpose() = W_trans.transpose() + (F*hiddenLayer_out*r.transpose() - F*sigmaPrime*V_trans*x*r.transpose() - kappa*F*r.norm()*W_trans.transpose()) * delT;

	sigmaPrimeTrans_W_r = sigmaPrime.transpose()*W_trans.transpose()*r;

	// Vk+1                  = Vk                  +  Vkdot                                                                                      			 * dt
	V_trans_next.transpose() = V_trans.transpose() + (G*x*sigmaPrimeTrans_W_r.transpose() - kappa*G*r.norm()*V_trans.transpose()) * delT;

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
