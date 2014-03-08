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

void TwoLayerNeuralNetworkController::Update( Eigen::VectorXd & qd_m  ,
					      Eigen::VectorXd & qd    ,
					      Eigen::VectorXd & q_m   ,
					      Eigen::VectorXd & q     ,
					      Eigen::VectorXd & qdd_m ,
					      Eigen::VectorXd & t_r   ,
					      Eigen::VectorXd & tau    )
{
	W_trans = W_trans_next;
	V_trans = V_trans_next;

	// Filtered error
	r = (qd_m - qd) + lambda*(q_m - q);

	// Robust term
	Z.block(0,0,num_Hidden,num_Outputs) = W_trans.transpose();
	Z.block(num_Hidden,num_Outputs,num_Inputs+1,num_Hidden) = V_trans.transpose();
	vRobust = - Kz*(Z.norm() + Zb)*r;

	// NN Input Vector
        x <<           1 ,
            (  q_m -  q) , //   q( 0 ) ;
            ( qd_m - qd) , //  qd( 0 ) ;
                    q_m  ,
                   qd_m  ,
                  qdd_m  ;


	hiddenLayer_in = V_trans*x;
	hiddenLayer_out = sigmoid(hiddenLayer_in);
	outputLayer_out = W_trans*hiddenLayer_out;

	y = outputLayer_out;

	// control torques
	tau = Kv*r + nn_ON*( y - vRobust ) - feedForwardForce*t_r ;
//	tau = (qd_m - qd) + 100*(q_m - q);

	//
	sigmaPrime = hiddenLayer_out.asDiagonal()*( hiddenLayerIdentity - hiddenLayerIdentity*hiddenLayer_out.asDiagonal() );

	// Wk+1                  = Wk                  +  Wkdot                                                                                                          * dt
	W_trans_next.transpose() = W_trans.transpose() + (F*hiddenLayer_out*r.transpose() - F*sigmaPrime*V_trans*x*r.transpose() - kappa*F*r.norm()*W_trans.transpose()) * delT;

	sigmaPrimeTrans_W_r = sigmaPrime.transpose()*W_trans.transpose()*r;

	// Vk+1                  = Vk                  +  Vkdot                                                                                      			 * dt
	V_trans_next.transpose() = V_trans.transpose() + (G*x*sigmaPrimeTrans_W_r.transpose() - kappa*G*r.norm()*V_trans.transpose()) * delT;

}

Eigen::VectorXd
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
