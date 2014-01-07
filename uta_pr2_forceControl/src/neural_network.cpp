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

void TwoLayerNeuralNetworkController::Update( SystemVector & qd_m  ,
											  SystemVector & qd    ,
											  SystemVector & q_m   ,
											  SystemVector & q     ,
											  SystemVector & qdd_m ,
											  SystemVector & t_r   ,
											  SystemVector & tau    )
{
	W_trans = W_trans_next;
	V_trans = V_trans_next;

	// Filtered error
	r = (qd_m - qd) + lambda*(q_m - q);

	// Robust term
	Z.block(0,0,Hidden,Outputs) = W_trans.transpose();
	Z.block(Hidden,Outputs,Inputs+1,Hidden) = V_trans.transpose();
	vRobust = - Kz*(Z.norm() + Zb)*r;

	x(0 ) =                  1   ;
	x(1 ) = (  q_m( 0 ) -  q(0) ); //   q( 0 ) ;
	x(2 ) = (  q_m( 1 ) -  q(1) ); //   q( 1 ) ;
	x(3 ) = (  q_m( 2 ) -  q(2) ); //   q( 2 ) ;
	x(4 ) = (  q_m( 3 ) -  q(3) ); //   q( 3 ) ;
	x(5 ) = (  q_m( 4 ) -  q(4) ); //   q( 4 ) ;
	x(6 ) = (  q_m( 5 ) -  q(5) ); //   q( 5 ) ;
	x(7 ) = (  q_m( 6 ) -  q(6) ); //   q( 6 ) ;
	x(8 ) = ( qd_m( 0 ) - qd(0) ); //  qd( 0 ) ;
	x(9 ) = ( qd_m( 1 ) - qd(1) ); //  qd( 1 ) ;
	x(10) = ( qd_m( 2 ) - qd(2) ); //  qd( 2 ) ;
	x(11) = ( qd_m( 3 ) - qd(3) ); //  qd( 3 ) ;
	x(12) = ( qd_m( 4 ) - qd(4) ); //  qd( 4 ) ;
	x(13) = ( qd_m( 5 ) - qd(5) ); //  qd( 5 ) ;
	x(14) = ( qd_m( 6 ) - qd(6) ); //  qd( 6 ) ;
	x(15) =             q_m( 0 ) ;
	x(16) =             q_m( 1 ) ;
	x(17) =             q_m( 2 ) ;
	x(18) =             q_m( 3 ) ;
	x(19) =             q_m( 4 ) ;
	x(20) =             q_m( 5 ) ;
	x(21) =             q_m( 6 ) ;
	x(22) =            qd_m( 0 ) ;
	x(23) =            qd_m( 1 ) ;
	x(24) =            qd_m( 2 ) ;
	x(25) =            qd_m( 3 ) ;
	x(26) =            qd_m( 4 ) ;
	x(27) =            qd_m( 5 ) ;
	x(28) =            qd_m( 6 ) ;
	x(29) =           qdd_m( 0 ) ;
	x(30) =           qdd_m( 1 ) ;
	x(31) =           qdd_m( 2 ) ;
	x(32) =           qdd_m( 3 ) ;
	x(33) =           qdd_m( 4 ) ;
	x(34) =           qdd_m( 5 ) ;
	x(35) =           qdd_m( 6 ) ;

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

	// Vk+1                  = Vk                  +  Vkdot                                                                                      			 * dt
	V_trans_next.transpose() = V_trans.transpose() + (G*x*(sigmaPrime.transpose()*W_trans.transpose()*r).transpose() - kappa*G*r.norm()*V_trans.transpose()) * delT;

}

Eigen::Matrix<double, TwoLayerNeuralNetworkController::Hidden, 1>
TwoLayerNeuralNetworkController::sigmoid( Eigen::Matrix<double, TwoLayerNeuralNetworkController::Hidden, 1> & z )
{
  for(uint i=0;i<z.size();i++)
  {
	z(i) = 1.0/(1.0 + exp(-(double)z(i)));
  }
  return z;
}

}

}



