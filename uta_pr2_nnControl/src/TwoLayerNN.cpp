/*
 * TwoLayerNN.cpp
 *
 *  Created on: Sep 7, 2013
 *      Author: Isura
 */

#include <TwoLayerNN.h>

namespace nn
{
	TwoLayerNN::TwoLayerNN(int n_, int m_, int l_, double kappa_, double Kp_, double Kd_, double G_, double F_)
	{
		init( n_, m_,  l_,  kappa_,  Kp_,  Kd_, G_,  F_);
	}

	TwoLayerNN::~TwoLayerNN()
	{
	  //Destructor
	}

	void TwoLayerNN::init(int n_, int m_, int l_, double kappa_, double Kp_, double Kd_, double G_, double F_)
	{
	  //Initialize the matrices
	  n = n_;  // Size of the inputs
	  m = m_;  // Size of the outputs
	  l = l_; // Size of the hidden layer

	  //delT = 0.0001;
	  kappa = kappa_; //0.3
	  Kp    = Kp_; // prop. gain for PID inner loop
	  Kd    = Kd_;//*std::sqrt(Kp); // der. gain for PID inner loop
	  Kz    = 3;
	  Zb    = 100;

	  G.resize(n + 1, n + 1);
	  G = G_*Eigen::MatrixXd::Identity(n + 1, n + 1);

	  F.resize(l,l);
	  F = F_*Eigen::MatrixXd::Identity(l,l);

	  L.resize(m,m);
	  L = Kp*Eigen::MatrixXd::Identity(m,m);

	  //TODO determine which is randomly initialized
	  V_trans.resize(l,n + 1);
	  V_trans = Eigen::MatrixXd::Random(l,n + 1);

	  W_trans.resize(m, l);
	  W_trans = Eigen::MatrixXd::Random(m, l);

	  V_trans_next.resize(l,n + 1);
	  V_trans_next = Eigen::MatrixXd::Random(l,n + 1);

	  W_trans_next.resize(m, l);
	  W_trans_next = Eigen::MatrixXd::Zero(m, l);

	  Z.resize(l+n+1,l+m);
	  Z = Eigen::MatrixXd::Zero(l+n+1,l+m);
	  Z.block(0,0,l,m) = W_trans.transpose();
	  Z.block(l,m,n+1,l) = V_trans.transpose();

	  r.resize(n,1);
	  r = Eigen::VectorXd::Zero(n,1);

	  vRobust.resize(n,1);
	  vRobust = Eigen::VectorXd::Zero(n,1);

	  x.resize(n + 1,1);
	  x = Eigen::VectorXd::Random(n + 1,1);
	  x(0) = 1;

	  y.resize(m,1);
	  y = Eigen::VectorXd::Random(m,1);

	  hiddenLayer_in.resize(l,1);
	  hiddenLayer_in = Eigen::VectorXd::Zero(l,1);

	  hiddenLayer_out.resize(l,1);
	  hiddenLayer_out = Eigen::VectorXd::Zero(l,1);

	  outputLayer_out.resize(m,1);
	  outputLayer_out = Eigen::VectorXd::Zero(m,1);

	  sigmaPrime.resize(m, l);
	  sigmaPrime = Eigen::MatrixXd::Zero(l, l);
	}


	Eigen::VectorXd TwoLayerNN::getFilteredError()
	{
		return r;
	}

	Eigen::VectorXd TwoLayerNN::getRobustifyingSignal()
	{
		Z.block(0,0,l,m) = W_trans.transpose();
		Z.block(l,m,n+1,l) = V_trans.transpose();

		double Frob_Z;


		Frob_Z = Z.norm();

		/*std::cout << Frob_Z;*/

		vRobust = -Kz*(Frob_Z + Zb)*r;
		return vRobust;
	}

	Eigen::VectorXd TwoLayerNN::NNAug_out(Eigen::VectorXd& qDes, Eigen::VectorXd& q, Eigen::VectorXd& qDesDot, Eigen::VectorXd& qDot, double dt)
	{

	  W_trans = W_trans_next;
	  V_trans = V_trans_next;

	  // Filtered error
	  r = Kd*(qDesDot - qDot) + L*(qDes - q);
	  x << 1,
		   q,
		   qDot;

	  hiddenLayer_in = V_trans*x;
	  hiddenLayer_out = sigmoid(hiddenLayer_in);
	  outputLayer_out = W_trans*hiddenLayer_out;
	  y = outputLayer_out;

	  sigmaPrime = hiddenLayer_out.asDiagonal()*(Eigen::MatrixXd::Identity(hiddenLayer_out.rows(),hiddenLayer_out.rows()) - Eigen::MatrixXd::Identity(hiddenLayer_out.rows(),hiddenLayer_out.rows())*hiddenLayer_out.asDiagonal());

	  VAug_update( q, r, dt);
	  WAug_update( q, r, dt);

	  return y;

	}

	void TwoLayerNN::VAug_update(Eigen::VectorXd& q, Eigen::VectorXd& r, double delT)
	{
	  Eigen::MatrixXd temp = (sigmaPrime.transpose()*W_trans.transpose()*r);
	  V_trans_next.transpose() = V_trans.transpose() + (G*x*temp.transpose() - kappa*G*r.norm()*V_trans.transpose())*delT;
	}

	void TwoLayerNN::WAug_update(Eigen::VectorXd& q, Eigen::VectorXd& r, double delT)
	{
	  W_trans_next.transpose() = W_trans.transpose() + (F*hiddenLayer_out*r.transpose() - F*sigmaPrime*V_trans*x*r.transpose() - kappa*F*r.norm()*W_trans.transpose())*delT;
	}

	Eigen::VectorXd TwoLayerNN::sigmoid(Eigen::VectorXd& z)
	{

	  for(uint i=0;i<z.size();i++)
	  {
		z(i) = 1.0/(1.0 + exp(-z(i)));
	  }

	  return z;

	}
}


