/**
 *
 * twoLayerNN: this is a class file that contains the two layer NN code
 *
 *
 * @author Isura Ranatunga, University of Texas at Arlington, Copyright (C) 2013.
 * @contact isura.ranatunga@mavs.uta.edu
 * @see ...
 * @created 02/13/2013
 * @modified 02/14/2013
 *
 */


#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>
#include <math.h>

//#include <twoLayerNN.h>

namespace nn
{

  class twoLayerNN
  {

    public:

            Eigen::MatrixXd V_trans, W_trans, G, F, L, Z, V_trans_next, W_trans_next, sigmaPrime;
            Eigen::VectorXd x, y, hiddenLayer_out, hiddenLayer_in, outputLayer_out, r, vRobust;
            int n, l, m;
            double  kappa, Kp, Kd, Kz, Zb; //delT

	    twoLayerNN(int a1, int a2, int a3, double a4, double a5, double a6, double a7, double a8)
            {

		init(a1, a2, a3, a4, a5, a6, a7, a8);

	    }

            void init(int a1, int a2, int a3, double a4, double a5, double a6, double a7, double a8)
            {
              //Initialize the matrices
              n = a1;  // Size of the inputs
              m = a2;  // Size of the outputs
              l = a3; // Size of the hidden layer


              //delT = 0.0001;
              kappa = a4; //0.3
              Kp = a5; // prop. gain for PID inner loop
              Kd = a6;//*std::sqrt(Kp); // der. gain for PID inner loop
              Kz = 3;
              Zb = 100;




			  G.resize(n + 1, n + 1);
			  G = a7*Eigen::MatrixXd::Identity(n + 1, n + 1);


			  F.resize(l,l);
			  F = a8*Eigen::MatrixXd::Identity(l,l);

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

            ~twoLayerNN()
            {
              //Destructor
            }

            Eigen::VectorXd getFilteredError(){return r;}
            
            Eigen::VectorXd getRobustifyingSignal()
            {
				Z.block(0,0,l,m) = W_trans.transpose();
				Z.block(l,m,n+1,l) = V_trans.transpose();

				double Frob_Z;


				Frob_Z = Z.norm();

				/*std::cout << Frob_Z;*/

				vRobust = -Kz*(Frob_Z + Zb)*r;
				return vRobust;
            }



            // This would give the W'sigma(V'x)
            Eigen::VectorXd NNAug_out(Eigen::VectorXd& qDes, Eigen::VectorXd& q, Eigen::VectorXd& qDesDot, Eigen::VectorXd& qDot, double dt)
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



            //This would give the update V(k+1) = V(k) + V'sigma(V'x)
            void VAug_update(Eigen::VectorXd& q, Eigen::VectorXd& r, double delT)
            {
              Eigen::MatrixXd temp = (sigmaPrime.transpose()*W_trans.transpose()*r);
              V_trans_next.transpose() = V_trans.transpose() + (G*x*temp.transpose() - kappa*G*r.norm()*V_trans.transpose())*delT;
            }

            void WAug_update(Eigen::VectorXd& q, Eigen::VectorXd& r, double delT)
            {
              W_trans_next.transpose() = W_trans.transpose() + (F*hiddenLayer_out*r.transpose() - F*sigmaPrime*V_trans*x*r.transpose() - kappa*F*r.norm()*W_trans.transpose())*delT;
            }



    private:

            Eigen::VectorXd sigmoid(Eigen::VectorXd& z)
            {

              for(uint i=0;i<z.size();i++)
              {
                z(i) = 1.0/(1.0 + exp(-z(i)));
              }

              return z;

            }


  };
}
