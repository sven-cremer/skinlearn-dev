/*
 * TwoLayerNN.h
 *
 *  Created on: Sep 7, 2013
 *      Author: Isura
 */

#ifndef TWOLAYERNN_H_
#define TWOLAYERNN_H_


#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>
#include <math.h>

namespace nn
{
  class TwoLayerNN
  {

    public:
            Eigen::MatrixXd V_trans, W_trans, G, F, L, Z, V_trans_next, W_trans_next, sigmaPrime;
            Eigen::VectorXd x, y, hiddenLayer_out, hiddenLayer_in, outputLayer_out, r, vRobust;
            int n, l, m;
            double  kappa, Kp, Kd, Kz, Zb; //delT

			TwoLayerNN(int n_, int m_, int l_, double kappa_, double Kp_, double Kd_, double G_, double F_);

            void init(int n_, int m_, int l_, double kappa_, double Kp_, double Kd_, double G_, double F_);

            ~TwoLayerNN();

            Eigen::VectorXd getFilteredError();
            Eigen::VectorXd getRobustifyingSignal();

            // This would give the W'sigma(V'x)
            Eigen::VectorXd NNAug_out(Eigen::VectorXd& qDes, Eigen::VectorXd& q, Eigen::VectorXd& qDesDot, Eigen::VectorXd& qDot, double dt);

            //This would give the update V(k+1) = V(k) + V'sigma(V'x)
            void VAug_update(Eigen::VectorXd& q, Eigen::VectorXd& r, double delT);

            void WAug_update(Eigen::VectorXd& q, Eigen::VectorXd& r, double delT);


    private:
            Eigen::VectorXd sigmoid(Eigen::VectorXd& z);

  };
}


#endif /* TWOLAYERNN_H_ */
