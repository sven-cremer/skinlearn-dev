/**
 *
 * kalman_estimate.cpp: ...
 *
 *
 * @author Isura Ranatunga, University of Texas at Arlington, Copyright (C) 2013.
 * @contact isura.ranatunga@mavs.uta.edu
 * @see ...
 * @created May 21, 2013
 * @modified May 21, 2013
 *
 */

#include <oel/kalman_estimate.hpp>

namespace oel
{

namespace kalman
{

  void DiscreteTimeKalmanFilter::TimeUpdate( const Eigen::MatrixXd & Uk )
  {

    // Error covariance
    Pk = Ak*Pk*Ak.transpose() + Gk*Qk*Gk.transpose();

    // Estimate
    Xk_hat = Ak*Xk_hat + Bk*Uk;

  }

  void DiscreteTimeKalmanFilter::MeasurementUpdate( const Eigen::MatrixXd & Zk )
  {

    // Error covariance
    Pk = Eigen::MatrixXd( Pk.inverse() + Hk.transpose()*Rk.inverse()*Hk ).inverse();

    // Estimate
    Xk_hat = Xk_hat + Pk*Hk.transpose()*Rk.inverse()*( Zk - Hk*Xk_hat );

  }

  void DiscreteTimeKalmanFilter::Update( const Eigen::MatrixXd & Zk,
                                         const Eigen::MatrixXd & Uk )
  {

    // Time update ( effect of system dynamics )
    DiscreteTimeKalmanFilter::TimeUpdate( Uk );

    // Measurement update ( effect of measurement Zk )
    DiscreteTimeKalmanFilter::MeasurementUpdate( Zk );

  }

  Eigen::MatrixXd DiscreteTimeKalmanFilter::getStateEstimate()
  {
    return Xk_hat;
  }


}

}

