/**
 *
 * kalman_estimate.hpp: ...
 *
 *
 * @author Isura Ranatunga, University of Texas at Arlington, Copyright (C) 2013.
 * @contact isura.ranatunga@mavs.uta.edu
 * @see ...
 * @created May 21, 2013
 * @modified May 21, 2013
 *
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace oel
{

namespace kalman
{

/*
 * This class will implement a Discrete-Time Kalman Filter as described in Table 2.1 pg 72 of
 * Optimal and Robust Estimation with an Introduction to Stochastic Control Theory, Second Edition by Lewis, F.L., et al; 2008
 * TODO add more description
 */
class DiscreteTimeKalmanFilter
{
  private:
            Eigen::MatrixXd Ak;
            Eigen::MatrixXd Bk;
            Eigen::MatrixXd Hk;

            Eigen::MatrixXd Gk;

            Eigen::MatrixXd Qk;
            Eigen::MatrixXd Rk;

            Eigen::MatrixXd Pk;
            Eigen::MatrixXd Xk_hat;

            void TimeUpdate( const Eigen::MatrixXd & Uk );

            void MeasurementUpdate( const Eigen::MatrixXd & Zk );

  public:

            DiscreteTimeKalmanFilter(  )
            {

            }

            DiscreteTimeKalmanFilter( const Eigen::MatrixXd & A,
                                      const Eigen::MatrixXd & B,
                                      const Eigen::MatrixXd & H,
                                      const Eigen::MatrixXd & G,
                                      const Eigen::MatrixXd & Q,
                                      const Eigen::MatrixXd & R,
                                      const Eigen::MatrixXd & X0,
                                      const Eigen::MatrixXd & P0 )
            {

             init( A ,
                   B ,
                   H ,
                   G ,
                   Q ,
                   R ,
                  X0 ,
                  P0  );

            }

            void init( const Eigen::MatrixXd & A,
                       const Eigen::MatrixXd & B,
                       const Eigen::MatrixXd & H,
                       const Eigen::MatrixXd & G,
                       const Eigen::MatrixXd & Q,
                       const Eigen::MatrixXd & R,
                       const Eigen::MatrixXd & X0,
                       const Eigen::MatrixXd & P0 )
            {

              Ak     = A;
              Bk     = B;
              Hk     = H;
              Gk     = G;
              Qk     = Q;
              Rk     = R;

              Xk_hat = X0; // Eigen::MatrixXd::Zero( A.cols(), 1 );
              Pk     = P0; // Eigen::MatrixXd::Zero( 1, 1 );
              // Gk     = Eigen::MatrixXd( Eigen::VectorXd::Ones( A.cols() ).asDiagonal() );

            }

            ~DiscreteTimeKalmanFilter()
            {

            }

            void Update( const Eigen::MatrixXd & Zk,
                         const Eigen::MatrixXd & Uk );

            Eigen::MatrixXd getStateEstimate();


            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

}
