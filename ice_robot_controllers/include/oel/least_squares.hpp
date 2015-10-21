/**
 *
 * least_squares.hpp: ...
 *
 *
 * @author Isura Ranatunga, University of Texas at Arlington, Copyright (C) 2013.
 * @contact isura.ranatunga@mavs.uta.edu
 * @see ...
 * @created Dec 26, 2013
 * @modified Dec 26, 2013
 *
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace oel
{

namespace ls
{

/*
 * This class will implement the Recursive Least Squares Filter as described in
 * Adaptive Filter Theory, Second Edition by Simon Haykin; 2002
 * TODO add more description
 */
class RLSFilter
{
  private:
            Eigen::MatrixXd Wk;
            Eigen::MatrixXd Wk_T;

            Eigen::MatrixXd Uk;
            Eigen::MatrixXd Uk_T;
            Eigen::MatrixXd Dk;

            Eigen::MatrixXd Pk;

            Eigen::MatrixXd lm;
            double          lmInv;

            Eigen::MatrixXd PIk;
            Eigen::MatrixXd Kk;

  public:
            RLSFilter()
            {
              lmInv = 1;
            }


            void init( const Eigen::MatrixXd & w ,
                       const Eigen::MatrixXd & u ,
                       const Eigen::MatrixXd & d ,
                       const Eigen::MatrixXd & P0,
                       const double          & l  )
            {
              Wk     = w ;
              Uk     = u ;
              Dk     = d ;
              Pk     = P0;

              lm.resize(1,1);
              lm    << l ;
              lmInv  = 1/l;


              PIk    = Eigen::MatrixXd::Zero( w.rows(), 1 );
              Kk     = Eigen::MatrixXd::Zero( w.rows(), 1 );
            }

            ~RLSFilter()
            {

            }

            void setLambda( const double & l )
            {
              lm.resize(1,1);
			  lm    << l ;
			  lmInv  = 1/l;
            }

            void Update( const Eigen::MatrixXd & w ,
                         const Eigen::MatrixXd & u ,
                         const Eigen::MatrixXd & d ,
                         const Eigen::MatrixXd & P0 );

            Eigen::MatrixXd getEstimate();
            Eigen::MatrixXd getCovariance();


            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

}
