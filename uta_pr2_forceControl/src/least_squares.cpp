/**
 *
 * least_squares.cpp: ...
 *
 *
 * @author Isura Ranatunga, University of Texas at Arlington, Copyright (C) 2013.
 * @contact isura.ranatunga@mavs.uta.edu
 * @see ...
 * @created Dec 26, 2013
 * @modified Dec 26, 2013
 *
 */

#include <oel/least_squares.hpp>
#include <iostream>

namespace oel
{

namespace ls
{

  void RLSFilter::Update( const Eigen::MatrixXd & w ,
                          const Eigen::MatrixXd & u ,
                          const Eigen::MatrixXd & d ,
                          const Eigen::MatrixXd & P )
  {
    Wk     = w ;
    Uk     = u ;
    Dk     = d ;
    Pk     = P;

    PIk   = Pk*Uk;
    Uk_T  = Uk.transpose();

    Kk    = PIk*( Uk_T*PIk + lm ).inverse();

    Wk_T  = Wk.transpose();
    Wk    = Wk + Kk*( Dk - Wk_T*Uk );

//    Pk    = Pk*lmInv - Kk*Uk_T*Pk*lmInv;
    Pk    = Pk - Kk*Uk_T*Pk;
  }

  Eigen::MatrixXd RLSFilter::getEstimate()
  {
    return Wk;
  }

  Eigen::MatrixXd RLSFilter::getCovariance()
  {
    return Pk;
  }

}

}

