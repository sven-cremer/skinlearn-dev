/*
 * outer_loop.h
 *
 *  Created on: Jan 22, 2014
 *      Author: Isura
 */

#ifndef RLS_OUTER_LOOP_H_
#define RLS_OUTER_LOOP_H_

#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <Eigen/Core>

#include "oel/least_squares.hpp"

// TODO take this inside class
typedef boost::array<double, 21> state_type;
typedef boost::array<double, 3> fir_state_type;
typedef boost::array<double, 6> oneDmsd_state_type;

void mass_spring_damper_model( const state_type &x , state_type &dxdt , double t )
{
//      double a = 10  ;
      double m = 1  ; // double m = a*a ;
      double d = 10 ; // double d = 2*a ;
      double k = 1  ; // double k = a*a ;

      dxdt[0 ] = x[7 ];
      dxdt[1 ] = x[8 ];
      dxdt[2 ] = x[9 ];
      dxdt[3 ] = x[10];
      dxdt[4 ] = x[11];
      dxdt[5 ] = x[12];
      dxdt[6 ] = x[13];

      //             f_r               qd_m      q_m
      dxdt[7 ] = m*( x[14] - d*x[7 ] - k*x[0 ] );
      dxdt[8 ] = m*( x[15] - d*x[8 ] - k*x[1 ] );
      dxdt[9 ] = m*( x[16] - d*x[9 ] - k*x[2 ] );
      dxdt[10] = m*( x[17] - d*x[10] - k*x[3 ] );
      dxdt[11] = m*( x[18] - d*x[11] - k*x[4 ] );
      dxdt[12] = m*( x[19] - d*x[12] - k*x[5 ] );
      dxdt[13] = m*( x[20] - d*x[13] - k*x[6 ] );

      dxdt[14] = 0 ;
      dxdt[15] = 0 ;
      dxdt[16] = 0 ;
      dxdt[17] = 0 ;
      dxdt[18] = 0 ;
      dxdt[19] = 0 ;
      dxdt[20] = 0 ;
}

void oneDmsd_model( const oneDmsd_state_type &x , oneDmsd_state_type &dxdt , double t )
{
      double  m = x[3 ]  ; // mass
      double  k = x[4 ]  ; // spring
      double  d = x[5 ]  ; // damper

      dxdt[0 ] = x[1 ];

      //           f_r      xd_m      x_m
      dxdt[1 ] = ( x[2] - d*x[1 ] - k*x[0 ] )/m;

      dxdt[2] = 0 ;

      dxdt[3 ] = 0  ; // mass
      dxdt[4 ] = 0  ; // spring
      dxdt[5 ] = 0  ; // damper

}

void task_model( const fir_state_type &x , fir_state_type &dxdt , double t )
{
      double a = 10; //0.004988;
      double b = 10; //0.995;
//      double m = a*a;
//      double d = 2*a;
//      double k = a*a;

      dxdt[0 ] = x[1 ];

      //           q_r       q_d
      dxdt[1 ] = a*x[2] -  b*x[0 ] ;

      dxdt[2] = 0 ;

}

#include<csl/JSpaceMsdModel.h>
#include<csl/MsdModel.h>
#include<csl/RlsModel.h>
#include<csl/CtRlsModel.h>
#include<csl/MracModel.h>
#include<csl/IrlModel.h>

#endif /* RLS_OUTER_LOOP_H_ */
