/*
 * outer_loop.h
 *
 *  Created on: Jan 22, 2014
 *      Author: isura
 */

#ifndef RLS_OUTER_LOOP_H_
#define RLS_OUTER_LOOP_H_

#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

#include <Eigen/Core>

// TODO take this inside class
typedef boost::array<double, 21> state_type;

void mass_spring_damper_model( const state_type &x , state_type &dxdt , double t )
{
      double m = 1;
      double d = 10;
      double k = 1;

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

      dxdt[14] = x[14];
      dxdt[15] = x[15];
      dxdt[16] = x[16];
      dxdt[17] = x[17];
      dxdt[18] = x[18];
      dxdt[19] = x[19];
      dxdt[20] = x[20];
}

namespace csl
{

namespace outer_loop
{

class MsdModel
{

  double num_Joints; // number of joints.

  Eigen::MatrixXd Mm;
  Eigen::MatrixXd Dm;
  Eigen::MatrixXd Km;
  Eigen::MatrixXd MmInv;

  Eigen::MatrixXd q;
  Eigen::MatrixXd qd;
  Eigen::MatrixXd qdd;

  Eigen::MatrixXd q_m;
  Eigen::MatrixXd qd_m;
  Eigen::MatrixXd qdd_m;
  Eigen::MatrixXd t_r;

  double delT; // Time step

  state_type ode_init_x;


public:
  MsdModel()
  {
    num_Joints = 7;
    delT = 0.001; /// 1000 Hz by default

    init( 1, 10, 1 );
  }
  ~MsdModel()
  {
  }

  void changeModelstructure(double para_num_Joints)
  {
    num_Joints = para_num_Joints;
  }

  void init( double p_m, double p_d, double p_k)
  {
    double m = p_m;
    double d = p_d;
    double k = p_k;

    Mm    .resize( num_Joints, num_Joints ) ;
    Dm    .resize( num_Joints, num_Joints ) ;
    Km    .resize( num_Joints, num_Joints ) ;
    MmInv .resize( num_Joints, num_Joints ) ;

    q     .resize( num_Joints, 1 ) ;
    qd    .resize( num_Joints, 1 ) ;
    qdd   .resize( num_Joints, 1 ) ;

    q_m   .resize( num_Joints, 1 ) ;
    qd_m  .resize( num_Joints, 1 ) ;
    qdd_m .resize( num_Joints, 1 ) ;
    t_r   .resize( num_Joints, 1 ) ;

    Mm << m, 0, 0, 0, 0, 0, 0,
         0, m, 0, 0, 0, 0, 0,
         0, 0, m, 0, 0, 0, 0,
         0, 0, 0, m, 0, 0, 0,
         0, 0, 0, 0, m, 0, 0,
         0, 0, 0, 0, 0, m, 0,
         0, 0, 0, 0, 0, 0, m;

    Dm << d, 0, 0, 0, 0, 0, 0,
         0, d, 0, 0, 0, 0, 0,
         0, 0, d, 0, 0, 0, 0,
         0, 0, 0, d, 0, 0, 0,
         0, 0, 0, 0, d, 0, 0,
         0, 0, 0, 0, 0, d, 0,
         0, 0, 0, 0, 0, 0, d;

    Km << k, 0, 0, 0, 0, 0, 0,
         0, k, 0, 0, 0, 0, 0,
         0, 0, k, 0, 0, 0, 0,
         0, 0, 0, k, 0, 0, 0,
         0, 0, 0, 0, k, 0, 0,
         0, 0, 0, 0, 0, k, 0,
         0, 0, 0, 0, 0, 0, k;

    q_m   << 0, 0, 0, 0, 0, 0, 0 ;
    qd_m  << 0, 0, 0, 0, 0, 0, 0 ;
    qdd_m << 0, 0, 0, 0, 0, 0, 0 ;

    t_r   << 0, 0, 0, 0, 0, 0, 0 ;

    MmInv = Mm;

    // initial conditions
    ode_init_x[0 ] = 0.0;
    ode_init_x[1 ] = 0.0;
    ode_init_x[2 ] = 0.0;
    ode_init_x[3 ] = 0.0;
    ode_init_x[4 ] = 0.0;
    ode_init_x[5 ] = 0.0;
    ode_init_x[6 ] = 0.0;

    ode_init_x[7 ] = 0.0;
    ode_init_x[8 ] = 0.0;
    ode_init_x[9 ] = 0.0;
    ode_init_x[10] = 0.0;
    ode_init_x[11] = 0.0;
    ode_init_x[12] = 0.0;
    ode_init_x[13] = 0.0;

    ode_init_x[14] = 0.0;
    ode_init_x[15] = 0.0;
    ode_init_x[16] = 0.0;
    ode_init_x[17] = 0.0;
    ode_init_x[18] = 0.0;
    ode_init_x[19] = 0.0;
    ode_init_x[20] = 0.0;
  }

  void updateDelT(double p_delT)
  {
    delT = p_delT;
  }

  Eigen::MatrixXd getMass()
  {
    return Mm;
  }

  Eigen::MatrixXd getSpring()
  {
    return Km;
  }

  Eigen::MatrixXd getDamper()
  {
    return Dm;
  }

  void Update( Eigen::MatrixXd & qd_m  ,
               Eigen::MatrixXd & qd    ,
               Eigen::MatrixXd & q_m   ,
               Eigen::MatrixXd & q     ,
               Eigen::MatrixXd & qdd_m ,
               Eigen::MatrixXd & t_r    )
  {

//      q_m   = q_m + delT*qd_m;
//      qd_m  = qd_m + delT*qdd_m;
//      qdd_m = MmInv*( t_r - Dm*qd_m - Km*q_m );

      ode_init_x[14] = t_r(0);
      ode_init_x[15] = t_r(1);
      ode_init_x[16] = t_r(2);
      ode_init_x[17] = t_r(3);
      ode_init_x[18] = t_r(4);
      ode_init_x[19] = t_r(5);
      ode_init_x[20] = t_r(6);

      boost::numeric::odeint::integrate( mass_spring_damper_model , ode_init_x , 0.0 , delT , delT );

      q_m(0)   = ode_init_x[0 ] ;
      q_m(1)   = ode_init_x[1 ] ;
      q_m(2)   = ode_init_x[2 ] ;
      q_m(3)   = ode_init_x[3 ] ;
      q_m(4)   = ode_init_x[4 ] ;
      q_m(5)   = ode_init_x[5 ] ;
      q_m(6)   = ode_init_x[6 ] ;

      qd_m(0)  = ode_init_x[7 ] ;
      qd_m(1)  = ode_init_x[8 ] ;
      qd_m(2)  = ode_init_x[9 ] ;
      qd_m(3)  = ode_init_x[10] ;
      qd_m(4)  = ode_init_x[11] ;
      qd_m(5)  = ode_init_x[12] ;
      qd_m(6)  = ode_init_x[13] ;

      qdd_m(0) = ode_init_x[14] ;
      qdd_m(1) = ode_init_x[15] ;
      qdd_m(2) = ode_init_x[16] ;
      qdd_m(3) = ode_init_x[17] ;
      qdd_m(4) = ode_init_x[18] ;
      qdd_m(5) = ode_init_x[19] ;
      qdd_m(6) = ode_init_x[20] ;
  }

};

/*
class FirModel
{

  double num_Joints; // number of joints.

  Eigen::MatrixXd q;
  Eigen::MatrixXd qd;
  Eigen::MatrixXd qdd;

  Eigen::MatrixXd q_m;
  Eigen::MatrixXd qd_m;
  Eigen::MatrixXd qdd_m;
  Eigen::MatrixXd t_r;

  double delT; // Time step

  state_type ode_init_x;

public:
  FirModel()
  {
    num_Joints = 7;
    delT = 0.001; /// 1000 Hz by default

    init();
  }
  ~FirModel()
  {
  }

  void changeModelstructure(double para_num_Joints)
  {
    num_Joints = para_num_Joints;
  }

  void init( )
  {
    q     .resize( num_Joints, 1 ) ;
    qd    .resize( num_Joints, 1 ) ;
    qdd   .resize( num_Joints, 1 ) ;

    q_m   .resize( num_Joints, 1 ) ;
    qd_m  .resize( num_Joints, 1 ) ;
    qdd_m .resize( num_Joints, 1 ) ;
    t_r   .resize( num_Joints, 1 ) ;

    q_m   << 0, 0, 0, 0, 0, 0, 0 ;
    qd_m  << 0, 0, 0, 0, 0, 0, 0 ;
    qdd_m << 0, 0, 0, 0, 0, 0, 0 ;

    t_r   << 0, 0, 0, 0, 0, 0, 0 ;

    // initial conditions
    ode_init_x[0 ] = 0.0;
    ode_init_x[1 ] = 0.0;
    ode_init_x[2 ] = 0.0;
    ode_init_x[3 ] = 0.0;
    ode_init_x[4 ] = 0.0;
    ode_init_x[5 ] = 0.0;
    ode_init_x[6 ] = 0.0;

    ode_init_x[7 ] = 0.0;
    ode_init_x[8 ] = 0.0;
    ode_init_x[9 ] = 0.0;
    ode_init_x[10] = 0.0;
    ode_init_x[11] = 0.0;
    ode_init_x[12] = 0.0;
    ode_init_x[13] = 0.0;

    ode_init_x[14] = 0.0;
    ode_init_x[15] = 0.0;
    ode_init_x[16] = 0.0;
    ode_init_x[17] = 0.0;
    ode_init_x[18] = 0.0;
    ode_init_x[19] = 0.0;
    ode_init_x[20] = 0.0;
  }

  void updateDelT(double p_delT)
  {
    delT = p_delT;
  }

  void Update( Eigen::MatrixXd & qd_m  ,
               Eigen::MatrixXd & qd    ,
               Eigen::MatrixXd & q_m   ,
               Eigen::MatrixXd & q     ,
               Eigen::MatrixXd & qdd_m ,
               Eigen::MatrixXd & t_r    )
  {

//      q_m   = q_m + delT*qd_m;
//      qd_m  = qd_m + delT*qdd_m;
//      qdd_m = MmInv*( t_r - Dm*qd_m - Km*q_m );

      ode_init_x[14] = t_r(0);
      ode_init_x[15] = t_r(1);
      ode_init_x[16] = t_r(2);
      ode_init_x[17] = t_r(3);
      ode_init_x[18] = t_r(4);
      ode_init_x[19] = t_r(5);
      ode_init_x[20] = t_r(6);

      //boost::numeric::odeint::integrate( mass_spring_damper_model , ode_init_x , 0.0 , delT , delT );

      q_m(0)   = ode_init_x[0 ] ;
      q_m(1)   = ode_init_x[1 ] ;
      q_m(2)   = ode_init_x[2 ] ;
      q_m(3)   = ode_init_x[3 ] ;
      q_m(4)   = ode_init_x[4 ] ;
      q_m(5)   = ode_init_x[5 ] ;
      q_m(6)   = ode_init_x[6 ] ;

      qd_m(0)  = ode_init_x[7 ] ;
      qd_m(1)  = ode_init_x[8 ] ;
      qd_m(2)  = ode_init_x[9 ] ;
      qd_m(3)  = ode_init_x[10] ;
      qd_m(4)  = ode_init_x[11] ;
      qd_m(5)  = ode_init_x[12] ;
      qd_m(6)  = ode_init_x[13] ;

      qdd_m(0) = ode_init_x[14] ;
      qdd_m(1) = ode_init_x[15] ;
      qdd_m(2) = ode_init_x[16] ;
      qdd_m(3) = ode_init_x[17] ;
      qdd_m(4) = ode_init_x[18] ;
      qdd_m(5) = ode_init_x[19] ;
      qdd_m(6) = ode_init_x[20] ;
  }
};
*/

}
}

#endif /* RLS_OUTER_LOOP_H_ */
