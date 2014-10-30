/*
 * JSpaceMsdModel.h
 *
 *  Created on: Oct 30, 2014
 *      Author: sven
 */

#ifndef JSPACEMSDMODEL_H_
#define JSPACEMSDMODEL_H_

namespace csl
{
namespace outer_loop
{
class JSpaceMsdModel
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

  double m ;
  double d ;
  double k ;

  state_type ode_init_x;


public:
  JSpaceMsdModel()
  {
    num_Joints = 7;
    delT = 0.001; /// 1000 Hz by default

    init( 1, 10, 1 );
  }
  ~JSpaceMsdModel()
  {
  }

  void changeModelstructure(double para_num_Joints)
  {
    num_Joints = para_num_Joints;
  }

  void init( double p_m, double p_d, double p_k)
  {
    m = p_m;
    d = p_d;
    k = p_k;

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

    q         = Eigen::MatrixXd::Zero( num_Joints, 1 );
    qd        = Eigen::MatrixXd::Zero( num_Joints, 1 );
    qdd       = Eigen::MatrixXd::Zero( num_Joints, 1 );

    q_m       = Eigen::MatrixXd::Zero( num_Joints, 1 );
    qd_m      = Eigen::MatrixXd::Zero( num_Joints, 1 );
    qdd_m     = Eigen::MatrixXd::Zero( num_Joints, 1 );

    t_r       = Eigen::MatrixXd::Zero( num_Joints, 1 );

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

      qdd_m(0) = m*( t_r(0) - d*ode_init_x[7 ] - k*ode_init_x[0 ] );
      qdd_m(1) = m*( t_r(1) - d*ode_init_x[8 ] - k*ode_init_x[1 ] );
      qdd_m(2) = m*( t_r(2) - d*ode_init_x[9 ] - k*ode_init_x[2 ] );
      qdd_m(3) = m*( t_r(3) - d*ode_init_x[10] - k*ode_init_x[3 ] );
      qdd_m(4) = m*( t_r(4) - d*ode_init_x[11] - k*ode_init_x[4 ] );
      qdd_m(5) = m*( t_r(5) - d*ode_init_x[12] - k*ode_init_x[5 ] );
      qdd_m(6) = m*( t_r(6) - d*ode_init_x[13] - k*ode_init_x[6 ] );
  }

};
}
}

#endif /* JSPACEMSDMODEL_H_ */
