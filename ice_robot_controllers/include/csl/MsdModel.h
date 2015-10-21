/*
 * MsdModel.h
 *
 *  Created on: Oct 30, 2014
 *      Author: sven
 */

#ifndef MSDMODEL_H_
#define MSDMODEL_H_

namespace csl
{
namespace outer_loop
{
class MsdModel
{

  int num_Joints; // number of joints.

  Eigen::MatrixXd x;
  Eigen::MatrixXd xd;
  Eigen::MatrixXd xdd;

  Eigen::MatrixXd x_m;
  Eigen::MatrixXd xd_m;
  Eigen::MatrixXd xdd_m;

  // Reference task model
  Eigen::MatrixXd ref_q_m;
  Eigen::MatrixXd ref_qd_m;
  Eigen::MatrixXd ref_qdd_m;

  Eigen::MatrixXd task_ref;

  Eigen::MatrixXd f_r;

  double delT; // Time step

  double lm;

  int iter;

  // Admittance model
  double m_M ;
  double m_D ;
  double m_S ;

  oneDmsd_state_type ode_init_x;

public:
  MsdModel()
  {
    delT = 0.001; /// 1000 Hz by default
    num_Joints = 1;

    //    m  s    d
    init( 1, 1,   10, 0, 0, 0 );
  }
  ~MsdModel()
  {
  }

  void init( double p_m, double p_k, double p_d, double x0, double xd0, double xdd0 )
  {
    m_M = p_m; // mass
    m_S = p_k; // spring
    m_D = p_d; // damper

    x     .resize( num_Joints, 1 ) ;
    xd    .resize( num_Joints, 1 ) ;
    xdd   .resize( num_Joints, 1 ) ;

    x_m   .resize( num_Joints, 1 ) ;
    xd_m  .resize( num_Joints, 1 ) ;
    xdd_m .resize( num_Joints, 1 ) ;

    ref_q_m   .resize( num_Joints, 1 ) ;
    ref_qd_m  .resize( num_Joints, 1 ) ;
    ref_qdd_m .resize( num_Joints, 1 ) ;

    task_ref  .resize( num_Joints, 1 ) ;

    f_r   .resize( num_Joints, 1 ) ;

    x     = Eigen::MatrixXd::Zero( num_Joints, 1 );
    xd    = Eigen::MatrixXd::Zero( num_Joints, 1 );
    xdd   = Eigen::MatrixXd::Zero( num_Joints, 1 );

    x_m   = Eigen::MatrixXd::Zero( num_Joints, 1 );
    xd_m  = Eigen::MatrixXd::Zero( num_Joints, 1 );
    xdd_m = Eigen::MatrixXd::Zero( num_Joints, 1 );

    f_r   = Eigen::MatrixXd::Zero( num_Joints, 1 );

    // initial conditions
    ode_init_x[0 ] = x0  ;
    ode_init_x[1 ] = xd0 ;
    ode_init_x[2 ] = xdd0;
    ode_init_x[3 ] = m_M   ; // mass
    ode_init_x[4 ] = m_S   ; // spring
    ode_init_x[5 ] = m_D   ; // damper

  }

  void updateDelT(double p_delT)
  {
    delT = p_delT;
  }

  void updateMsd( double & param_m_M,
                  double & param_m_S,
                  double & param_m_D )
  {
    m_M = param_m_M ; // mass
    m_S = param_m_S ; // spring
    m_D = param_m_D ; // damper
  }

  void update( double & param_xd_m  ,
               double & param_xd    ,
               double & param_x_m   ,
               double & param_x     ,
               double & param_xdd_m ,
               double & param_f_r    )
  {
    xd_m  (0) = param_xd_m ;
    xd    (0) = param_xd   ;
    x_m   (0) = param_x_m  ;
    x     (0) = param_x    ;
    xdd_m (0) = param_xdd_m;
    f_r   (0) = param_f_r  ;

    update();

    param_xd_m  = xd_m  (0);
//    param_xd    = xd    (0);
    param_x_m   = x_m   (0);
//    param_x     = x     (0);
    param_xdd_m = xdd_m (0);
//    param_f_r   = f_r   (0);
  }

  void update( Eigen::MatrixXd & param_xd_m    ,
               Eigen::MatrixXd & param_xd      ,
               Eigen::MatrixXd & param_x_m     ,
               Eigen::MatrixXd & param_x       ,
               Eigen::MatrixXd & param_xdd_m   ,
               Eigen::MatrixXd & param_f_r      )
  {
    xd_m     = param_xd_m    ;
    xd       = param_xd      ;
    x_m      = param_x_m     ;
    x        = param_x       ;
    xdd_m    = param_xdd_m   ;
    f_r      = param_f_r     ;

    update();

    param_xd_m  = xd_m ;
//    param_xd    = xd   ;
    param_x_m   = x_m  ;
//    param_x     = x    ;
    param_xdd_m = xdd_m;
//    param_f_r   = f_r  ;

  }

  void update()
  {

//    ode_init_x[2] = f_r(0);
//
//    boost::numeric::odeint::integrate( oneDmsd_model , ode_init_x , 0.0 , delT , delT );
//
//    x_m  (0) = ode_init_x[0 ] ;
//    xd_m (0) = ode_init_x[1 ] ;
//    xdd_m(0) = m*( ode_init_x[2] - d*ode_init_x[1 ] - k*ode_init_x[0 ] );

    x_m   = x_m  + xd_m *delT ;
    xd_m  = xd_m + xdd_m*delT ;
    xdd_m = ( f_r - m_D*xd_m - m_S*x_m )/m_M;

  }
};
}
}

#endif /* MSDMODEL_H_ */
