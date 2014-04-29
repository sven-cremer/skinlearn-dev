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


class RlsModel
{

  int num_Joints; // number of joints.
  int num_Fir   ; // number of FIR parameters.

  Eigen::MatrixXd q;
  Eigen::MatrixXd qd;
  Eigen::MatrixXd qdd;

  Eigen::MatrixXd q_m;
  Eigen::MatrixXd qd_m;
  Eigen::MatrixXd qdd_m;

  Eigen::MatrixXd prv_q_m;
  Eigen::MatrixXd prv_qd_m;

  // Reference task model
  Eigen::MatrixXd ref_q_m;
  Eigen::MatrixXd ref_qd_m;
  Eigen::MatrixXd ref_qdd_m;

  Eigen::MatrixXd task_ref;
  Eigen::MatrixXd task_ref_model;

  Eigen::MatrixXd t_r;

  Eigen::MatrixXd Wk           ; // FIR weights
  Eigen::MatrixXd Uk           ; // Input
  Eigen::MatrixXd Uk_plus      ; // FIR inputs time series f_r temp to use for update
  Eigen::MatrixXd Dk           ; // Desired
  Eigen::MatrixXd Pk           ; // Covariance matrix

  double delT; // Time step

  double lm;

  int iter;

  // Task model
  double m ;
  double d ;
  double k ;

  // 1st order model
  double a_task ;
  double b_task ;

  fir_state_type ode_init_x;

  oel::ls::RLSFilter rls_filter;

  // Switch RLS on/off
  bool useFixedWeights ;

  void stackFirIn( Eigen::MatrixXd & u_in )
  {
    // TODO parameterize this
    // Moves top to bottom rows are time series, columns are joints
    // First in First out bottom most location nth row is dumped
//    Uk_plus.block<8-1, 1>(1,0) = Uk.block<8-1, 1>(0,0);
//    Uk_plus.block<1,1>(0,0) = u_in.transpose();

    Uk_plus.block<8-1, 1>(1,0) = Uk.block<8-1, 1>(0,0);
    Uk_plus.block<1,1>(0,0) = u_in.transpose();

    Uk = Uk_plus;
  }

  void stackArmaIn( Eigen::MatrixXd & y_prev, Eigen::MatrixXd & u_in )
  {
    // TODO parameterize this
    // Moves top to bottom rows are time series, columns are joints
    // First in First out bottom most location nth row is dumped
    Uk_plus.block<4-1, 1>(1,0) = Uk.block<4-1, 1>(0,0);
    Uk_plus.block<1,1>(0,0) = - y_prev.transpose();

    Uk_plus.block<4-1, 1>(5,0) = Uk.block<4-1, 1>(4,0);
    Uk_plus.block<1,1>(4,0) = u_in.transpose();

    Uk = Uk_plus;
  }

public:
  RlsModel()
  {
    delT = 0.001; /// 1000 Hz by default
    iter = 1;

    a_task = 0.5 ;
    b_task = 0.5 ;

    //
    init( 1, 8 );
  }
  ~RlsModel()
  {
  }

  void init( int para_num_Joints, int para_num_Fir )
  {

    num_Fir    = para_num_Fir;
    num_Joints = para_num_Joints;

    q         .resize( num_Joints, 1 ) ;
    qd        .resize( num_Joints, 1 ) ;
    qdd       .resize( num_Joints, 1 ) ;

    q_m       .resize( num_Joints, 1 ) ;
    qd_m      .resize( num_Joints, 1 ) ;
    qdd_m     .resize( num_Joints, 1 ) ;

    prv_q_m   .resize( num_Joints, 1 ) ;
    prv_qd_m  .resize( num_Joints, 1 ) ;

    ref_q_m   .resize( num_Joints, 1 ) ;
    ref_qd_m  .resize( num_Joints, 1 ) ;
    ref_qdd_m .resize( num_Joints, 1 ) ;

    task_ref      .resize( num_Joints, 1 ) ;

    t_r   .resize( num_Joints, 1 ) ;

    Wk    .resize( num_Fir, num_Joints ) ;
    Wk = Eigen::MatrixXd::Zero( num_Fir, num_Joints );

    Dk    .resize( num_Joints, 1 ) ;
    Dk = Eigen::MatrixXd::Zero( num_Joints, 1 );

    // FIXME need to make this a 3 dimensional matrix
    Pk    .resize( num_Fir, num_Fir       ) ;
    Pk = Eigen::MatrixXd::Identity( num_Fir, num_Fir )/0.001;

    Uk.resize( num_Fir, num_Joints ) ;
    Uk = Eigen::MatrixXd::Zero( num_Fir, num_Joints );

    Uk_plus.resize( num_Fir, num_Joints ) ;
    Uk_plus = Eigen::MatrixXd::Zero( num_Fir, num_Joints );

    q         = Eigen::MatrixXd::Zero( num_Joints, 1 );
    qd        = Eigen::MatrixXd::Zero( num_Joints, 1 );
    qdd       = Eigen::MatrixXd::Zero( num_Joints, 1 );

    q_m       = Eigen::MatrixXd::Zero( num_Joints, 1 );
    qd_m      = Eigen::MatrixXd::Zero( num_Joints, 1 );
    qdd_m     = Eigen::MatrixXd::Zero( num_Joints, 1 );

    prv_q_m   = Eigen::MatrixXd::Zero( num_Joints, 1 );
    prv_qd_m  = Eigen::MatrixXd::Zero( num_Joints, 1 );

    ref_q_m   = Eigen::MatrixXd::Zero( num_Joints, 1 );
    ref_qd_m  = Eigen::MatrixXd::Zero( num_Joints, 1 );
    ref_qdd_m = Eigen::MatrixXd::Zero( num_Joints, 1 );

    task_ref  = Eigen::MatrixXd::Zero( num_Joints, 1 );

    t_r       = Eigen::MatrixXd::Zero( num_Joints, 1 );

    lm        = 0.98; // Forgetting factor

    // Don't use fixed weights by default
    useFixedWeights = false;

    // initial conditions
    ode_init_x[0 ] = 0.0;
    ode_init_x[1 ] = 0.0;
    ode_init_x[2 ] = 0.0;

    rls_filter.init( Wk, Uk, Dk, Pk, lm );

  }

  void updateDelT(double p_delT)
  {
    delT = p_delT;
  }

  void updateMsd( double param_m_M,
                  double param_m_S,
                  double param_m_D )
  {
    m = param_m_M ; // mass
    k = param_m_S ; // spring
    d = param_m_D ; // damper
  }

  void updateAB( double param_a_task,
                 double param_b_task )
  {
    a_task = param_a_task ;
    b_task = param_b_task ;
  }

  void getWeights( Eigen::MatrixXd & param_Wk )
  {
	  param_Wk = Wk ;
  }

  void setWeights( Eigen::MatrixXd & param_Wk )
  {
    Wk = param_Wk;
  }

  void setFixedWeights( Eigen::MatrixXd & param_Wk )
  {
	Wk = param_Wk;
	useFixedWeights = true;
  }

  void updateFIR( double & param_qd_m           ,
                  double & param_qd             ,
                  double & param_q_m            ,
                  double & param_q              ,
                  double & param_qdd_m          ,
                  double & param_t_r            ,
                  double & param_task_ref       ,
                  double & param_task_ref_model  )
  {
    qd_m    (0)          = param_qd_m     ;
    qd      (0)          = param_qd       ;
    q_m     (0)          = param_q_m      ;
    q       (0)          = param_q        ;
    qdd_m   (0)          = param_qdd_m    ;
    t_r     (0)          = param_t_r      ;
    task_ref(0)          = param_task_ref ;

    // Save input forces/torques
    stackFirIn( t_r );
    update();

    param_task_ref_model = ref_q_m(0)     ;
    param_q_m            = q_m(0)         ;
    param_qd_m           = qd_m(0)        ;
    param_qdd_m          = qdd_m(0)       ;
  }

  void updateFIR( Eigen::MatrixXd & param_qd_m          ,
                  Eigen::MatrixXd & param_qd            ,
                  Eigen::MatrixXd & param_q_m           ,
                  Eigen::MatrixXd & param_q             ,
                  Eigen::MatrixXd & param_qdd_m         ,
                  Eigen::MatrixXd & param_t_r           ,
                  Eigen::MatrixXd & param_task_ref      ,
                  Eigen::MatrixXd & param_task_ref_model )
  {
    qd_m                 = param_qd_m     ;
    qd                   = param_qd       ;
    q_m                  = param_q_m      ;
    q                    = param_q        ;
    qdd_m                = param_qdd_m    ;
    t_r                  = param_t_r      ;
    task_ref             = param_task_ref ;

    // Save input forces/torques
    stackFirIn( t_r );
    update();

    param_task_ref_model = ref_q_m        ;
    param_q_m            = q_m            ;
    param_qd_m           = qd_m           ;
    param_qdd_m          = qdd_m          ;
  }

  void updateARMA( double & param_qd_m           ,
                   double & param_qd             ,
                   double & param_q_m            ,
                   double & param_q              ,
                   double & param_qdd_m          ,
                   double & param_t_r            ,
                   double & param_task_ref       ,
                   double & param_task_ref_model  )
  {
    qd_m    (0)          = param_qd_m     ;
    qd      (0)          = param_qd       ;
    q_m     (0)          = param_q_m      ;
    q       (0)          = param_q        ;
    qdd_m   (0)          = param_qdd_m    ;
    t_r     (0)          = param_t_r      ;
    task_ref(0)          = param_task_ref ;

    // Save input forces/torques
    stackArmaIn( q_m, t_r );
    update();

    param_task_ref_model = ref_q_m(0)     ;
    param_q_m            = q_m(0)         ;
    param_qd_m           = qd_m(0)        ;
    param_qdd_m          = qdd_m(0)       ;
  }

  void updateARMA( Eigen::MatrixXd & param_qd_m          ,
                   Eigen::MatrixXd & param_qd            ,
                   Eigen::MatrixXd & param_q_m           ,
                   Eigen::MatrixXd & param_q             ,
                   Eigen::MatrixXd & param_qdd_m         ,
                   Eigen::MatrixXd & param_t_r           ,
                   Eigen::MatrixXd & param_task_ref      ,
                   Eigen::MatrixXd & param_task_ref_model )
  {
    qd_m                 = param_qd_m     ;
    qd                   = param_qd       ;
    q_m                  = param_q_m      ;
    q                    = param_q        ;
    qdd_m                = param_qdd_m    ;
    t_r                  = param_t_r      ;
    task_ref             = param_task_ref ;

    // Save input forces/torques
    stackArmaIn( q_m, t_r );
    update();

    param_task_ref_model = ref_q_m        ;
    param_q_m            = q_m            ;
    param_qd_m           = qd_m           ;
    param_qdd_m          = qdd_m          ;
  }

  void update()
  {
    ode_init_x[2] = task_ref(0);

//    boost::numeric::odeint::integrate( task_model , ode_init_x , 0.0 , delT , delT );

    ref_q_m(0)   = ref_q_m(0) + ref_qd_m(0)*delT;
    ref_qd_m(0)  = a_task*task_ref(0) - b_task*ref_q_m(0);

    ref_qdd_m(0) = 0; //m*( task_ref(0) - d*ode_init_x[1 ] - k*ode_init_x[0 ] );

//    ref_q_m(0)   = ode_init_x[0 ] ;
//    ref_qd_m(0)  = ode_init_x[1 ] ;
//    ref_qdd_m(0) = 0; //m*( task_ref(0) - d*ode_init_x[1 ] - k*ode_init_x[0 ] );

    // Save iteration number
    iter = iter + 1;

    // Desired is the task reference model
    Dk = ref_q_m;

    //if( iter > num_Fir )
    {
      if( !useFixedWeights )
      {
		  rls_filter.Update( Wk, Uk, Dk, Pk );

		  Wk = rls_filter.getEstimate();
		  Pk = rls_filter.getCovariance();
      }

      q_m   = Uk.transpose()*Wk  ;

      // Backward difference
      // TODO better way to do this?
      qd_m  = (q_m  - prv_q_m )/delT ;
      qdd_m = (qd_m - prv_qd_m)/delT ;
    }

    prv_q_m  = q_m ;
    prv_qd_m = qd_m;

  }
};


class MracModel
{

  int num_Joints                 ; // number of joints.
  int num_Fir                    ; // number of FIR parameters.

  Eigen::MatrixXd q              ;
  Eigen::MatrixXd qd             ;
  Eigen::MatrixXd qdd            ;

  Eigen::MatrixXd q_m            ;
  Eigen::MatrixXd qd_m           ;
  Eigen::MatrixXd qdd_m          ;

  Eigen::MatrixXd prv_q_m        ;
  Eigen::MatrixXd prv_qd_m       ;

  // Reference task model
  Eigen::MatrixXd ref_q_m        ;
  Eigen::MatrixXd ref_qd_m       ;
  Eigen::MatrixXd ref_qdd_m      ;

  Eigen::MatrixXd task_ref       ;
  Eigen::MatrixXd task_ref_model ;

  Eigen::MatrixXd t_r   ;

  double a              ;
  double b              ;
  double am             ;
  double bm             ;
  double an             ;
  double bn             ;

  double gamma_1        ;
  double gamma_2        ;
  double gamma_3        ;
  double gamma_4        ;
  double gamma_5        ;


  double u_c            ;
  double u              ;
  double e              ;

  double ym             ;
  double yp             ;
  double y              ;
  double y_hat          ;
  double y_tilde        ;

  double theta_1        ;
  double theta_2        ;
  double theta_3        ;
  double ahat           ;
  double bhat           ;


  double ym_dot         ;
  double yp_dot         ;
  double y_dot          ;
  double yhat_dot       ;

  double theta_1_dot    ;
  double theta_2_dot    ;
  double theta_3_dot    ;
  double ahat_dot       ;
  double bhat_dot       ;

  // Time step
  double delT           ;
  double lm             ;
  int    iter           ;

  // Task model

  // 2nd order model
  double m      ;
  double d      ;
  double k      ;

  // 1st order model
  double a_task ;
  double b_task ;

public:
  MracModel()
  {
    delT = 0.001 ; /// 1000 Hz by default
    iter = 1     ;

    a_task = 0.5 ;
    b_task = 0.5 ;

    //
    init( 1 );
  }
  ~MracModel()
  {
  }

  void init( int para_num_Joints )
  {

    // Transfer Functions
    a  = 1  ; b  = 0.5 ;
    am = 1  ; bm = 1   ;
    an = 1  ; bn = 1   ;

    // Intial Values
    theta_1  = 1 ; theta_2 = 1 ; theta_3 = 1 ;
    yhat_dot = 1 ; y_hat   = 1 ;
    yp       = 1 ; ym      = 1 ;

//    u_c = 1;

    // Gains
    gamma_1 = 1     ,
    gamma_2 = 2000  ,
    gamma_3 = 2.5e5 ,
    gamma_4 = 5000 ,
    gamma_5 = 5000  ;

    u = - theta_1 * yhat_dot - theta_2 * yp - theta_3 * y_hat;
    e = yp - ym;

    num_Joints = para_num_Joints;

    q         .resize( num_Joints, 1 ) ;
    qd        .resize( num_Joints, 1 ) ;
    qdd       .resize( num_Joints, 1 ) ;

    q_m       .resize( num_Joints, 1 ) ;
    qd_m      .resize( num_Joints, 1 ) ;
    qdd_m     .resize( num_Joints, 1 ) ;

    prv_q_m   .resize( num_Joints, 1 ) ;
    prv_qd_m  .resize( num_Joints, 1 ) ;

    ref_q_m   .resize( num_Joints, 1 ) ;
    ref_qd_m  .resize( num_Joints, 1 ) ;
    ref_qdd_m .resize( num_Joints, 1 ) ;

    task_ref  .resize( num_Joints, 1 ) ;

    t_r       .resize( num_Joints, 1 ) ;

    q         = Eigen::MatrixXd::Zero( num_Joints, 1 );
    qd        = Eigen::MatrixXd::Zero( num_Joints, 1 );
    qdd       = Eigen::MatrixXd::Zero( num_Joints, 1 );

    q_m       = Eigen::MatrixXd::Zero( num_Joints, 1 );
    qd_m      = Eigen::MatrixXd::Zero( num_Joints, 1 );
    qdd_m     = Eigen::MatrixXd::Zero( num_Joints, 1 );

    prv_q_m   = Eigen::MatrixXd::Zero( num_Joints, 1 );
    prv_qd_m  = Eigen::MatrixXd::Zero( num_Joints, 1 );

    ref_q_m   = Eigen::MatrixXd::Zero( num_Joints, 1 );
    ref_qd_m  = Eigen::MatrixXd::Zero( num_Joints, 1 );
    ref_qdd_m = Eigen::MatrixXd::Zero( num_Joints, 1 );

    task_ref  = Eigen::MatrixXd::Zero( num_Joints, 1 );

    t_r       = Eigen::MatrixXd::Zero( num_Joints, 1 );

  }

  void updateDelT(double p_delT)
  {
    delT = p_delT;
  }

  void updateMsd( double param_m_M,
                  double param_m_S,
                  double param_m_D )
  {
    m = param_m_M ; // mass
    k = param_m_S ; // spring
    d = param_m_D ; // damper
  }

  void updateAB( double param_a_task,
                 double param_b_task )
  {
    a_task = param_a_task ;
    b_task = param_b_task ;
  }

  void updateIni( double param_yp,
                  double param_ym )
  {
	yp     = param_yp ;
	ym     = param_ym ;
  }

  void update( double & param_qd_m           ,
               double & param_qd             ,
               double & param_q_m            ,
               double & param_q              ,
               double & param_qdd_m          ,
               double & param_t_r            ,
               double & param_task_ref       ,
               double & param_task_ref_model  )
  {
    qd_m    (0)       = param_qd_m ;
    qd      (0)       = param_qd   ;
    q_m     (0)       = param_q_m  ;
    q       (0)       = param_q    ;
    qdd_m   (0)       = param_qdd_m;
    t_r     (0)       = param_t_r  ;
    task_ref(0)       = param_task_ref;

    update();

    param_task_ref_model = ref_q_m(0) ;
    param_q_m            = q_m(0);
    param_qd_m           = qd_m(0);
    param_qdd_m          = qdd_m(0);
//    param_t_r            = t_r(0);
  }

  void update( Eigen::MatrixXd & param_qd_m          ,
               Eigen::MatrixXd & param_qd            ,
               Eigen::MatrixXd & param_q_m           ,
               Eigen::MatrixXd & param_q             ,
               Eigen::MatrixXd & param_qdd_m         ,
               Eigen::MatrixXd & param_t_r           ,
               Eigen::MatrixXd & param_task_ref      ,
               Eigen::MatrixXd & param_task_ref_model )
  {
    qd_m           = param_qd_m          ;
    qd             = param_qd            ;
    q_m            = param_q_m           ;
    q              = param_q             ;
    qdd_m          = param_qdd_m         ;
    t_r            = param_t_r           ;
    task_ref       = param_task_ref      ;

    update();

    param_task_ref_model = ref_q_m ;
    param_q_m            = q_m;
    param_qd_m           = qd_m;
    param_qdd_m          = qdd_m;
  }

  void update()
  {
//    ode_init_x[2] = task_ref(0);

//    boost::numeric::odeint::integrate( task_model , ode_init_x , 0.0 , delT , delT );

    ref_q_m(0)   = ref_q_m(0) + ref_qd_m(0)*delT;
    ref_qd_m(0)  = a_task*task_ref(0) -  b_task*ref_q_m(0);

    ref_qdd_m(0) = 0; //m*( task_ref(0) - d*ode_init_x[1 ] - k*ode_init_x[0 ] );

//    ref_q_m(0)   = ode_init_x[0 ] ;
//    ref_qd_m(0)  = ode_init_x[1 ] ;
//    ref_qdd_m(0) = 0; //m*( task_ref(0) - d*ode_init_x[1 ] - k*ode_init_x[0 ] );

    // Save iteration number
    iter = iter + 1;

    // Desired is the task reference model
    u_c = ref_q_m(0);

    {

      // Human force
      y = t_r(0) ;

      u           = - theta_1 * y_hat - theta_2 * yp - theta_3 * y     ;
      e           = yp - ym                                            ;
      y_tilde     = y - y_hat                                          ;

      // k + 1
      // dot
      ym_dot      = -am        * ym            + bm     * u_c          ;
      yp_dot      = -an        * yp            + bn     * u            ;
      // FIXME Fake Human Force
      // y_dot       = -a         * y             + b      * u_c       ;
      yhat_dot    = -ahat      * y_hat         + bhat   * u_c          ;
      theta_1_dot =  gamma_1   * e * u_c                               ;
      theta_2_dot =  gamma_2   * e * yp                                ;
      theta_3_dot =  gamma_3   * e * bn * y_hat + gamma_1 * ahat * u_c ;
      ahat_dot    = -1*gamma_4 * y_tilde * y_hat                       ;
      bhat_dot    =  gamma_5 * y_tilde * u_c                           ;

      // 1dt order integrator
      ym      = ym      + ym_dot      * delT ;
      yp      = yp      + yp_dot      * delT ;
      // FIXME Fake Human Force
      // y       = y       + y_dot       * delT ;
      y_hat   = y_hat   + yhat_dot    * delT ;
      theta_1 = theta_1 + theta_1_dot * delT ;
      theta_2 = theta_2 + theta_2_dot * delT ;
      theta_3 = theta_3 + theta_3_dot * delT ;
      ahat    = ahat    + ahat_dot    * delT ;
      bhat    = bhat    + bhat_dot    * delT ;

      // Model output
      q_m(0)   = ym ;

      // FIXME Fake Human Force
      // t_r(0) = y;

      // Backward difference
      // TODO better way to do this?
      qd_m  = (q_m  - prv_q_m )/delT ;
      qdd_m = (qd_m - prv_qd_m)/delT ;

    }

    prv_q_m  = q_m ;
    prv_qd_m = qd_m;

//      std::cout<< "Uk : " << Uk.transpose() <<"\n\n";
//      std::cout<< "q  : " << q_m <<"\n\n";
  }
};




}
}

#endif /* RLS_OUTER_LOOP_H_ */
