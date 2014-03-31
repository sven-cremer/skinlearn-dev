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
      double a = 10;
      double m = a*a;
      double d = 2*a;
      double k = a*a;

      dxdt[0 ] = x[1 ];

      //             f_r      qd_m      q_m
      dxdt[1 ] = m*( x[2] - d*x[1 ] - k*x[0 ] );

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

  // Task model
  double m ;
  double d ;
  double k ;

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
    m = p_m; // mass
    k = p_k; // spring
    d = p_d; // damper

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

    x         = Eigen::MatrixXd::Zero( num_Joints, 1 );
    xd        = Eigen::MatrixXd::Zero( num_Joints, 1 );
    xdd       = Eigen::MatrixXd::Zero( num_Joints, 1 );

    x_m       = Eigen::MatrixXd::Zero( num_Joints, 1 );
    xd_m      = Eigen::MatrixXd::Zero( num_Joints, 1 );
    xdd_m     = Eigen::MatrixXd::Zero( num_Joints, 1 );

    f_r       = Eigen::MatrixXd::Zero( num_Joints, 1 );

    // initial conditions
    ode_init_x[0 ] = x0  ;
    ode_init_x[1 ] = xd0 ;
    ode_init_x[2 ] = xdd0;
    ode_init_x[3 ] = m   ; // mass
    ode_init_x[4 ] = k   ; // spring
    ode_init_x[5 ] = d   ; // damper

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
  void update( double & param_xd_m  ,
               double & param_xd    ,
               double & param_x_m   ,
               double & param_x     ,
               double & param_xdd_m ,
               double   param_f_r    )
  {
    xd_m  (0) = param_xd_m ;
    xd    (0) = param_xd   ;
    x_m   (0) = param_x_m  ;
    x     (0) = param_x    ;
    xdd_m (0) = param_xdd_m;
    f_r   (0) = param_f_r  ;

    update();

    param_xd_m  = xd_m  (0);
    param_xd    = xd    (0);
    param_x_m   = x_m   (0);
    param_x     = x     (0);
    param_xdd_m = xdd_m (0);
    param_f_r   = f_r   (0);

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
  }

  void update()
  {

    ode_init_x[2] = f_r(0);

    boost::numeric::odeint::integrate( oneDmsd_model , ode_init_x , 0.0 , delT , delT );

    x_m  (0) = ode_init_x[0 ] ;
    xd_m (0) = ode_init_x[1 ] ;
    xdd_m(0) = m*( ode_init_x[2] - d*ode_init_x[1 ] - k*ode_init_x[0 ] );

  }
};



class FirModel
{

  int num_Joints; // number of joints.
  int num_Fir   ; // number of FIR parameters.

  Eigen::MatrixXd q;
  Eigen::MatrixXd qd;
  Eigen::MatrixXd qdd;

  Eigen::MatrixXd q_m;
  Eigen::MatrixXd qd_m;
  Eigen::MatrixXd qdd_m;

  // Reference task model
  Eigen::MatrixXd ref_q_m;
  Eigen::MatrixXd ref_qd_m;
  Eigen::MatrixXd ref_qdd_m;

  Eigen::MatrixXd task_ref;

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

  fir_state_type ode_init_x;

  oel::ls::RLSFilter rls_filter;

  void stackFirIn( Eigen::MatrixXd & in )
  {
    // TODO parameterize this
    // Moves top to bottom rows are time series, columns are joints
    // First in First out bottom most location nth row is dumped
    Uk_plus.block<1,1>(0,0) = in.transpose();
    Uk_plus.block<8-1, 1>(1,0) = Uk.block<8-1, 1>(0,0);
    Uk = Uk_plus;
  }

public:
  FirModel()
  {
    delT = 0.001; /// 1000 Hz by default
    iter = 1;

    double a = 10;

    //          m    d    k
    init( 1, 8, a*a, 2*a, a*a );
  }
  ~FirModel()
  {
  }

  void init( int para_num_Joints, int para_num_Fir, double p_m, double p_d, double p_k )
  {
    m = p_m;
    d = p_d;
    k = p_k;

    num_Fir    = para_num_Fir;
    num_Joints = para_num_Joints;

    q     .resize( num_Joints, 1 ) ;
    qd    .resize( num_Joints, 1 ) ;
    qdd   .resize( num_Joints, 1 ) ;

    q_m   .resize( num_Joints, 1 ) ;
    qd_m  .resize( num_Joints, 1 ) ;
    qdd_m .resize( num_Joints, 1 ) ;

    ref_q_m   .resize( num_Joints, 1 ) ;
    ref_qd_m  .resize( num_Joints, 1 ) ;
    ref_qdd_m .resize( num_Joints, 1 ) ;

    task_ref  .resize( num_Joints, 1 ) ;

    t_r   .resize( num_Joints, 1 ) ;

    Wk    .resize( num_Fir, num_Joints ) ;
    Wk = Eigen::MatrixXd::Zero( num_Fir, num_Joints );

    Dk    .resize( num_Joints, 1 ) ;
    Dk = Eigen::MatrixXd::Zero( num_Joints, 1 );

    // FIXME need to make this a 3 dimensional matrix
    Pk    .resize( num_Fir, num_Fir       ) ;
    Pk = Eigen::MatrixXd::Identity( num_Fir, num_Fir )/0.0001;

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

    ref_q_m   = Eigen::MatrixXd::Zero( num_Joints, 1 );
    ref_qd_m  = Eigen::MatrixXd::Zero( num_Joints, 1 );
    ref_qdd_m = Eigen::MatrixXd::Zero( num_Joints, 1 );

    task_ref   = Eigen::MatrixXd::Zero( num_Joints, 1 );

    t_r       = Eigen::MatrixXd::Zero( num_Joints, 1 );

    lm = 0.98; // Forgetting factor

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

  void Update( double & param_qd_m  ,
               double & param_qd    ,
               double & param_q_m   ,
               double & param_q     ,
               double & param_qdd_m ,
               double & param_t_r    )
  {
    qd_m  (0)= param_qd_m ;
    qd    (0)= param_qd   ;
    q_m   (0)= param_q_m  ;
    q     (0)= param_q    ;
    qdd_m (0)= param_qdd_m;
    t_r   (0)= param_t_r  ;

    Update();
  }

  void Update( double & param_qd_m     ,
               double & param_qd       ,
               double & param_q_m      ,
               double & param_q        ,
               double & param_qdd_m    ,
               double & param_t_r      ,
               double & param_task_ref  )
  {
    qd_m    (0) = param_qd_m ;
    qd      (0) = param_qd   ;
    q_m     (0) = param_q_m  ;
    q       (0) = param_q    ;
    qdd_m   (0) = param_qdd_m;
    t_r     (0) = param_t_r  ;
    task_ref(0) = param_task_ref;

    Update();
  }

  void Update( Eigen::MatrixXd & param_qd_m    ,
               Eigen::MatrixXd & param_qd      ,
               Eigen::MatrixXd & param_q_m     ,
               Eigen::MatrixXd & param_q       ,
               Eigen::MatrixXd & param_qdd_m   ,
               Eigen::MatrixXd & param_t_r     ,
               Eigen::MatrixXd & param_task_ref )
  {
    qd_m     = param_qd_m    ;
    qd       = param_qd      ;
    q_m      = param_q_m     ;
    q        = param_q       ;
    qdd_m    = param_qdd_m   ;
    t_r      = param_t_r     ;
    task_ref = param_task_ref;
    Update();
  }

  void Update()
  {
    // Save input forces/torques
    stackFirIn( t_r );

    ode_init_x[2] = t_r(0);

    boost::numeric::odeint::integrate( task_model , ode_init_x , 0.0 , delT , delT );

    ref_q_m(0)   = ode_init_x[0 ] ;
    ref_qd_m(0)  = ode_init_x[1 ] ;
    ref_qdd_m(0) = m*( task_ref(0) - d*ode_init_x[1 ] - k*ode_init_x[0 ] );

    // Save iteration number
    iter = iter + 1;

    // Desired is the task reference model
    Dk = ref_q_m;

    if( iter > num_Fir )
    {
      rls_filter.Update( Wk, Uk, Dk, Pk );

      Wk = rls_filter.getEstimate();
      Pk = rls_filter.getCovariance();

      q_m   = Uk.transpose()*Wk  ;

      // Backward difference
      // TODO better way to do this?
      qd_m  = (q_m  - q_m )/delT ;
      qdd_m = (qd_m - qd_m)/delT ;
    }

//      std::cout<< "Uk : " << Uk.transpose() <<"\n\n";
//      std::cout<< "q  : " << q_m <<"\n\n";
  }
};



}
}

#endif /* RLS_OUTER_LOOP_H_ */
