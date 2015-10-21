/*
 * CtRlsModel.h
 *
 *  Created on: Dec 10, 2014
 *      Author: Isura
 */

#ifndef CTRLSMODEL_H_
#define CTRLSMODEL_H_

namespace csl
{
namespace outer_loop
{
class CtRlsModel
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
  Eigen::MatrixXd Rk           ;

  double delT; // Time step

  double m_lm;
  double m_sigma;

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

  void stackIntegrate( Eigen::MatrixXd & x_d, Eigen::MatrixXd & f_h )
  {
    // TODO parameterize this
    Uk(0,0) = -(Uk(0,0) + x_d(0,0) * delT);
    Uk(1,0) = -(Uk(1,0) + Uk (0,0) * delT);
    Uk(2,0) = -(Uk(2,0) + Uk (1,0) * delT);
    Uk(3,0) = -(Uk(3,0) + Uk (2,0) * delT);

    Uk(4,0) =   Uk(4,0) + f_h(0,0) * delT ;
    Uk(5,0) =   Uk(5,0) + Uk (4,0) * delT ;
    Uk(6,0) =   Uk(6,0) + Uk (5,0) * delT ;
    Uk(7,0) =   Uk(7,0) + Uk (6,0) * delT ;
  }

public:
  CtRlsModel()
  {
    delT = 0.001; /// 1000 Hz by default
    iter = 1;

    a_task = 0.5 ;
    b_task = 0.5 ;

    //
    init( 1, 8 );
  }
  ~CtRlsModel()
  {
  }

  void init( int para_num_Joints, int para_num_Fir )
  {

    num_Fir    = para_num_Fir          ;
    num_Joints = para_num_Joints       ;

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

    t_r  .resize( num_Joints, 1 ) ;

    Rk   .resize( num_Fir, num_Joints ) ;
	Rk = Eigen::MatrixXd::Zero( num_Fir, num_Joints );

    Wk   .resize( num_Fir, num_Joints ) ;
    Wk = Eigen::MatrixXd::Zero( num_Fir, num_Joints );

    Dk   .resize( num_Joints, 1 ) ;
    Dk = Eigen::MatrixXd::Zero( num_Joints, 1 );

    // FIXME need to make this a 3 dimensional matrix
    Pk    .resize( num_Fir, num_Fir ) ;
    m_sigma = 0.001 ;
    Pk = Eigen::MatrixXd::Identity( num_Fir, num_Fir )/m_sigma ;

    Uk.resize( num_Fir, num_Joints ) ;
    Uk = Eigen::MatrixXd::Zero( num_Fir, num_Joints ) ;

    Uk_plus.resize( num_Fir, num_Joints ) ;
    Uk_plus = Eigen::MatrixXd::Zero( num_Fir, num_Joints ) ;

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

    m_lm      = 0.98; // Forgetting factor

    // Don't use fixed weights by default
    useFixedWeights = false;

    // initial conditions
    ode_init_x[0 ] = 0.0;
    ode_init_x[1 ] = 0.0;
    ode_init_x[2 ] = 0.0;

    rls_filter.init( Wk, Uk, Dk, Pk, m_lm );

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

  void setLambda( const double & l )
  {
    m_lm = l;
    rls_filter.setLambda( m_lm ) ;
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

  void setUpdatedWeights()
  {
	useFixedWeights = false;
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

    // Save iteration number
    iter = iter + 1;

    // Desired is the task reference model
    Dk = ref_q_m;

    // Save input forces/torques
	stackIntegrate( q_m, t_r );

    //if( iter > num_Fir )
    {
      if( !useFixedWeights )
      {
    	  Pk = Pk - Pk*Uk*(1.00/0.01)*Uk.transpose()*Pk * delT;
    	  Wk = Pk*Uk*(1.00/0.01)*( Dk - Uk.transpose()*Wk );
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
}
}

#endif /* CTRLSMODEL_H_ */
