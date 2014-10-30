/*
 * MracModel.h
 *
 *  Created on: Oct 30, 2014
 *      Author: sven
 */

#ifndef MRACMODEL_H_
#define MRACMODEL_H_

namespace csl
{
namespace outer_loop
{
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
    theta_1  = 0 ; theta_2 = 0 ; theta_3 = 0 ;
    yhat_dot = 0 ; y_hat   = 0 ;
    yp       = 0 ; ym      = 0 ;

//    u_c = 1;

    // Gains
    gamma_1 = 1     ,
    gamma_2 = 2000  ,
    gamma_3 = 2500  ,
    gamma_4 = 5000  ,
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

  void updateGamma( double param_gamma_1,
                    double param_gamma_2,
                    double param_gamma_3,
                    double param_gamma_4,
                    double param_gamma_5 )
  {
	gamma_1 = param_gamma_1 ;
	gamma_2 = param_gamma_2 ;
	gamma_3 = param_gamma_3 ;
	gamma_4 = param_gamma_4 ;
	gamma_5 = param_gamma_5 ;
  }

  void getGamma( double & param_gamma_1,
                 double & param_gamma_2,
                 double & param_gamma_3,
                 double & param_gamma_4,
                 double & param_gamma_5 )
  {
    param_gamma_1 = gamma_1 ;
    param_gamma_2 = gamma_2 ;
    param_gamma_3 = gamma_3 ;
    param_gamma_4 = gamma_4 ;
    param_gamma_5 = gamma_5 ;
  }

  void getEstimatedParams( double & param_y_hat  ,
		                   double & param_theta_1,
		                   double & param_theta_2,
		                   double & param_theta_3,
		                   double & param_ahat   ,
		                   double & param_bhat    )
  {
      param_y_hat   = y_hat   ;
      param_theta_1 = theta_1 ;
      param_theta_2 = theta_2 ;
      param_theta_3 = theta_3 ;
      param_ahat    = ahat    ;
      param_bhat    = bhat    ;
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

//    ref_q_m(0)   = ref_q_m(0) + ref_qd_m(0)*delT;
//    ref_qd_m(0)  = a_task*task_ref(0) -  b_task*ref_q_m(0);

//    ref_qdd_m(0) = 0; //m*( task_ref(0) - d*ode_init_x[1 ] - k*ode_init_x[0 ] );

//    ref_q_m(0)   = ode_init_x[0 ] ;
//    ref_qd_m(0)  = ode_init_x[1 ] ;
//    ref_qdd_m(0) = 0; //m*( task_ref(0) - d*ode_init_x[1 ] - k*ode_init_x[0 ] );

    // Save iteration number
    iter = iter + 1;

    // Desired is the task reference model
    u_c = task_ref(0);

    {

      // Human force
      y = t_r(0) ;

      u           = - theta_1 * yhat_dot - theta_2 * yp - theta_3 * yhat_dot ;
      e           = yp - ym                                                  ;
      y_tilde     = y - y_hat                                                ;

      // k + 1
      // dot
      // FIXME Fake Human Force
      // y_dot       = -a         * y             + b      * u_c             ;
      yp_dot      = -an        * yp            + bn     * u                  ;
      ym_dot      = -am        * ym            + bm     * u_c                ;

      yhat_dot    = -ahat      * y_hat         + bhat   * u_c                ;
      theta_1_dot =  gamma_1   * e * u_c                                     ;
      theta_2_dot =  gamma_2   * e * yp                                      ;
      theta_3_dot =  gamma_3   * e * bn * y_hat + gamma_1 * ahat * e * u_c   ;
      ahat_dot    = -gamma_4 * y_tilde * y_hat                               ;
      bhat_dot    =  gamma_5 * y_tilde * u_c                                 ;

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
      q_m(0)   = yp ;

      // FIXME Fake Human Force
      // t_r(0) = y;

      // Backward difference
      // TODO better way to do this?
      qd_m  = (q_m  - prv_q_m )/delT ;
      qdd_m = (qd_m - prv_qd_m)/delT ;

      ref_q_m(0) = ym ;

    }

    prv_q_m  = q_m ;
    prv_qd_m = qd_m;

//      std::cout<< "Uk : " << Uk.transpose() <<"\n\n";
//      std::cout<< "q  : " << q_m <<"\n\n";
  }
};
}
}

#endif /* MRACMODEL_H_ */
