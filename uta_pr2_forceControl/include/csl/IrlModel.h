/*
 * IrlModel.h
 *
 *  Created on: Oct 30, 2014
 *      Author: sven
 */

#ifndef IRLMODEL_H_
#define IRLMODEL_H_

#include <csl/math.h>

namespace csl
{
namespace outer_loop
{
class IrlModel
{

  int num_dof    ; // number of joints.
  int num_samples; // number of joints.

  Eigen::MatrixXd x;
  Eigen::MatrixXd xd;
  Eigen::MatrixXd xdd;

  Eigen::MatrixXd x_m;
  Eigen::MatrixXd xd_m;
  Eigen::MatrixXd xdd_m;

  Eigen::MatrixXd prv_x_m;
  Eigen::MatrixXd prv_xd_m;

  // Reference task model
  Eigen::MatrixXd x_d;
  Eigen::MatrixXd xd_d;
  Eigen::MatrixXd xdd_d;

  Eigen::MatrixXd x_r;
  Eigen::MatrixXd task_ref_model;

  Eigen::MatrixXd f_h;

  Eigen::MatrixXd X           ;
  Eigen::MatrixXd X_0         ;
  Eigen::MatrixXd X_kron      ;
  Eigen::MatrixXd X_0_kron    ;
  Eigen::MatrixXd XU_kron     ;
  Eigen::MatrixXd ed_bar      ;

  Eigen::MatrixXd Delxx       ;
  Eigen::MatrixXd Ixx         ;
  Eigen::MatrixXd Ixu         ;

  Eigen::MatrixXd In          ;

  Eigen::MatrixXd Theta       ;
  Eigen::MatrixXd Ksi         ;
  Eigen::MatrixXd Psi         ;

  Eigen::MatrixXd Kh          ;
  Eigen::MatrixXd Kq          ;
  Eigen::MatrixXd K           ;
  Eigen::MatrixXd Kvec        ;
  Eigen::MatrixXd P           ;
  Eigen::MatrixXd Q           ;
  Eigen::MatrixXd R           ;
  Eigen::MatrixXd B           ;
  Eigen::MatrixXd U           ;

  Eigen::MatrixXd M_bar        ; // Optimal Mass
  Eigen::MatrixXd D_bar        ; // Optimal Damping
  Eigen::MatrixXd K_bar        ; // Optimal Stiffness

  double delT; // Time step

  int iter;

  // Task model
  double m ;
  double d ;
  double k ;

  // 1st order model
  double a_task ;
  double b_task ;

  // Switch RLS on/off
  bool useIrl ;

public:
  IrlModel()
  {
    delT = 0.001; /// 1000 Hz by default
    iter = 0;

    a_task = 0.5 ;
    b_task = 0.5 ;

    //
    init( 1, 2000 );
  }
  ~IrlModel()
  {
  }

  void init( int para_num_Dof, int para_num_Samples )
  {
    num_dof     = para_num_Dof     ;
    num_samples = para_num_Samples ;

    x         .resize( num_dof, 1 ) ;
    xd        .resize( num_dof, 1 ) ;
    xdd       .resize( num_dof, 1 ) ;

    x_m       .resize( num_dof, 1 ) ;
    xd_m      .resize( num_dof, 1 ) ;
    xdd_m     .resize( num_dof, 1 ) ;

    prv_x_m   .resize( num_dof, 1 ) ;
    prv_xd_m  .resize( num_dof, 1 ) ;

    x_d       .resize( num_dof, 1 ) ;
    xd_d      .resize( num_dof, 1 ) ;
    xdd_d     .resize( num_dof, 1 ) ;

    x_r       .resize( num_dof, 1 ) ;

    f_h       .resize( num_dof, 1 ) ;

    X         .resize( 3*num_dof, 1 );
    X         = Eigen::MatrixXd::Zero( 3*num_dof, 1 );

    X_0       .resize( 3*num_dof, 1 );
    X_0       = Eigen::MatrixXd::Zero( 3*num_dof, 1 );

    X_kron    .resize(X.rows()*X.rows(), X.cols()*X.cols());
	X_kron    = Eigen::MatrixXd::Zero(X.rows()*X.rows(), X.cols()*X.cols());

	X_0_kron  .resize(X.rows()*X.rows(), X.cols()*X.cols());
    X_0_kron  = Eigen::MatrixXd::Zero(X.rows()*X.rows(), X.cols()*X.cols());

    XU_kron   .resize(3*num_dof*num_dof, 1);
    XU_kron   = Eigen::MatrixXd::Zero(3*num_dof*num_dof, 1);

	Delxx     .resize( num_samples, 3*num_dof*(3*num_dof+1)/2 );
	Delxx     = Eigen::MatrixXd::Zero( num_samples, 3*num_dof*(3*num_dof+1)/2 );

	Ixx       .resize( num_samples, 1 );
	Ixx       = Eigen::MatrixXd::Zero( num_samples, 3*num_dof*3*num_dof );

	Ixu       .resize( num_samples, 1 ) ;
	Ixu       = Eigen::MatrixXd::Zero( num_samples, 3*num_dof*num_dof );

	In        .resize( 3*num_dof, 3*num_dof ) ;
	In        = Eigen::MatrixXd::Identity( 3*num_dof, 3*num_dof );

	Theta     .resize( num_samples, 3*num_dof*(3*num_dof+1)/2 + num_dof*3*num_dof ) ;
	Theta     = Eigen::MatrixXd::Zero( num_samples, 3*num_dof*(3*num_dof+1)/2 + num_dof*3*num_dof );

	Ksi       .resize( num_samples, 1 ) ;
	Ksi       = Eigen::MatrixXd::Zero( num_samples, 1 );

	Psi       .resize( 3*num_dof*3*num_dof*num_dof*3*num_dof, 1 ) ;
	Psi       = Eigen::MatrixXd::Zero( 3*num_dof*3*num_dof*num_dof*3*num_dof, 1 );

    ed_bar    .resize( 2*num_dof, 1 ) ;
    ed_bar    = Eigen::MatrixXd::Zero( 2*num_dof, 1 );

    Kh        .resize( num_dof, num_dof ) ;
    Kh        = Eigen::MatrixXd::Zero( num_dof, num_dof );

    Kq        .resize( num_dof, 2*num_dof ) ;
    Kq        = Eigen::MatrixXd::Zero( num_dof, 2*num_dof );

    K         .resize( num_dof, 3*num_dof ) ;
    K         = Eigen::MatrixXd::Zero( num_dof, 3*num_dof );

    Kvec      .resize( num_dof*3*num_dof, 1 ) ;
    Kvec      = Eigen::MatrixXd::Zero( num_dof*3*num_dof, 1 );

    P         .resize( 3*num_dof, 3*num_dof ) ;
    P         = Eigen::MatrixXd::Zero( 3*num_dof, 3*num_dof );

    Q         .resize( 3*num_dof, 3*num_dof ) ;
	Q         = Eigen::MatrixXd::Identity( 3*num_dof, 3*num_dof );

    R         .resize( num_dof, num_dof ) ;
    R         = Eigen::MatrixXd::Identity( num_dof, num_dof );

    B         .resize( 3*num_dof, num_dof ) ;
    B         << Eigen::MatrixXd::Zero(     num_dof, num_dof ),
    		     Eigen::MatrixXd::Identity( num_dof, num_dof ),
    		     Eigen::MatrixXd::Zero(     num_dof, num_dof );

    U         .resize( num_dof, 1 ) ;
	U         = Eigen::MatrixXd::Identity( num_dof, 1 );

    M_bar     .resize( num_dof, num_dof ) ;
    M_bar     = 20*Eigen::MatrixXd::Identity( num_dof, num_dof );

    D_bar     .resize( num_dof, num_dof ) ;
    D_bar     = 50*Eigen::MatrixXd::Identity( num_dof, num_dof );

    K_bar     .resize( num_dof, num_dof ) ;
    K_bar     = 0*Eigen::MatrixXd::Identity( num_dof, num_dof );

    x         = Eigen::MatrixXd::Zero( num_dof, 1 );
    xd        = Eigen::MatrixXd::Zero( num_dof, 1 );
    xdd       = Eigen::MatrixXd::Zero( num_dof, 1 );

    x_m       = Eigen::MatrixXd::Zero( num_dof, 1 );
    xd_m      = Eigen::MatrixXd::Zero( num_dof, 1 );
    xdd_m     = Eigen::MatrixXd::Zero( num_dof, 1 );

    prv_x_m   = Eigen::MatrixXd::Zero( num_dof, 1 );
    prv_xd_m  = Eigen::MatrixXd::Zero( num_dof, 1 );

    x_d       = Eigen::MatrixXd::Zero( num_dof, 1 );
    xd_d      = Eigen::MatrixXd::Zero( num_dof, 1 );
    xdd_d     = Eigen::MatrixXd::Zero( num_dof, 1 );

    x_r       = Eigen::MatrixXd::Zero( num_dof, 1 );

    f_h       = Eigen::MatrixXd::Zero( num_dof, 1 );

    // Don't use fixed weights by default
    useIrl = false;

    // IRL init
    //rls_filter.init( Wk, Uk, Dk, Pk, m_lm );

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

  //TODO
  void initIrl(  )
  {
	  // FIXME IRL
//    rls_filter.init( Wk, Uk, Dk, Pk, m_lm );
  }

//  void setWeights( Eigen::MatrixXd & param_Wk )
//  {
//    Wk = param_Wk;
//  }

  // FIXME
  void setFixedWeights( )
  {
	useIrl = false;
  }

  void setUpdateIrl()
  {
	  useIrl = true;
  }

  void updateIrl(  Eigen::MatrixXd & param_xd_m          ,
                   Eigen::MatrixXd & param_xd            ,
                   Eigen::MatrixXd & param_x_m           ,
                   Eigen::MatrixXd & param_x             ,
                   Eigen::MatrixXd & param_xdd_m         ,
                   Eigen::MatrixXd & param_f_h           ,
                   Eigen::MatrixXd & param_x_r           ,
                   Eigen::MatrixXd & param_task_ref_model )
  {
    xd_m   = param_xd_m  ;
    xd     = param_xd    ;
    x_m    = param_x_m   ;
    x      = param_x     ;
    xdd_m  = param_xdd_m ;
    f_h    = param_f_h   ;
    x_r    = param_x_r   ;

    // Save input forces/torques
//    stackArmaIn( x_m, f_h );
    update();

    param_task_ref_model = x_d   ;
    param_x_m            = x_m   ;
    param_xd_m           = xd_m  ;
    param_xdd_m          = xdd_m ;
  }

  void update()
  {
	// Desired trajectory from the task reference model
    x_d   = x_d        + xd_d*delT ;
    xd_d  = a_task*x_r - b_task*x_d;

    // No task acceleration
//    xdd_d = 0;

    ed_bar << x_d  - x_m ,
    		  xd_d - xd_m;

    X << x_m ,
         xd_m,
         f_h ;

    // Save iteration number
    iter = iter + 1;

    if( useIrl )
    {
      if( iter < num_samples )
      {
    	  X_0_kron = csl::math::kroneckerProduct(X_0,X_0);
    	  X_kron   = csl::math::kroneckerProduct(X,X);
    	  XU_kron   = csl::math::kroneckerProduct(X,U);

    	  Delxx.row(iter) = (X_kron - X_0_kron).transpose();
    	  Ixx.row(iter)   = X_kron*delT;
    	  Ixu.row(iter)   = XU_kron*delT;
      }else
      {
    	  // FIXME
		  Eigen::MatrixXd KtR = K.transpose()*R;

		  Theta << Delxx, -2*Ixx*csl::math::kroneckerProduct(In,KtR) - 2*Ixu*csl::math::kroneckerProduct(In,R);
		  Ksi << -Ixx*Q.array().transpose();

		  Eigen::MatrixXd ThetaThetaTrans = Theta.transpose()*Theta;

		  Psi = ThetaThetaTrans.inverse()*Theta.transpose()*Ksi;

		  Kvec = Psi.block( 3*num_dof*3*num_dof, 0, num_dof*3*num_dof, 1);
		  K = Eigen::Map<Eigen::MatrixXd>(Kvec.data(),num_dof, 3*num_dof);
      }
    }

    if( iter > num_samples )
	{
      // First order integration
      // TODO better way to do this?
      xdd_m = ( f_h - D_bar * xd_m - K_bar * x_m );
      xd_m  =   xd_m + xdd_m  *  delT             ;
      x_m   =   x_m  + xd_m   *  delT             ;
	}

    prv_x_m  = x_m ;
    prv_xd_m = xd_m;

    X_0 = X;

  }

};
}
}

#endif /* RLSMODEL_H_ */
