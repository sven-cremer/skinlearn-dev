/*
 * RlsModel.h
 *
 *  Created on: Oct 30, 2014
 *      Author: Isura
 */

#ifndef RLSMODEL_H_
#define RLSMODEL_H_

namespace csl
{
namespace outer_loop
{
class RlsModel
{

  int num_Dim;		// Number of dimensions (joints or Cartesian)
  int num_Sen;		// Number of force sensors
  int num_Fir;		// Number of FIR parameters
  int num_x_Fir;	// Number of x_m FIR parameters
  int num_f_Fir;	// Number of f_h FIR parameters

  Eigen::MatrixXd q;
  Eigen::MatrixXd qd;
  Eigen::MatrixXd qdd;

  Eigen::MatrixXd q_m;
  Eigen::MatrixXd qd_m;
  Eigen::MatrixXd qdd_m;

  Eigen::MatrixXd prv_q_m;
  Eigen::MatrixXd prv_qd_m;

  // Reference task model
  Eigen::MatrixXd ref_q_d;
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

  double m_lm;
  double m_sigma;

  double posInit;

  int iter;

  // Task model
  double m ;
  double d ;
  double k ;

  // 1st order model
  double a_task ;
  double b_task ;

//  fir_state_type ode_init_x;

  oel::ls::RLSFilter rls_filter;

  // Switch RLS on/off
  bool useFixedWeights ;
  bool firstTime ;

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
    // Moves top to bottom rows are time series, columns are dimension
    // First in First out bottom most location nth row is dumped

	  Uk_plus.block<4-1, 1>(1,0) = Uk.block<4-1, 1>(0,0);
	  Uk_plus.block<1,1>(0,0) = - y_prev.transpose();

	  Uk_plus.block<4-1, 1>(5,0) = Uk.block<4-1, 1>(4,0);
	  Uk_plus.block<1,1>(4,0) = u_in.transpose();
/*
	// Update x_m
    Uk_plus.block(1,0,num_x_Fir-1,num_Dim) = Uk.block(0,0,num_x_Fir-1,num_Dim);	// Move down rows (overwrites last entry)
    Uk_plus.block(0,0,1,num_Dim) = - y_prev.transpose();						// Update first entry with new value(s)

    // Update f_i for each sensor i
    for(int i=num_x_Fir; i<num_x_Fir+num_Sen*num_f_Fir; i += num_f_Fir)
    {
		Uk_plus.block(i+1,0,num_f_Fir-1,num_Dim) = Uk.block(i,0,num_f_Fir-1,num_Dim);	// Move down rows
		Uk_plus.block(i,0,1,num_Dim) = u_in.col(i).transpose();							// Update first entry
    }
*/
    Uk = Uk_plus;
  }

public:
  RlsModel()
  {
    delT = 0.001; /// 1000 Hz by default
    iter = 1;

    a_task = 0.5 ;
    b_task = 0.5 ;

    init( 1, 4, 4, 1 );	// TODO make parameters
  }
  ~RlsModel()
  {
  }

  void init( int para_num_Dim, int para_num_Fir,  int para_num_Sen = 1)
  {
	  init( para_num_Dim, para_num_Fir, para_num_Fir, para_num_Sen);
  }

  void init( int para_num_Dim, int para_num_x_Fir, int para_num_f_Fir, int para_num_Sen)
  {
    num_Dim = para_num_Dim;
    num_Sen = para_num_Sen;
    num_x_Fir = para_num_x_Fir;
    num_f_Fir = para_num_f_Fir;

    num_Fir = num_x_Fir + num_Sen*num_f_Fir;

    // Set dimensions
    q         .resize( num_Dim, 1 ) ;
    qd        .resize( num_Dim, 1 ) ;
    qdd       .resize( num_Dim, 1 ) ;

    q_m       .resize( num_Dim, 1 ) ;
    qd_m      .resize( num_Dim, 1 ) ;
    qdd_m     .resize( num_Dim, 1 ) ;

    prv_q_m   .resize( num_Dim, 1 ) ;
    prv_qd_m  .resize( num_Dim, 1 ) ;

    ref_q_d   .resize( num_Dim, 1 ) ;
    ref_qd_m  .resize( num_Dim, 1 ) ;
    ref_qdd_m .resize( num_Dim, 1 ) ;

    task_ref  .resize( num_Dim, 1 ) ;
    t_r		  .resize( num_Dim, 1 ) ;

    Wk		.resize( num_Fir, num_Dim ) ;
    Uk		.resize( num_Fir, num_Dim ) ;
    Uk_plus	.resize( num_Fir, num_Dim ) ;
    Dk		.resize( num_Dim, 1       ) ;
    Pk		.resize( num_Fir, num_Fir ) ;

    // Initialize matricies
    q         .setZero();
    qd        .setZero();
    qdd       .setZero();
    q_m       .setZero();
    qd_m      .setZero();
    qdd_m     .setZero();
    prv_q_m   .setZero();
    prv_qd_m  .setZero();
    ref_q_d   .setZero();
    ref_qd_m  .setZero();
    ref_qdd_m .setZero();
    task_ref  .setZero();
    t_r       .setZero();
    Wk		  .setZero();
    Dk		  .setZero();
    Uk_plus	  .setZero();
    Uk		  .setZero();

    m_sigma = 0.001 ;
    Pk = Eigen::MatrixXd::Identity( num_Fir, num_Fir )/m_sigma ;

    useFixedWeights = false;	// Don't use fixed weights by default
    firstTime = true;

    // initial conditions
//    ode_init_x[0 ] = 0.0;
//    ode_init_x[1 ] = 0.0;
//    ode_init_x[2 ] = 0.0;

    posInit = 0;
    m_lm 	= 0.98;				// Forgetting factor

    rls_filter.init( Wk, Uk, Dk, Pk, m_lm );

  }

  void initPos( double p_posInit )
  {
	  posInit = p_posInit;

	  Eigen::MatrixXd tmp = Eigen::MatrixXd::Ones(num_x_Fir, num_Dim);
	  tmp *= posInit;

	  Uk_plus.block(0,0,num_x_Fir, num_Dim) = tmp;

	  Uk = Uk_plus;

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

  void initRls( const double & l, const double & sigma )
  {
    m_lm    = l     ;
    m_sigma = sigma ;
    rls_filter.init( Wk, Uk, Dk, Pk, m_lm );
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

    param_task_ref_model = ref_q_d(0)     ;
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

    param_task_ref_model = ref_q_d        ;
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

    // Save input forces/torques in Uk
    stackArmaIn( q_m, t_r );

    // Compute xd and perform RLS update
    update();

    // Return results
    param_task_ref_model = ref_q_d(0)     ;		// x_d
    param_q_m            = q_m(0)         ;		// x_m
    param_qd_m           = qd_m(0)        ;		// xd_m
    param_qdd_m          = qdd_m(0)       ;		// xdd_m
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
    task_ref             = param_task_ref ;		// x_r

    // Save input forces/torques in Uk
    stackArmaIn( q_m, t_r );

    // Compute xd and perform RLS update
    update();

    // Return results
    param_task_ref_model = ref_q_d        ;		// x_d
    param_q_m            = q_m            ;		// x_m
    param_qd_m           = qd_m           ;		// xd_m
    param_qdd_m          = qdd_m          ;		// xdd_m
  }

  void update()
  {
	if( firstTime )
	{
		prv_q_m  = q_m ;
		prv_qd_m = qd_m;

		ref_q_d  = q_m;

		firstTime = false;
	}

//    ode_init_x[2] = task_ref(0);
//    boost::numeric::odeint::integrate( task_model , ode_init_x , 0.0 , delT , delT );

	// Compute xd using prescribed task model D(s)
    ref_q_d(0)   = ref_q_d(0) + ref_qd_m(0)*delT;				// xd
    ref_qd_m(0)  = a_task*task_ref(0) - b_task*ref_q_d(0);		//

    ref_qdd_m(0) = 0; //m*( task_ref(0) - d*ode_init_x[1 ] - k*ode_init_x[0 ] );

//    ref_q_m(0)   = ode_init_x[0 ] ;
//    ref_qd_m(0)  = ode_init_x[1 ] ;
//    ref_qdd_m(0) = 0; //m*( task_ref(0) - d*ode_init_x[1 ] - k*ode_init_x[0 ] );

    // Save iteration number
    iter = iter + 1;

    // Desired is the task reference model (xm -> xd)
    Dk = ref_q_d;

    // Update filter weights using RLS
    if( !useFixedWeights )
    {
    	rls_filter.Update( Wk, Uk, Dk, Pk );

    	Wk = rls_filter.getEstimate();
    	Pk = rls_filter.getCovariance();
    }

    // Compute x_m(t) = h(t)*theta(t)
    q_m   = Uk.transpose()*Wk  ;

    // Compute xd_m and xdd_m using backward difference (TODO better way to do this?)
    qd_m  = (q_m  - prv_q_m )/delT ;
    qdd_m = (qd_m - prv_qd_m)/delT ;

    // Store values
    prv_q_m  = q_m ;
    prv_qd_m = qd_m;S

  }
};
}
}

#endif /* RLSMODEL_H_ */
