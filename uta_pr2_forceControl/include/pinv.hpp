/*
 * pinv.hpp
 *
 *  Created on: Feb 4, 2013
 *      Author: andrew.somerville
 */

#ifndef PINV_HPP_
#define PINV_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

template <typename MatrixT>
typename Eigen::Matrix<typename MatrixT::Scalar, MatrixT::ColsAtCompileTime, MatrixT::RowsAtCompileTime>
pseudoInverse( const MatrixT & inputMat,
               const double lambda = 1 )
{
	typedef typename Eigen::JacobiSVD<MatrixT> JacobiSVD;
	JacobiSVD svd;

	// FIXME should we pass in max iterations somehow?
	// FIXME this can be made faster if the input mat is dynamic using ComputeThinU/V
	svd.compute(inputMat, Eigen::ComputeFullU | Eigen::ComputeFullV );

	typename JacobiSVD::MatrixUType        uMat = svd.matrixU();
	typename JacobiSVD::MatrixVType        vMat = svd.matrixV();

	typename JacobiSVD::SingularValuesType sMat = svd.singularValues();
	typename JacobiSVD::SingularValuesType sMatPinv = sMat; //svd.invertedSingularValues();;

	double epsilon = 1.e-6; // taken from example
	for (long i = 0; i < sMat.size(); ++i)
	{
		if (sMat(i) > epsilon)
			sMatPinv(i) = 1.0 / sMat(i);
		else
			sMatPinv(i) = sMat(i)/(sMat(i)*sMat(i)+lambda*lambda);
	}

	typedef typename Eigen::Matrix<typename MatrixT::Scalar, MatrixT::ColsAtCompileTime, MatrixT::RowsAtCompileTime> PinvMat;
	PinvMat pinvMat;

	// FIXME this is using lowest common denominator interface to handle dynamic matrix case
	PinvMat singularValuesCorrected(inputMat.cols(), inputMat.rows());
	singularValuesCorrected = PinvMat::Zero(inputMat.cols(), inputMat.rows());

	// FIXME this is using lowest common denominator interface to handle dynamic matrix case
	int minDim = std::min(inputMat.cols(), inputMat.rows());

	// FIXME this is using lowest common denominator interface to handle dynamic matrix case
	singularValuesCorrected.block( 0,0,minDim,minDim ) = sMatPinv.asDiagonal();

	pinvMat = (vMat * singularValuesCorrected * uMat.transpose());

	return pinvMat;
}


//template <typename MatrixT0, typename MatrixT1 = Eigen::MatrixXd, typename MatrixT2 = Eigen::MatrixXd >
inline
typename Eigen::Matrix<typename Eigen::MatrixXd::Scalar, Eigen::MatrixXd::ColsAtCompileTime, Eigen::MatrixXd::RowsAtCompileTime>
weightedPseudoInverse( const Eigen::MatrixXd & inputMat,
                       const Eigen::MatrixXd & inputSpaceWeightMat   = Eigen::MatrixXd(),
                       const Eigen::MatrixXd & outputSpaceWeightMat  = Eigen::MatrixXd(),
                       const double lambda = 1 )
//template <typename MatrixT0, typename MatrixT1 = Eigen::MatrixXd, typename MatrixT2 = Eigen::MatrixXd >
//typename Eigen::Matrix<typename MatrixT0::Scalar, MatrixT0::ColsAtCompileTime, MatrixT0::RowsAtCompileTime>
//weightedPseudoInverse( const MatrixT0 & inputMat,
//                       const MatrixT1 & inputSpaceWeightMat   = Eigen::MatrixXd(),
//                       const MatrixT2 & outputSpaceWeightMat  = Eigen::MatrixXd(),
//                       const double lambda = 1 )
{
    typedef Eigen::MatrixXd MatrixT0; // compile work around
    typedef Eigen::MatrixXd MatrixT1; // compile work around
    typedef Eigen::MatrixXd MatrixT2; // compile work around


	typedef typename Eigen::JacobiSVD<MatrixT0/*,Eigen::FullPivHouseholderQRPreconditioner*/> JacobiSVD;
	JacobiSVD svd;

	MatrixT0 weightedInputMat;
	weightedInputMat = inputMat;

	if( inputSpaceWeightMat.rows() != 0 )
		weightedInputMat = weightedInputMat.lazyProduct( inputSpaceWeightMat );
//		weightedInputMat = weightedInputMat * inputSpaceWeightMat;

	if( outputSpaceWeightMat.rows() != 0 )
		weightedInputMat = outputSpaceWeightMat.lazyProduct( weightedInputMat );
//		weightedInputMat = outputSpaceWeightMat * weightedInputMat;

	// FIXME this can be made faster if the input mat is dynamic using ComputeThinU/V
	svd.compute(weightedInputMat, Eigen::ComputeFullU | Eigen::ComputeFullV );

	typename JacobiSVD::MatrixUType        uMat = svd.matrixU();
	typename JacobiSVD::MatrixVType        vMat = svd.matrixV();

	typename JacobiSVD::SingularValuesType sMat = svd.singularValues();
	typename JacobiSVD::SingularValuesType sMatPinv = sMat; //svd.invertedSingularValues();;

	if( inputSpaceWeightMat.rows() != 0 )
		vMat = inputSpaceWeightMat.lazyProduct( vMat ); // undo earlier weighting?
//		vMat = inputSpaceWeightMat  * vMat; // undo earlier weighting?

	if( outputSpaceWeightMat.rows() != 0 )
		uMat = outputSpaceWeightMat.lazyProduct( uMat ); // undo earlier weighting? // this one seems wrong, but matches the KDL wlds implementation
//		uMat = outputSpaceWeightMat * uMat; // undo earlier weighting? // this one seems wrong, but matches the KDL wlds implementation

	double epsilon = 1.e-6; // taken from example
	for (long i = 0; i < sMat.size(); ++i)
	{
		if (sMat(i) > epsilon)
			sMatPinv(i) = 1.0 / sMat(i);
		else
			sMatPinv(i) = (sMat(i)/(sMat(i)*sMat(i)+lambda*lambda));
//			sMatPinv(i) = 0;
	}

	typedef typename Eigen::Matrix<typename MatrixT0::Scalar, MatrixT0::ColsAtCompileTime, MatrixT0::RowsAtCompileTime> PinvMat;
	PinvMat pinvMat;

	// FIXME this is using lowest common denominator interface to handle dynamic matrix case
	PinvMat singularValuesCorrected(inputMat.cols(), inputMat.rows());
	singularValuesCorrected = PinvMat::Zero(inputMat.cols(), inputMat.rows());

	// FIXME this is using lowest common denominator interface to handle dynamic matrix case
	int minDim = std::min(inputMat.cols(), inputMat.rows());

	// FIXME this is using lowest common denominator interface to handle dynamic matrix case
	singularValuesCorrected.block( 0,0,minDim,minDim ) = sMatPinv.asDiagonal();

	pinvMat = (vMat * singularValuesCorrected * uMat.transpose());

	return pinvMat;
}


#endif /* PINV_HPP_ */
