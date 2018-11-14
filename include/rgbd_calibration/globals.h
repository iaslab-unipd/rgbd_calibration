/*
 *  Copyright (C) 2013 - Filippo Basso <bassofil@dei.unipd.it>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RGBD_CALIBRATION_GLOBALS_H_
#define RGBD_CALIBRATION_GLOBALS_H_

#include <calibration_common/base/math.h>
#include <calibration_common/objects/globals.h>
#include <calibration_common/depth/depth.h>

//#include <kinect/depth/polynomial_function_fit.h>
#include <kinect/depth/polynomial_matrix_fit.h>
#include <kinect/depth/two_steps_undistortion.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#define G_POLY_DEGREE 2
#define G_POLY_MIN_DEGREE 1
#define L_POLY_DEGREE 2
#define L_POLY_MIN_DEGREE 0

//#define KINECT_FOV_X 58.6 //Scalar(DEG2RAD(57.0)) 58.6
//#define KINECT_FOV_Y 45.7 //Scalar(DEG2RAD(43.5)) 45.7

//#define KINECT_ERROR_POLY Vector3(0.0, 0.0, 0.0035)

namespace calibration
{

typedef Polynomial<Scalar, L_POLY_DEGREE, L_POLY_MIN_DEGREE>  LocalPolynomial;
typedef Polynomial<Scalar, G_POLY_DEGREE, G_POLY_MIN_DEGREE>  GlobalPolynomial;
typedef Polynomial<Scalar, G_POLY_DEGREE, 0>  InverseGlobalPolynomial;

typedef PolynomialMatrixSmoothModel<LocalPolynomial>                            LocalModel;
typedef PolynomialMatrixPCL<LocalModel, Scalar, PCLPoint3>                      LocalMatrixPCL;
typedef PolynomialMatrixEigen<LocalModel, Scalar>                               LocalMatrixEigen;
typedef PolynomialMatrixSmoothModelFitPCL<LocalPolynomial, Scalar, PCLPoint3>   LocalMatrixFitPCL;
typedef PolynomialMatrixSmoothModelFitEigen<LocalPolynomial, Scalar>            LocalMatrixFitEigen;

typedef PolynomialMatrixSmoothModel<GlobalPolynomial>                           GlobalModel;
typedef PolynomialMatrixPCL<GlobalModel, Scalar, PCLPoint3>                     GlobalMatrixPCL;
typedef PolynomialMatrixEigen<GlobalModel, Scalar>                              GlobalMatrixEigen;
typedef PolynomialMatrixSmoothModelFitPCL<GlobalPolynomial, Scalar, PCLPoint3>  GlobalMatrixFitPCL;
typedef PolynomialMatrixSmoothModelFitEigen<GlobalPolynomial, Scalar>           GlobalMatrixFitEigen;

typedef PolynomialMatrixSimpleModel<InverseGlobalPolynomial>                           InverseGlobalModel;
typedef PolynomialMatrixPCL<InverseGlobalModel, Scalar, PCLPoint3>              InverseGlobalMatrixPCL;
typedef PolynomialMatrixEigen<InverseGlobalModel, Scalar>                       InverseGlobalMatrixEigen;
typedef PolynomialMatrixSimpleModelFitPCL<InverseGlobalPolynomial, Scalar, PCLPoint3>  InverseGlobalMatrixFitPCL;
typedef PolynomialMatrixSimpleModelFitEigen<InverseGlobalPolynomial, Scalar>           InverseGlobalMatrixFitEigen;

//typedef PolynomialFunctionModel<GlobalPolynomial>::Data UFunctionData;
//typedef PolynomialFunctionPCL<GlobalPolynomial, PCLPoint3> UFunctionPCL;
//typedef PolynomialFunctionEigen<GlobalPolynomial, Scalar> UFunctionEigen;
//typedef PolynomialFunctionFitPCL<GlobalPolynomial, Scalar, PCLPoint3> UFunctionFitPCL;
//typedef PolynomialFunctionFitEigen<GlobalPolynomial, Scalar> UFunctionFitEigen;

typedef TwoStepsModel<Scalar, LocalModel, GlobalModel>                      UndistortionModel;
typedef TwoStepsUndistortionEigen<Scalar, LocalModel, GlobalModel>          UndistortionEigen;
typedef TwoStepsUndistortionPCL<Scalar, PCLPoint3, LocalModel, GlobalModel> UndistortionPCL;

} /* namespace calibration */
#endif /* RGBD_CALIBRATION_GLOBALS_H_ */
