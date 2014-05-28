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

#include <kinect/depth/polynomial_function_fit.h>
#include <kinect/depth/polynomial_matrix_fit.h>
#include <kinect/depth/two_steps_undistortion.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#define G_POLY_DEGREE 2
#define G_POLY_MIN_DEGREE 0

#define L_POLY_DEGREE 2
#define L_POLY_MIN_DEGREE 0

#define KINECT_FOV_X 58.6 //Scalar(DEG2RAD(57.0)) 58.6
#define KINECT_FOV_Y 45.7 //Scalar(DEG2RAD(43.5)) 45.7

#define KINECT_ERROR_POLY Vector3(0.0, 0.0, 0.0035)

namespace calibration
{

typedef Polynomial<Scalar, L_POLY_DEGREE, L_POLY_MIN_DEGREE> LocalPolynomial;
typedef Polynomial<Scalar, G_POLY_DEGREE, G_POLY_MIN_DEGREE> GlobalPolynomial;

typedef PolynomialMatrixProjectedModel<LocalPolynomial> LUMatrixModel;

typedef DepthUndistortionImpl<LUMatrixModel, DepthPCL> LUMatrixPCL;
typedef DepthUndistortionImpl<LUMatrixModel, DepthEigen> LUMatrixEigen;

typedef PolynomialUndistortionMatrixFitPCL<LocalPolynomial, PCLPoint3> LUndMatrixFitPCL;
typedef PolynomialUndistortionMatrixFitEigen<LocalPolynomial> LUndMatrixFitEigen;

typedef PolynomialMatrixProjectedModel<GlobalPolynomial> GUMatrixModel;

typedef DepthUndistortionImpl<GUMatrixModel, DepthPCL> GUMatrixPCL;
typedef DepthUndistortionImpl<GUMatrixModel, DepthEigen> GUMatrixEigen;

typedef PolynomialUndistortionMatrixFitPCL<GlobalPolynomial, PCLPoint3> GUMatrixFitPCL;
typedef PolynomialUndistortionMatrixFitEigen<GlobalPolynomial> GUMatrixFitEigen;

typedef PolynomialUndistortionFunctionModel<GlobalPolynomial>::Data UFunctionData;
typedef PolynomialUndistortionFunctionPCL<GlobalPolynomial, PCLPoint3> UFunctionPCL;
typedef PolynomialUndistortionFunctionEigen<GlobalPolynomial> UFunctionEigen;
typedef PolynomialUndistortionFunctionFitPCL<GlobalPolynomial, PCLPoint3> UFunctionFitPCL;
typedef PolynomialUndistortionFunctionFitEigen<GlobalPolynomial> UFunctionFitEigen;

typedef TwoStepsModel<Scalar, LUMatrixModel, GUMatrixModel> UndistortionModel;
typedef DepthUndistortionImpl<UndistortionModel, DepthEigen> UndistortionEigen;
typedef DepthUndistortionImpl<UndistortionModel, DepthPCL> UndistortionPCL;

//typedef TwoStepsUndistortionEigen<Scalar, LUMatrixEigen, GUMatrixEigen> UndistortionEigen;
//typedef TwoStepsUndistortionPCL<Scalar, PCLPoint3, LUMatrixPCL, GUMatrixPCL> UndistortionPCL;

} /* namespace calibration */
#endif /* RGBD_CALIBRATION_GLOBALS_H_ */
