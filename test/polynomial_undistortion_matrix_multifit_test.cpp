#include <calibration_common/base/math.h>
#include <rgbd_calibration/globals.h>
#include <kinect/depth/polynomial_matrix_fit.h>
#include <gtest/gtest.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

//using namespace calibration;

//typedef PolynomialUndistortionMatrixEigen<Polynomial<Types::Scalar, 2, 0> > UMap;
//typedef PolynomialUndistortionMatrixFitEigen<Polynomial<Types::Scalar, 2, 0> > UMapFit;

//TEST(PolynomialUndistortionMatrixFit, PolynomialUndistortionMatrixFit)
//{
//  UMapFit u_map(10, 10, M_PI / 3.0, M_PI / 5.0);
//  SUCCEED();
//}
//
//TEST(PolynomialUndistortionMatrixFit, getIndex)
//{
//  UMapFit u_map(10, 10, M_PI / 3.0, M_PI / 5.0);
//  size_t x, y;
//  u_map.getIndex(UMap::toSphericalCoordinates(Types::Point3(0.01, -0.01, 1.0)), x, y);
//  EXPECT_EQ(5, x);
//  EXPECT_EQ(5, y);
//}
//
//TEST(PolynomialUndistortionMatrixFit, updateDistortionMap)
//{
//  UMapFit u_map(10, 10, M_PI / 3.0, M_PI / 5.0);
//
//  // applied distortion: z_d = c3 * z_p*z_p + c2 * z_p + c1
//  Types::Scalar c0 = 1.0;
//  Types::Scalar c1 = -2.0;
//  Types::Scalar c2 = 1.0;
//
//  // gaussian noise on the detected point, linear with respect to the z coordinate.
//  boost::mt19937 rnd_gen;
//  boost::normal_distribution<double> norm_dist(0.0, 1e-4);
//  boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > normal_gen(rnd_gen, norm_dist);
//
//  // update ignore bins with less than 2*Dimension (6) points.
//  for (Types::Scalar z = 1.0; z <= 10.0; z++)
//    for (int n = 0; n < 10; n++)
//      u_map.addPoint(Types::Point3(0.0, 0.0, z), Types::Plane(-Eigen::Vector3d::UnitZ(), c2 * z * z + (c1 + normal_gen()) * z + c0));
//  u_map.update();
//
//  size_t x, y;
//  u_map.getIndex(UMap::toSphericalCoordinates(Types::Point3(0.0, 0.0, 1.0)), x, y);
//  EXPECT_NEAR(u_map.polynomialAt(x, y).coefficients()[0], c0, 1e-2);
//  EXPECT_NEAR(u_map.polynomialAt(x, y).coefficients()[1], c1, 1e-2);
//  EXPECT_NEAR(u_map.polynomialAt(x, y).coefficients()[2], c2, 1e-2);
//}
//
//TEST(PolynomialUndistortionMatrixFit, accumulatePoint)
//{
//  Types::Scalar c0 = 1.0;
//  Types::Scalar c1 = -2.0;
//  Types::Scalar c2 = 1.0;
//
//  boost::mt19937 rnd_gen;
//  boost::normal_distribution<double> norm_dist(0.0, 1e-4);
//  boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > normal_gen(rnd_gen, norm_dist);
//
//  UMapFit u_map(10, 10, M_PI / 3.0, M_PI / 5.0);
//  UMapFit u_map2(10, 10, M_PI / 3.0, M_PI / 5.0);
//
//  for (double z = 1.0; z <= 10.0; z++)
//  {
//    for (int n = 0; n < 10; n++)
//    {
//      Types::Plane plane(-Eigen::Vector3d::UnitZ(), c2 * z * z + (c1 + normal_gen()) * z + c0);
//      u_map.addPoint(pcl::PointXYZ(0.0, 0.0, z), plane);
//      u_map2.accumulatePoint(pcl::PointXYZ(0.0, 0.0, z));
//      u_map2.addAccumulatedPoints(plane);
//    }
//  }
//  u_map.update();
//  u_map2.update();
//
//  EXPECT_EQ(u_map.polynomialAt(0, 0).coefficients()[0], u_map2.polynomialAt(0, 0).coefficients()[0]);
//  EXPECT_EQ(u_map.polynomialAt(0, 0).coefficients()[1], u_map2.polynomialAt(0, 0).coefficients()[1]);
//  EXPECT_EQ(u_map.polynomialAt(0, 0).coefficients()[2], u_map2.polynomialAt(0, 0).coefficients()[2]);
//}

int main(int argc,
         char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
