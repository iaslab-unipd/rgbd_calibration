#include <ros/ros.h>

#include <pcl/io/pcd_io.h>

#include <eigen_conversions/eigen_msg.h>

#include <calibration_common/pinhole/camera_model.h>
#include <camera_info_manager/camera_info_manager.h>

#include <kinect/depth/polynomial_matrix_io.h>

#include <rgbd_calibration/test_node.h>

//#include <swissranger_camera/utility.h>

using namespace camera_info_manager;
using namespace calibration_msgs;

namespace calibration
{

TestNode::TestNode(ros::NodeHandle & node_handle)
  : node_handle_(node_handle)
{
  checkerboards_sub_ = node_handle_.subscribe("checkerboard_array", 1, &TestNode::checkerboardArrayCallback, this);

  if (not node_handle_.getParam("camera_calib_url", camera_calib_url_))
    ROS_FATAL("Missing \"camera_calib_url\" parameter!!");
  node_handle_.param("camera_name", camera_name_, std::string("camera"));

  if (not node_handle_.getParam("depth_camera_calib_url", depth_camera_calib_url_))
      ROS_FATAL("Missing \"depth_camera_calib_url\" parameter!!");
  node_handle_.param("depth_camera_name", depth_camera_name_, std::string("depth_camera"));

  node_handle_.param("downsample_ratio", downsample_ratio_, 1);
  if (downsample_ratio_ < 1)
  {
    downsample_ratio_ = 1;
    ROS_WARN("\"downsample_ratio\" cannot be < 1. Skipping.");
  }

#if (!defined(UNCALIBRATED)) || (defined(UNCALIBRATED) && defined(SKEW))
  if (node_handle_.hasParam("camera_pose"))
  {
    Translation3 translation;
    Quaternion rotation;

    node_handle_.getParam("camera_pose/position/x", translation.vector().coeffRef(0));
    node_handle_.getParam("camera_pose/position/y", translation.vector().coeffRef(1));
    node_handle_.getParam("camera_pose/position/z", translation.vector().coeffRef(2));

    node_handle_.getParam("camera_pose/orientation/x", rotation.coeffs().coeffRef(0));
    node_handle_.getParam("camera_pose/orientation/y", rotation.coeffs().coeffRef(1));
    node_handle_.getParam("camera_pose/orientation/z", rotation.coeffs().coeffRef(2));
    node_handle_.getParam("camera_pose/orientation/w", rotation.coeffs().coeffRef(3));

    rotation.normalize();

    camera_pose_ = translation * rotation;
  }
  else
    ROS_FATAL("Missing \"camera_pose\" parameter!!");
#else
  camera_pose_ = Eigen::Affine3d::Identity() * Translation3(0.052, 0.0, 0.0);
#endif



  if (node_handle_.hasParam("local_und_matrix_file"))
    node_handle_.getParam("local_und_matrix_file", local_matrix_file_);
  else
    ROS_FATAL("Missing \"local_und_matrix_file\" parameter!!");

  if (node_handle_.hasParam("global_und_matrix_file"))
    node_handle_.getParam("global_und_matrix_file", global_matrix_file_);
  else
    ROS_FATAL("Missing \"global_und_matrix_file\" parameter!!");

  if (not node_handle_.getParam("path", path_))
    ROS_FATAL("Missing \"path\" parameter!!");

  if (path_[path_.size() - 1] != '/')
    path_.append("/");

  bool ok = true;
  if (not node_handle_.hasParam("intrinsics"))
    ROS_FATAL("Missing \"intrinsics\" parameter!!");
  depth_intrinsics_.resize(4);
  ok = ok and node_handle_.getParam("intrinsics/fx", depth_intrinsics_[0]);
  ok = ok and node_handle_.getParam("intrinsics/fy", depth_intrinsics_[1]);
  ok = ok and node_handle_.getParam("intrinsics/cx", depth_intrinsics_[2]);
  ok = ok and node_handle_.getParam("intrinsics/cy", depth_intrinsics_[3]);
#ifdef SKEW
  ok = ok and node_handle_.getParam("intrinsics/s", depth_intrinsics_[4]);
#endif
  if (not ok)
    ROS_FATAL("Malformed \"intrinsics\" parameter!!");

  if (not node_handle_.getParam("instances", instances_))
    ROS_FATAL("Missing \"instances\" parameter!!");

  node_handle_.param("image_extension", image_extension_, std::string("png"));
  node_handle_.param("starting_index", starting_index_, 1);
  node_handle_.param("image_filename", image_filename_, std::string("image_"));
  node_handle_.param("cloud_filename", cloud_filename_, std::string("cloud_"));

  node_handle_.param("only_show", only_show_, false);

  int images_size_x, images_size_y;
  node_handle_.param("depth_image/cols", images_size_x, 640);
  node_handle_.param("depth_image/rows", images_size_y, 480);
  images_size_.x() = images_size_x / downsample_ratio_;
  images_size_.y() = images_size_y / downsample_ratio_;

  std::string depth_type_s;
  node_handle_.param("depth_type", depth_type_s, std::string("none"));
  if (depth_type_s == "kinect1_depth")
    depth_type_ = KINECT1_DEPTH;
  else if (depth_type_s == "swiss_ranger_depth")
    depth_type_ = SWISS_RANGER_DEPTH;
  else
    ROS_FATAL("Missing \"depth_type\" parameter!! Use \"kinect1_depth\" or \"swiss_ranger_depth\"");

}

bool TestNode::initialize()
{
  if (not waitForMessages())
    return false;

  for (size_t i = 0; i < checkerboard_array_msg_->checkerboards.size(); ++i)
    cb_vec_.push_back(createCheckerboard(checkerboard_array_msg_->checkerboards[i], i));

  BaseObject::Ptr world = boost::make_shared<BaseObject>();
  world->setFrameId("/world");

  CameraInfoManager depth_manager(node_handle_, depth_camera_name_, depth_camera_calib_url_);
  sensor_msgs::CameraInfo msg = depth_manager.getCameraInfo();
  /*msg.P[0] = depth_intrinsics_[0];
  msg.P[5] = depth_intrinsics_[1];
  msg.P[2] = depth_intrinsics_[2];
  msg.P[6] = depth_intrinsics_[3];
  msg.K[0] = depth_intrinsics_[0];
  msg.K[4] = depth_intrinsics_[1];
  msg.K[2] = depth_intrinsics_[2];
  msg.K[5] = depth_intrinsics_[3];

#ifdef SKEW
  msg.P[1] = depth_intrinsics_[4];
  msg.K[1] = depth_intrinsics_[4];
#endif*/

  KinectDepthCameraModel::ConstPtr depth_pinhole_model = boost::make_shared<KinectDepthCameraModel>(msg);

  depth_sensor_ = boost::make_shared<KinectDepthSensor<UndistortionModel> >();
  depth_sensor_->setFrameId("/depth_sensor");
  depth_sensor_->setParent(world);
  depth_sensor_->setCameraModel(depth_pinhole_model);
//  Polynomial<Scalar, 2> depth_error_function(KINECT_ERROR_POLY); // TODO add parameter
//  depth_sensor_->setDepthErrorFunction(depth_error_function);

  CameraInfoManager manager(node_handle_, camera_name_, camera_calib_url_);
  msg = manager.getCameraInfo();

  PinholeCameraModel::ConstPtr pinhole_model = boost::make_shared<PinholeCameraModel>(manager.getCameraInfo());
  cv::Mat P = cv::getOptimalNewCameraMatrix(depth_pinhole_model->fullIntrinsicMatrix(), depth_pinhole_model->distortionCoeffs(),
                                            depth_pinhole_model->fullResolution(), 0);

  std::cout << P << std::endl;

  color_sensor_ = boost::make_shared<PinholeSensor>();
  color_sensor_->setFrameId("/pinhole_sensor");
  color_sensor_->setCameraModel(pinhole_model);
  color_sensor_->setParent(depth_sensor_);
  color_sensor_->transform(camera_pose_);

  LocalModel::Data::Ptr local_und_data;

  PolynomialUndistortionMatrixIO<LocalPolynomial> local_io;
  if (not local_io.read(local_und_data, local_matrix_file_))
    ROS_FATAL_STREAM("File " << local_matrix_file_ << " not found!!");

  GlobalModel::Data::Ptr global_data;

  PolynomialUndistortionMatrixIO<GlobalPolynomial> global_io;
  if (not global_io.read(global_data, global_matrix_file_))
    ROS_FATAL_STREAM("File " << global_matrix_file_ << " not found!!");

  LocalModel::Ptr local_model = boost::make_shared<LocalModel>(images_size_);
  local_model->setMatrix(local_und_data);

  LocalMatrixPCL::Ptr local_und = boost::make_shared<LocalMatrixPCL>();
  local_und->setModel(local_model);

  GlobalModel::Ptr global_model = boost::make_shared<GlobalModel>(images_size_);
  global_model->setMatrix(global_data);

  GlobalMatrixPCL::Ptr global_matrix = boost::make_shared<GlobalMatrixPCL>();
  global_matrix->setModel(global_model);

  UndistortionModel::Ptr model = boost::make_shared<UndistortionModel>();
  model->setLocalModel(local_model);
  model->setGlobalModel(global_model);

//  UndistortionPCL::Ptr undistortion = boost::make_shared<UndistortionPCL>();
//  undistortion->setModel(model);

  depth_sensor_->setUndistortionModel(model);

  publisher_ = boost::make_shared<Publisher>(node_handle_);

  test_ = boost::make_shared<CalibrationTest>();

  test_->setCheckerboards(cb_vec_);
  test_->setDepthSensor(depth_sensor_);
  test_->setColorSensor(color_sensor_);
  test_->setLocalModel(local_model);
  test_->setGlobalModel(global_model);
  test_->setPublisher(publisher_);
  test_->setDownSampleRatio(downsample_ratio_);

  return true;
}

void TestNode::checkerboardArrayCallback(const CheckerboardArray::ConstPtr & msg)
{
  checkerboard_array_msg_ = msg;
}

bool TestNode::waitForMessages() const
{
  ros::Rate rate(1.0);
  ros::spinOnce();
  while (ros::ok() and (not checkerboard_array_msg_))
  {
    ROS_WARN("Not all messages received!");
    rate.sleep();
    ros::spinOnce();
  }
  return checkerboard_array_msg_;
}

Checkerboard::Ptr TestNode::createCheckerboard(const CheckerboardMsg::ConstPtr & msg,
                                               int id)
{
  return createCheckerboard(*msg, id);
}

Checkerboard::Ptr TestNode::createCheckerboard(const CheckerboardMsg & msg,
                                               int id)
{
  std::stringstream ss;
  ss << "/checkerboard_" << id;
  Checkerboard::Ptr cb = boost::make_shared<Checkerboard>(msg.cols, msg.rows, msg.cell_width, msg.cell_height);
  cb->setFrameId(ss.str());
  return cb;
}

void TestNode::spin()
{
  ros::Rate rate(10.0);

  int added = 0;

  /*std::fstream fs;
  fs.open("/tmp/points.txt", std::fstream::out);
  fs.close();*/

  ROS_INFO("Getting data...");
  for (int i = starting_index_; ros::ok() and i < starting_index_ + instances_; ++i)
  {

    std::stringstream image_file, cloud_file, depth_file;
    image_file << path_ << image_filename_ << (i < 10 ? "000" : (i < 100 ? "00" : (i < 1000 ? "0" : ""))) << i << "." << image_extension_;
    cloud_file << path_ << cloud_filename_ << (i < 10 ? "000" : (i < 100 ? "00" : (i < 1000 ? "0" : ""))) << i << ".pcd";
    depth_file << path_ << "depth_" << (i < 10 ? "000" : (i < 100 ? "00" : (i < 1000 ? "0" : ""))) << i << ".png";

    cv::Mat image = cv::imread(image_file.str());

    if (not image.data)
      continue;

    PCLCloud3::Ptr cloud;

    pcl::PCDReader pcd_reader;
    if (depth_type_ == KINECT1_DEPTH)
    {
      cloud = boost::make_shared<PCLCloud3>();

      if (pcd_reader.read(cloud_file.str(), *cloud) < 0)
      {
        ROS_WARN_STREAM(cloud_file.str() << " not valid!");
        continue;
      }

      /*cv::Mat depth = cv::imread(depth_file.str(), 2);
      cloud->width = depth.cols;
      cloud->height = depth.rows;
      cloud->points.resize(cloud->width * cloud->height);
      cloud->is_dense = false;

      for (int k = 0; k < cloud->width; ++k)
      {
        for (int j = 0; j < cloud->height; ++j)
        {
          if (depth.at<uint16_t>(j, k) > 0)
            cloud->at(k, j).z = depth.at<uint16_t>(j, k) / 1000.0f;
          else
            cloud->at(k, j).z = std::numeric_limits<float>::quiet_NaN();
        }
      }*/

    }

#ifdef HERRERA
    PCLCloud3::Ptr cloud_ = PCLCloud3::Ptr(new PCLCloud3(cloud->width, cloud->height));
    cloud_->header = cloud->header;
    cloud_->is_dense = cloud->is_dense;
    for (size_t p_i = 0; p_i < cloud->points.size(); ++p_i)
    {
      int row = p_i % cloud->height;
      int column = p_i / cloud->height;
      int index = row * cloud->width + column;
      cloud_->points[index].x = cloud->points[p_i].x;
      cloud_->points[index].y = cloud->points[p_i].y;
      cloud_->points[index].z = cloud->points[p_i].z;

    }
    cloud = cloud_;
#else
#  if (!defined(UNCALIBRATED)) || (defined(UNCALIBRATED) && defined(SKEW))
    //std::cout << cv::getOptimalNewCameraMatrix(color_sensor_->cameraModel()->intrinsicMatrix(), color_sensor_->cameraModel()->distortionCoeffs(),
    //		color_sensor_->cameraModel()->fullResolution(), 0) << std::endl;

    const Scalar & fx = depth_intrinsics_[0];
    const Scalar & fy = depth_intrinsics_[1];
    const Scalar & cx = depth_intrinsics_[2];
    const Scalar & cy = depth_intrinsics_[3];
#    ifdef SKEW
    const Scalar & s = depth_intrinsics_[4];
#    endif
    
    for (int k = 0; k < cloud->width; ++k)
    {
      for (int j = 0; j < cloud->height; ++j)
      {
#    ifdef SKEW
        // x = (z*(k*fy - cx*fy - j*s + cy*s))/(fx*fy)
        cloud->at(k, j).x = (k*fy - cx*fy - j*s + cy*s) * cloud->at(k, j).z / (fx*fy);
#    else
        cloud->at(k, j).x = (k - cx) * cloud->at(k, j).z / fx;
#    endif
        cloud->at(k, j).y = (j - cy) * cloud->at(k, j).z / fy;
      }
    }
#  endif
#endif


    PCLCloudRGB::Ptr cloud_rgb = test_->addData(image, cloud);
    ++added;

    std::stringstream cloud_rgb_file;
    cloud_rgb_file << path_ << cloud_filename_ << (i < 10 ? "000" : "00") << i << "_rgb.pcd";

    pcl::PCDWriter pcd_writer;
    if (depth_type_ == KINECT1_DEPTH)
    {
      if (pcd_writer.write(cloud_rgb_file.str(), *cloud_rgb) < 0)
      {
        ROS_WARN_STREAM(cloud_rgb_file.str() << " not valid!");
        continue;
      }
    }

    //rate.sleep();
  }
  ROS_INFO_STREAM("Added " << added << " images + point clouds.");
  if (not only_show_)
  {
//    test_->visualizeClouds();
//    test_->testPlanarityError();
//    test_->testCheckerboardError();
    test_->testCube();
  }


  rate = ros::Rate(1.0);

  while (ros::ok())
  {
    test_->publishData();
    rate.sleep();
  }

}

void TestNode::spin2()
{
  ros::Rate rate(10.0);

  ROS_INFO("Getting data...");
  for (int i = 1000; ros::ok() and i < 5000; ++i)
  {
    std::stringstream depth_file, depth_file_und;
    depth_file << path_ << "depth_" << (i < 10 ? "000" : (i < 100 ? "00" : (i < 1000 ? "0" : ""))) << i << ".png";
    depth_file_und << path_ << "und/depth_" << (i < 10 ? "000" : (i < 100 ? "00" : (i < 1000 ? "0" : ""))) << i << ".png";

    cv::Mat depth = cv::imread(depth_file.str(), 2);

    if (not depth.data)
      continue;

    for (int k = 0; k < depth.rows; ++k)
    {
      for (Size1 j = 0; j < depth.cols; ++j)
      {
        Scalar z = depth.at<uint16_t>(k, j) / Scalar(1000.0);
        if (z == 0)
          continue;

        depth_sensor_->undistortionModel()->localModel()->undistort(j, k, z);
        depth_sensor_->undistortionModel()->globalModel()->undistort(j, k, z);

        depth.at<uint16_t>(k, j) = cv::saturate_cast<uint16_t>(cvRound(z * 1000));
      }
    }

    cv::imwrite(depth_file_und.str(), depth);

  }
  ROS_INFO("OK");
}

void TestNode::spin3 ()
{
  ros::Rate rate(10.0);

  const Scalar & fx = depth_intrinsics_[0];
  const Scalar & fy = depth_intrinsics_[1];
  const Scalar & cx = depth_intrinsics_[2];
  const Scalar & cy = depth_intrinsics_[3];

  Transform t_original = Eigen::Affine3d::Identity() * Translation3(-0.025, 0.0, 0.0);
  Transform t = color_sensor_->pose().inverse();

  ROS_INFO("Getting data...");
  for (int i = 0; ros::ok() and i < 5000; ++i)
  {
    std::stringstream depth_file_2, depth_file, depth_file_und;
    depth_file << path_ << "depth_" << (i < 10 ? "000" : (i < 100 ? "00" : (i < 1000 ? "0" : ""))) << i << ".png";
    depth_file_2 << path_ << "alberto/depth_" << (i < 10 ? "000" : (i < 100 ? "00" : (i < 1000 ? "0" : ""))) << i << ".png";
    depth_file_und << path_ << "alberto/und/depth_" << (i < 10 ? "000" : (i < 100 ? "00" : (i < 1000 ? "0" : ""))) << i << ".png";

    cv::Mat depth = cv::imread(depth_file.str(), 2);

    if (not depth.data)
      continue;

    cv::Mat_<uint16_t> depth_2 = cv::Mat(depth.size(), 0);
    cv::Mat_<uint16_t> depth_und = cv::Mat(depth.size(), 0);

    for (int k = 0; k < depth.rows; ++k)
    {
      for (int j = 0; j < depth.cols; ++j)
      {
        Scalar z = depth.at<uint16_t>(k, j) / Scalar(1000.0);

        if (z > 0)
        {

          Point3 point_eigen;
          Point3 point_eigen_original;

          point_eigen.x() = (j - cx) * z / fx;
          point_eigen.y() = (k - cy) * z / fy;
          point_eigen.z() = z;

          point_eigen_original = t_original * point_eigen;
          Point2 point_image_original = color_sensor_->cameraModel()->project3dToPixel(point_eigen_original);

          uint16_t new_z_original = cv::saturate_cast<uint16_t>(cvRound(point_eigen_original.z() * 1000));
          int x = cvRound(point_image_original[0]);
          int y = cvRound(point_image_original[1]);

          if ((x >= 0 and x < depth.cols and y >= 0 and y < depth.rows) and
              (depth_2.at<uint16_t>(y, x) == 0 or new_z_original < depth_2.at<uint16_t>(y, x)))
            depth_2.at<uint16_t>(y, x) = new_z_original;


          depth_sensor_->undistortionModel()->localModel()->undistort(j, k, z);
          depth_sensor_->undistortionModel()->globalModel()->undistort(j, k, z);
          point_eigen.x() = (j - cx) * z / fx;
          point_eigen.y() = (k - cy) * z / fy;
          point_eigen.z() = z;

          point_eigen = t * point_eigen;
          Point2 point_image = color_sensor_->cameraModel()->project3dToPixel(point_eigen);

          uint16_t new_z = cv::saturate_cast<uint16_t>(cvRound(point_eigen.z() * 1000));
          x = cvRound(point_image[0]);
          y = cvRound(point_image[1]);

          if ((x >= 0 and x < depth.cols and y >= 0 and y < depth.rows) and
              (depth_und.at<uint16_t>(y, x) == 0 or new_z < depth_und.at<uint16_t>(y, x)))
            depth_und.at<uint16_t>(y, x) = new_z;

        }
      }
    }

    cv::imwrite(depth_file_und.str(), depth_und);
    cv::imwrite(depth_file_2.str(), depth_2);

  }
  ROS_INFO("OK");
}


} /* namespace calibration */

int main(int argc,
         char ** argv)
{
  ros::init(argc, argv, "test_calibration");
  ros::NodeHandle node_handle("~");

  try
  {
    calibration::TestNode test_node(node_handle);
    if (not test_node.initialize())
      return 0;
    test_node.spin();

    ros::spin();
  }
  catch (std::runtime_error & error)
  {
    ROS_FATAL("Test node error: %s", error.what());
    return 1;
  }

  return 0;
}
