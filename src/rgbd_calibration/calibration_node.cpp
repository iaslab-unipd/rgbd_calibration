#include <ros/ros.h>

#include <pcl/io/pcd_io.h>

#include <eigen_conversions/eigen_msg.h>

#include <calibration_common/pinhole/camera_model.h>
#include <camera_info_manager/camera_info_manager.h>

#include <rgbd_calibration/calibration_node.h>

using namespace camera_info_manager;
using namespace calibration_msgs;

namespace calibration
{

CalibrationNode::CalibrationNode(ros::NodeHandle & node_handle)
  : node_handle_(node_handle),
    publisher_(new Publisher(node_handle)),
    has_initial_transform_(false)
{
  checkerboards_sub_ = node_handle_.subscribe("checkerboard_array", 1, &CalibrationNode::checkerboardArrayCallback, this);

  if (not node_handle_.getParam("camera_calib_url", camera_calib_url_))
    ROS_FATAL("Missing \"camera_calib_url\" parameter!!");

  node_handle_.param("camera_name", camera_name_, std::string("camera"));

  int undistortion_matrix_cell_size_x, undistortion_matrix_cell_size_y;
  node_handle_.param("undistortion_matrix/cell_size_x", undistortion_matrix_cell_size_x, 8);
  node_handle_.param("undistortion_matrix/cell_size_y", undistortion_matrix_cell_size_y, 8);
  undistortion_matrix_cell_size_.x() = undistortion_matrix_cell_size_x;
  undistortion_matrix_cell_size_.y() = undistortion_matrix_cell_size_y;

  int images_size_x, images_size_y;
  node_handle_.param("depth_image/cols", images_size_x, 640);
  node_handle_.param("depth_image/rows", images_size_y, 480);
  images_size_.x() = images_size_x;
  images_size_.y() = images_size_y;

  node_handle_.param("downsample_ratio", downsample_ratio_, 1);
  if (downsample_ratio_ < 1)
  {
    downsample_ratio_ = 1;
    ROS_WARN("\"downsample_ratio\" cannot be < 1. Skipping.");
  }
  else
  {
    images_size_ /= downsample_ratio_;
  }

  if (not node_handle_.getParam("depth_error_function", depth_error_coeffs_))
    ROS_FATAL("Missing \"depth_error_function\" parameter!!");
  else if (depth_error_coeffs_.size() != 3)
    ROS_FATAL("\"depth_error_function\" must be a vector of size 3!!");

  if (node_handle_.hasParam("camera_pose"))
  {
    Translation3 translation;
    Quaternion rotation;

    node_handle_.getParam("camera_pose/translation/x", translation.vector().coeffRef(0));
    node_handle_.getParam("camera_pose/translation/y", translation.vector().coeffRef(1));
    node_handle_.getParam("camera_pose/translation/z", translation.vector().coeffRef(2));

    node_handle_.getParam("camera_pose/rotation/x", rotation.coeffs().coeffRef(0));
    node_handle_.getParam("camera_pose/rotation/y", rotation.coeffs().coeffRef(1));
    node_handle_.getParam("camera_pose/rotation/z", rotation.coeffs().coeffRef(2));
    node_handle_.getParam("camera_pose/rotation/w", rotation.coeffs().coeffRef(3));

    initial_transform_ = translation * rotation;
    has_initial_transform_ = true;
  }

}

bool CalibrationNode::initialize()
{
  if (not waitForMessages())
    return false;

  calibration_ = boost::make_shared<Calibration>();

  for (size_t i = 0; i < checkerboard_array_msg_->checkerboards.size(); ++i)
    cb_vec_.push_back(createCheckerboard(checkerboard_array_msg_->checkerboards[i], i));

  BaseObject::Ptr world = boost::make_shared<BaseObject>();
  world->setFrameId("/world");

  depth_sensor_ = boost::make_shared<KinectDepthSensor<UndistortionModel> >();
  depth_sensor_->setFrameId("/depth_sensor");
  depth_sensor_->setParent(world);
  Polynomial<Scalar, 2> depth_error_function(Vector3(depth_error_coeffs_[0], depth_error_coeffs_[1], depth_error_coeffs_[2]));
  depth_sensor_->setDepthErrorFunction(depth_error_function);

  CameraInfoManager manager(node_handle_, camera_name_, camera_calib_url_);
  PinholeCameraModel::ConstPtr pinhole_model = boost::make_shared<PinholeCameraModel>(manager.getCameraInfo());

  color_sensor_ = boost::make_shared<PinholeSensor>();
  color_sensor_->setFrameId("/pinhole_sensor");
  color_sensor_->setCameraModel(pinhole_model);
  if (has_initial_transform_)
  {
    color_sensor_->setParent(depth_sensor_);
    color_sensor_->transform(initial_transform_);
  }

  calibration_->setCheckerboards(cb_vec_);
  calibration_->setDepthSensor(depth_sensor_);
  calibration_->setColorSensor(color_sensor_);

  if (not has_initial_transform_)
    calibration_->setEstimateInitialTransform(true);

  LocalModel::Ptr local_model = boost::make_shared<LocalModel>(images_size_);
  LocalModel::Data::Ptr local_matrix = local_model->createMatrix(undistortion_matrix_cell_size_, LocalPolynomial::IdentityCoefficients());
  local_model->setMatrix(local_matrix);


  GlobalModel::Ptr global_model = boost::make_shared<GlobalModel>(images_size_);
  GlobalModel::Data::Ptr global_data = boost::make_shared<GlobalModel::Data>(Size2(2, 2), GlobalPolynomial::IdentityCoefficients());
  global_model->setMatrix(global_data);

  UndistortionModel::Ptr model = boost::make_shared<UndistortionModel>();
  model->setLocalModel(local_model);
  model->setGlobalModel(global_model);

  depth_sensor_->setUndistortionModel(model);

  calibration_->setLocalModel(local_model);
  calibration_->setGlobalModel(global_model);
  calibration_->initDepthUndistortionModel();
  calibration_->setPublisher(publisher_);
  calibration_->setDownSampleRatio(downsample_ratio_);

  return true;
}

void CalibrationNode::checkerboardArrayCallback(const CheckerboardArray::ConstPtr & msg)
{
  checkerboard_array_msg_ = msg;
}

bool CalibrationNode::waitForMessages() const
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

Checkerboard::Ptr CalibrationNode::createCheckerboard(const CheckerboardMsg::ConstPtr & msg,
                                                      int id)
{
  return createCheckerboard(*msg, id);
}

Checkerboard::Ptr CalibrationNode::createCheckerboard(const CheckerboardMsg & msg,
                                                      int id)
{
  std::stringstream ss;
  ss << "/checkerboard_" << id;
  Checkerboard::Ptr cb = boost::make_shared<Checkerboard>(msg.cols, msg.rows, msg.cell_width, msg.cell_height);
  cb->setFrameId(ss.str());
  return cb;
}

} /* namespace calibration */
