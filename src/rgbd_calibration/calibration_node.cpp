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
  checkerboards_sub_ = node_handle_.subscribe("checkerboard_array",
                                              1,
                                              &CalibrationNode::checkerboardArrayCallback,
                                              this);

  if (not node_handle_.getParam("camera_calib_url", camera_calib_url_))
    ROS_FATAL("Missing \"camera_calib_url\" parameter!!");

  node_handle_.param("camera_name", camera_name_, std::string("camera"));

  node_handle_.param("estimate_depth_distortion", estimate_depth_distortion_, true); // TODO default = false
  //  node_handle_.param("distortion/cols", distortion_cols_, 640); // TODO rename
  //  node_handle_.param("distortion/rows", distortion_rows_, 480); // TODO rename
  node_handle_.param("distortion/cols", distortion_cols_, 80); // TODO rename
  node_handle_.param("distortion/rows", distortion_rows_, 60); // TODO rename
  node_handle_.param("downsample_ratio", downsample_ratio_, 1);
  if (downsample_ratio_ < 1)
  {
    downsample_ratio_ = 1;
    ROS_WARN("\"downsample_ratio\" cannot be < 1. Skipping.");
  }

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
  Polynomial<Scalar, 2> depth_error_function(KINECT_ERROR_POLY); // TODO add parameter
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

  if (estimate_depth_distortion_)
  {
    const Scalar fov_x = Scalar(DEG2RAD(KINECT_FOV_X)); // TODO add parameter
    const Scalar fov_y = Scalar(DEG2RAD(KINECT_FOV_Y)); // TODO add parameter

    LUMatrixModel::Data::Ptr local_und_data =
      boost::make_shared<LUMatrixModel::Data>(distortion_cols_,
                                              distortion_rows_,
                                              LocalPolynomial::IdentityCoefficients());

    LUMatrixModel::Ptr local_und_model = boost::make_shared<LUMatrixModel>();
    local_und_model->setData(local_und_data);
    local_und_model->setFieldOfView(fov_x, fov_y);

//    LUMatrixPCL::Ptr local_und = boost::make_shared<LUMatrixPCL>();
//    local_und->setModel(local_und_model);

    GUMatrixModel::Data::Ptr global_und_data =
      boost::make_shared<GUMatrixModel::Data>(2, 2, GlobalPolynomial::IdentityCoefficients());

    GUMatrixModel::Ptr global_und_model = boost::make_shared<GUMatrixModel>();
    global_und_model->setData(global_und_data);
    global_und_model->setFieldOfView(2 * fov_x, 2 * fov_y);

//    GUMatrixPCL::Ptr global_und = boost::make_shared<GUMatrixPCL>();
//    global_und->setModel(global_und_model);

    UndistortionModel::Ptr model = boost::make_shared<UndistortionModel>();
    model->setLocalModel(local_und_model);
    model->setGlobalModel(global_und_model);

//    UndistortionPCL::Ptr undistortion = boost::make_shared<UndistortionPCL>();
//    undistortion->setModel(model);

    depth_sensor_->setUndistortionModel(model);

    calibration_->setLocalUndistortionModel(local_und_model);
    calibration_->setGlobalUndistortionModel(global_und_model);
    calibration_->setEstimateDepthUndistortionModel(true);
  }

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
