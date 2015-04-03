#include <ros/ros.h>

#include <pcl/io/pcd_io.h>

#include <eigen_conversions/eigen_msg.h>

#include <calibration_common/pinhole/camera_model.h>
#include <camera_info_manager/camera_info_manager.h>

#include <kinect/depth/polynomial_matrix_io.h>

#include <rgbd_calibration/test_node.h>

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

    camera_pose_ = translation * rotation;
  }
  else
    ROS_FATAL("Missing \"camera_pose\" parameter!!");

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
  images_size_.x() = images_size_x;
  images_size_.y() = images_size_y;

}

bool TestNode::initialize()
{
  if (not waitForMessages())
    return false;

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

  std::fstream fs;
  fs.open("/tmp/points.txt", std::fstream::out);
  fs.close();

  ROS_INFO("Getting data...");
  for (int i = starting_index_; ros::ok() and i < starting_index_ + instances_; ++i)
  {
    std::stringstream image_file;
    image_file << path_ << image_filename_ << i << "." << image_extension_;

    std::stringstream cloud_file;
    cloud_file << path_ << cloud_filename_ << i << ".pcd";

    cv::Mat image = cv::imread(image_file.str());

    if (not image.data)
      continue;

    PCLCloud3::Ptr cloud(new PCLCloud3);
    pcl::PCDReader pcd_reader;

    if (pcd_reader.read(cloud_file.str(), *cloud) < 0)
      continue;

    test_->addData(image, cloud);
    ++added;

    //rate.sleep();
  }

  ROS_INFO_STREAM("Added " << added << " images + point clouds.");
  if (not only_show_)
  {
    test_->testPlanarityError();
    //test_->testCheckerboardError();
  }

  rate = ros::Rate(1.0);

  while (ros::ok())
  {
    test_->publishData();
    rate.sleep();
  }

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
