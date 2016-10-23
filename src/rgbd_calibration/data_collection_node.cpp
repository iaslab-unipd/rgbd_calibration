#include <ros/ros.h>
#include <ros/topic.h>

#include <pcl/io/pcd_io.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//#include <pcl/point_types.h>
//#include <pcl_ros/point_cloud.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <calibration_common/pinhole/pinhole.h>
#include <calibration_common/objects/checkerboard.h>

#include <calibration_common/algorithms/automatic_checkerboard_finder.h>

#include <camera_info_manager/camera_info_manager.h>

#include <calibration_msgs/CheckerboardArray.h>
#include <calibration_msgs/CheckerboardMsg.h>

#include <rgbd_calibration/Acquisition.h>

using namespace camera_info_manager;
using namespace calibration_msgs;

using rgbd_calibration::Acquisition;

namespace calibration
{

class DataCollectionNode
{
public:

  enum
  {
    SAVE_NONE = 0,
    SAVE_IMAGE = 1,
    SAVE_DEPTH_IMAGE = 2,
    SAVE_POINT_CLOUD = 4,
    SAVE_IMAGE_CAMERA_INFO = 8,
    SAVE_DEPTH_CAMERA_INFO = 16,
    SAVE_ALL = 31
  };

  enum DepthType
  {
    DEPTH_FLOAT32,
    DEPTH_UINT16
  };

  DataCollectionNode(ros::NodeHandle & node_handle);

  virtual
  ~DataCollectionNode();

  bool
  initialize();

  static Checkerboard::Ptr
  createCheckerboard(const CheckerboardMsg::ConstPtr & msg,
                     int id);

  static Checkerboard::Ptr
  createCheckerboard(const CheckerboardMsg & msg,
                     int id);

protected:

  void
  save(const sensor_msgs::CameraInfo::ConstPtr & camera_info,
       const std::string & file_name);

  void
  save(const pcl::PCLPointCloud2::ConstPtr & cloud,
       const std::string & file_name);

  void
  save(const cv::Mat & image,
       const std::string & file_name);

  void
  saveDepth(const cv::Mat & depth_image,
            const std::string & file_name);

  void
  pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr & msg);

  void
  imageCallback(const sensor_msgs::Image::ConstPtr & msg);

  void
  depthImageCallback(const sensor_msgs::Image::ConstPtr & msg);

  void
  cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & msg);

  void
  depthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & msg);

  void
  actionCallback(const Acquisition::ConstPtr & msg);

  void
  checkerboardArrayCallback(const calibration_msgs::CheckerboardArray::ConstPtr & msg);

  bool
  waitForMessages();

  /* variables */

  ros::NodeHandle node_handle_;
  image_transport::ImageTransport image_transport_;

  ros::Subscriber cloud_sub_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_image_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Subscriber depth_camera_info_sub_;

  ros::Subscriber omnicamera_info_sub_;
  ros::Subscriber action_sub_;

  pcl::PCLPointCloud2::Ptr cloud_msg_;
  sensor_msgs::Image::ConstPtr image_msg_;
  sensor_msgs::Image::ConstPtr depth_image_msg_;
  sensor_msgs::CameraInfo::ConstPtr camera_info_msg_;
  sensor_msgs::CameraInfo::ConstPtr depth_camera_info_msg_;

// TODO find another way to get checkerboards
  ros::Subscriber checkerboards_sub_;
  calibration_msgs::CheckerboardArray::ConstPtr checkerboard_array_msg_;

  std::vector<Checkerboard::ConstPtr> cb_vec_;
  std::vector<std::vector<cv::Point2f> > last_corners_;
  std::vector<std::vector<cv::Point2f> > last_saved_corners_;

  int starting_index_;
  int file_index_;

  std::string save_folder_;
  std::string image_extension_;
  std::string image_filename_;
  std::string depth_filename_;
  std::string cloud_filename_;

  bool search_checkerboard_;

  int save_flags_;
  DepthType depth_type_;

};

DataCollectionNode::DataCollectionNode(ros::NodeHandle & node_handle)
  : node_handle_(node_handle),
    image_transport_(node_handle),
    search_checkerboard_(false)
{
  checkerboards_sub_ = node_handle_.subscribe("checkerboard_array", 1, &DataCollectionNode::checkerboardArrayCallback, this);

  action_sub_ = node_handle.subscribe("action", 1, &DataCollectionNode::actionCallback, this);

  node_handle_.param("starting_index", starting_index_, 1);
  if (not node_handle_.getParam("save_folder", save_folder_))
    ROS_FATAL_STREAM("[" << ros::this_node::getName() << "] Missing folder!!");

  if (save_folder_.at(save_folder_.size() - 1) != '/')
    save_folder_.append("/");

  file_index_ = starting_index_;

  node_handle_.param("image_extension", image_extension_, std::string("png"));
  node_handle_.param("image_filename", image_filename_, std::string("image_"));
  node_handle_.param("depth_filename", depth_filename_, std::string("depth_"));
  node_handle_.param("cloud_filename", cloud_filename_, std::string("cloud_"));

  node_handle_.param("search_checkerboard", search_checkerboard_, false);

  bool save_image, save_image_camera_info;
  node_handle_.param("save_image", save_image, false);
  node_handle_.param("save_image_camera_info", save_image_camera_info, false);

  bool save_depth_image, save_depth_camera_info;
  node_handle_.param("save_depth_image", save_depth_image, false);
  node_handle_.param("save_depth_camera_info", save_depth_camera_info, false);

  bool save_point_cloud;
  node_handle_.param("save_point_cloud", save_point_cloud, false);

  save_flags_ = 0;
  save_flags_ |= save_image ? SAVE_IMAGE : 0;
  save_flags_ |= save_image_camera_info ? SAVE_IMAGE_CAMERA_INFO : 0;
  save_flags_ |= save_depth_image ? SAVE_DEPTH_IMAGE : 0;
  save_flags_ |= save_depth_camera_info ? SAVE_DEPTH_CAMERA_INFO : 0;
  save_flags_ |= save_point_cloud ? SAVE_POINT_CLOUD : 0;

  if (save_flags_ & SAVE_POINT_CLOUD)
    cloud_sub_ = node_handle.subscribe("point_cloud", 1, &DataCollectionNode::pointCloudCallback, this);

  if (save_flags_ & SAVE_IMAGE)
    image_sub_ = image_transport_.subscribe("image", 1, &DataCollectionNode::imageCallback, this);

  if (save_flags_ & SAVE_DEPTH_IMAGE)
    depth_image_sub_ = image_transport_.subscribe("depth_image", 1, &DataCollectionNode::depthImageCallback, this);

  if (save_flags_ & SAVE_IMAGE_CAMERA_INFO)
    camera_info_sub_ = node_handle.subscribe("camera_info", 1, &DataCollectionNode::cameraInfoCallback, this);

  if (save_flags_ & SAVE_DEPTH_CAMERA_INFO)
    depth_camera_info_sub_ = node_handle.subscribe("depth_camera_info", 1, &DataCollectionNode::depthCameraInfoCallback, this);

  std::string depth_type_s;
  node_handle_.param("depth_type", depth_type_s, std::string("float32"));
  if (depth_type_s == std::string("float32"))
    depth_type_ = DEPTH_FLOAT32;
  else if (depth_type_s == std::string("uint16"))
    depth_type_ = DEPTH_UINT16;
  else
    ROS_FATAL_STREAM("[" << ros::this_node::getName() << "] Wrong \"depth_type\" parameter. Use \"float32\" or \"uint16\".");

}

DataCollectionNode::~DataCollectionNode()
{
  // Do nothing
}

bool DataCollectionNode::initialize()
{
  if (not waitForMessages())
    return false;

  ros::spinOnce();

  if (search_checkerboard_)
  {
    for (size_t i = 0; i < checkerboard_array_msg_->checkerboards.size(); ++i)
      cb_vec_.push_back(createCheckerboard(checkerboard_array_msg_->checkerboards[i], i));
  }

  return true;
}

void DataCollectionNode::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  cloud_msg_ = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2());
  pcl_conversions::toPCL(*msg, *cloud_msg_);
}

void DataCollectionNode::imageCallback(const sensor_msgs::Image::ConstPtr & msg)
{
  image_msg_ = msg;
}

void DataCollectionNode::depthImageCallback(const sensor_msgs::Image::ConstPtr & msg)
{
  depth_image_msg_ = msg;
}

void DataCollectionNode::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & msg)
{
  camera_info_msg_ = msg;
}

void DataCollectionNode::depthCameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & msg)
{
  depth_camera_info_msg_ = msg;
}

void DataCollectionNode::actionCallback(const Acquisition::ConstPtr & msg)
{

  try
  {
    std::stringstream info_file_name;
    info_file_name << save_folder_ << "info.yaml";
    std::stringstream file_index_ss;
    file_index_ss << std::setw(4) << std::setfill('0') << file_index_;

    std::ofstream file;
    file.open(info_file_name.str().c_str(), file_index_ == 1 ? std::ios_base::out : std::ios_base::out | std::ios_base::app);
    if (file_index_ == 1)
      file << "data:" << std::endl;
    file << "  - id: " << file_index_ss.str() << std::endl;
    file << "    timestamp_image: " << std::setprecision(19) << image_msg_->header.stamp.toSec() << std::setprecision(6) << std::endl;
    file << "    timestamp_depth: " << std::setprecision(19) << depth_image_msg_->header.stamp.toSec() << std::setprecision(6) << std::endl;
    //file << "    distance: " << msg->distance << std::endl;
    //file << "    info: \"" << msg->info << "\"" << std::endl;
    file.close();

    if (file_index_ == 1)
    {
      if (save_flags_ & SAVE_IMAGE_CAMERA_INFO)
      {
        std::stringstream camera_info_file_name;
        camera_info_file_name << save_folder_ << image_filename_ << "camera_info.yaml";
        save(camera_info_msg_, camera_info_file_name.str());
      }

      if (save_flags_ & SAVE_DEPTH_CAMERA_INFO)
      {
        std::stringstream depth_camera_info_file_name;
        depth_camera_info_file_name << save_folder_ << depth_filename_ << "camera_info.yaml";
        save(depth_camera_info_msg_, depth_camera_info_file_name.str());
      }
    }

    if (save_flags_ & SAVE_POINT_CLOUD)
    {
      std::stringstream cloud_file_name;
      cloud_file_name << save_folder_ << cloud_filename_ << file_index_ss.str() << ".pcd";
      save(cloud_msg_, cloud_file_name.str());
    }

    if (save_flags_ & SAVE_IMAGE)
    {
      cv_bridge::CvImage::Ptr image_ptr = cv_bridge::toCvCopy(image_msg_);
      std::stringstream image_file_name;
      image_file_name << save_folder_ << image_filename_ << file_index_ss.str() << "." << image_extension_;
      save(image_ptr->image, image_file_name.str());
    }

    if (save_flags_ & SAVE_DEPTH_IMAGE)
    {
      cv_bridge::CvImage::Ptr depth_image_ptr;
      if (depth_type_ == DEPTH_FLOAT32)
        depth_image_ptr = cv_bridge::toCvCopy(depth_image_msg_, sensor_msgs::image_encodings::TYPE_32FC1);
      else if (depth_type_ == DEPTH_UINT16)
        depth_image_ptr = cv_bridge::toCvCopy(depth_image_msg_, sensor_msgs::image_encodings::TYPE_16UC1);
      std::stringstream depth_file_name;
      depth_file_name << save_folder_ << depth_filename_ << file_index_ss.str() << "." << image_extension_;
      saveDepth(depth_image_ptr->image, depth_file_name.str());
    }

    ROS_INFO_STREAM_THROTTLE(1, "[" << ros::this_node::getName() << "] " << file_index_ss.str() << " saved");
    file_index_++;

  }
  catch (cv_bridge::Exception & ex)
  {
    ROS_ERROR_STREAM("[" << ros::this_node::getName() << "] cv_bridge exception: " << ex.what());
  }


}

void DataCollectionNode::save(const sensor_msgs::CameraInfo::ConstPtr & camera_info,
                              const std::string & file_name)
{
  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(camera_info);

  std::ofstream file;
  file.open(file_name.c_str());

  file << "frame_id: " << camera_info_msg_->header.frame_id << std::endl;
  file << "height: " << camera_info_msg_->height << std::endl;
  file << "width: " << camera_info_msg_->width << std::endl;
  file << "distortion_model: " << camera_info_msg_->distortion_model << std::endl;
  file << "D: " << model.distortionCoeffs() << std::endl;
  file << "K: " << model.intrinsicMatrix().reshape<1, 9>() << std::endl;
  file << "R: " << model.rotationMatrix().reshape<1, 9>() << std::endl;
  file << "P: " << model.projectionMatrix().reshape<1, 12>() << std::endl;
  file << "binning_x: " << camera_info_msg_->binning_x << std::endl;
  file << "binning_y: " << camera_info_msg_->binning_y << std::endl;
  file << "roi:" << std::endl;
  file << "  x_offset: " << camera_info_msg_->roi.x_offset << std::endl;
  file << "  y_offset: " << camera_info_msg_->roi.y_offset << std::endl;
  file << "  height: " << camera_info_msg_->roi.height << std::endl;
  file << "  width: " << camera_info_msg_->roi.width << std::endl;
  file << "  do_rectify: " << (camera_info_msg_->roi.do_rectify ? "True" : "False") << std::endl;

  file.close();
}

void DataCollectionNode::checkerboardArrayCallback(const calibration_msgs::CheckerboardArray::ConstPtr & msg)
{
  checkerboard_array_msg_ = msg;
}

bool DataCollectionNode::waitForMessages()
{
  ROS_INFO_STREAM("[" << ros::this_node::getName() << "] Waiting for sensors...");
  bool ret = true;

  if (search_checkerboard_)
    ret = ret and ros::topic::waitForMessage<CheckerboardArray>("checkerboard_array", node_handle_);

  if (save_flags_ & SAVE_IMAGE)
    ret = ret and ros::topic::waitForMessage<sensor_msgs::Image>("image", node_handle_);

  if (save_flags_ & SAVE_POINT_CLOUD)
    ret = ret and ros::topic::waitForMessage<sensor_msgs::PointCloud2>("point_cloud", node_handle_);

  if (save_flags_ & SAVE_IMAGE_CAMERA_INFO)
    ret = ret and ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera_info", node_handle_);

  if (save_flags_ & SAVE_DEPTH_IMAGE)
    ret = ret and ros::topic::waitForMessage<sensor_msgs::Image>("depth_image", node_handle_);

  if (save_flags_ & SAVE_DEPTH_CAMERA_INFO)
    ret = ret and ros::topic::waitForMessage<sensor_msgs::CameraInfo>("depth_camera_info", node_handle_);

  if (ret)
    ROS_INFO_STREAM("[" << ros::this_node::getName() << "] All sensors connected");
  else
    ROS_ERROR_STREAM("[" << ros::this_node::getName() << "] Not all sensors connected!");

  return ret;
}

void DataCollectionNode::save(const pcl::PCLPointCloud2::ConstPtr & cloud,
                              const std::string & file_name)
{
  pcl::PCDWriter pcd_writer;
  pcd_writer.writeBinary(file_name, *cloud);
}

void DataCollectionNode::save(const cv::Mat & image,
                              const std::string & file_name)
{
  cv::imwrite(file_name, image);
}

void DataCollectionNode::saveDepth(const cv::Mat & depth_image,
                                   const std::string & file_name)
{
  if (depth_type_ == DEPTH_FLOAT32)
  {
    cv::Mat depth_image_16;
    depth_image.convertTo(depth_image_16, CV_16UC1, 1000);
    cv::imwrite(file_name, depth_image_16);
  }
  else if (depth_type_ == DEPTH_UINT16)
  {
    cv::imwrite(file_name, depth_image);
  }
  else
  {
    // Do nothing
  }
}

Checkerboard::Ptr DataCollectionNode::createCheckerboard(const CheckerboardMsg::ConstPtr & msg,
                                                         int id)
{
  return createCheckerboard(*msg, id);
}

Checkerboard::Ptr DataCollectionNode::createCheckerboard(const CheckerboardMsg & msg,
                                                         int id)
{
  std::stringstream ss;
  ss << "/checkerboard_" << id;
  Checkerboard::Ptr cb = boost::make_shared<Checkerboard>(msg.cols, msg.rows, msg.cell_width, msg.cell_height);
  cb->setFrameId(ss.str());
  return cb;
}

} /* namespace calibration */

using namespace calibration;

int main(int argc,
         char ** argv)
{
  ros::init(argc, argv, "rgbd_data_collector");
  ros::NodeHandle node_handle("~");

  try
  {
    DataCollectionNode collector_node(node_handle);
    if (not collector_node.initialize())
      return 0;
    ros::spin();
  }
  catch (std::runtime_error & error)
  {
    ROS_FATAL("Calibration error: %s", error.what());
    return 1;
  }

  return 0;
}
