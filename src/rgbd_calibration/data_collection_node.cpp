#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/String.h>

#include <calibration_common/pinhole/pinhole.h>
#include <calibration_common/objects/checkerboard.h>

#include <calibration_common/algorithms/automatic_checkerboard_finder.h>

#include <camera_info_manager/camera_info_manager.h>

#include <calibration_msgs/CheckerboardArray.h>
#include <calibration_msgs/CheckerboardMsg.h>

using namespace camera_info_manager;
using namespace calibration_msgs;

namespace calibration
{

class DataCollectionNode
{
public:

  DataCollectionNode(ros::NodeHandle & node_handle);

  virtual
  ~DataCollectionNode();

  bool
  initialize();

  void
  spin();

  static Checkerboard::Ptr
  createCheckerboard(const CheckerboardMsg::ConstPtr & msg,
                     int id);

  static Checkerboard::Ptr
  createCheckerboard(const CheckerboardMsg & msg,
                     int id);

protected:

  void
  save(const cv::Mat & image,
       const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud);

  void
  pointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & msg);

  void
  imageCallback(const sensor_msgs::Image::ConstPtr & msg);

  void
  cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & msg);

//  void
//  omnicameraInfoCallback(const OmnicameraInfo::ConstPtr & msg);

  void
  actionCallback(const std_msgs::String::ConstPtr & msg);

  void
  checkerboardArrayCallback(const calibration_msgs::CheckerboardArray::ConstPtr & msg);

  bool
  waitForMessages() const;

  /* variables */

  ros::NodeHandle node_handle_;
  image_transport::ImageTransport image_transport_;

  ros::Subscriber cloud_sub_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Subscriber omnicamera_info_sub_;
  ros::Subscriber action_sub_;

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_msg_;
  sensor_msgs::Image::ConstPtr image_msg_;
  sensor_msgs::CameraInfo::ConstPtr camera_info_msg_;
//  OmnicameraInfo::ConstPtr omnicamera_info_msg_;

// TODO find another way to get checkerboards
  ros::Subscriber checkerboards_sub_;
  calibration_msgs::CheckerboardArray::ConstPtr checkerboard_array_msg_;

  std::vector<Checkerboard::ConstPtr> cb_vec_;

  int starting_index_;
  int file_index_;

  std::string save_folder_;
  std::string image_extension_;
  std::string image_filename_;
  std::string cloud_filename_;

  bool search_checkerboard_;

};

DataCollectionNode::DataCollectionNode(ros::NodeHandle & node_handle)
  : node_handle_(node_handle),
    image_transport_(node_handle),
    search_checkerboard_(false)
{
  cloud_sub_ = node_handle.subscribe("point_cloud", 1, &DataCollectionNode::pointCloudCallback, this);
  image_sub_ = image_transport_.subscribe("image", 1, &DataCollectionNode::imageCallback, this);
  camera_info_sub_ = node_handle.subscribe("camera_info", 1, &DataCollectionNode::cameraInfoCallback, this);
//  omnicamera_info_sub_ = node_handle.subscribe("omnicamera_info", 1, &DataCollectionNode::omnicameraInfoCallback, this);
  checkerboards_sub_ = node_handle_.subscribe("checkerboard_array",
                                              1,
                                              &DataCollectionNode::checkerboardArrayCallback,
                                              this);
  action_sub_ = node_handle.subscribe("action", 1, &DataCollectionNode::actionCallback, this);

//  std::string camera_type_string;
//  node_handle_.param("camera_type", camera_type_string, std::string("pinhole"));
//
//  if (camera_type_string == "pinhole")
//    camera_type_ = ColorSensor<double>::PINHOLE;
//  else if (camera_type_string == "omnidirectional")
//    camera_type_ = ColorSensor<double>::OMNIDIRECTIONAL;
//  else
//    throw std::runtime_error("Unknown camera type. Use \"pinhole\" or \"omnidirectional\"");

//  std::string laser_type_string;
//  node_handle_.param("laser_type", laser_type_string, std::string("kinect"));
//
//  if (laser_type_string == "kinect")
//    laser_type_ = DepthSensor<double, pcl::PointXYZ>::KINECT;
//  else if (laser_type_string == "laser")
//    laser_type_ = DepthSensor<double, pcl::PointXYZ>::LASER;
//  else
//    throw std::runtime_error("Unknown laser type. Use \"kinect\" or \"laser\"");

  node_handle_.param("starting_index", starting_index_, 1);
  if (not node_handle_.getParam("save_folder", save_folder_))
    ROS_FATAL("Missing folder!!");

  if (save_folder_.at(save_folder_.size() - 1) != '/')
    save_folder_.append("/");

  file_index_ = starting_index_;

  node_handle_.param("image_extension", image_extension_, std::string("png"));
  node_handle_.param("image_filename", image_filename_, std::string("image_"));
  node_handle_.param("cloud_filename", cloud_filename_, std::string("cloud_"));

  node_handle_.param("search_checkerboard", search_checkerboard_, false);

}

DataCollectionNode::~DataCollectionNode()
{
  // Do nothing
}

bool DataCollectionNode::initialize()
{
  if (not waitForMessages())
    return false;

  for (unsigned int i = 0; i < checkerboard_array_msg_->checkerboards.size(); ++i)
    cb_vec_.push_back(createCheckerboard(checkerboard_array_msg_->checkerboards[i], i));

  return true;
}

void DataCollectionNode::pointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & msg)
{
  cloud_msg_ = pcl::PointCloud<pcl::PointXYZ>::ConstPtr(new pcl::PointCloud<pcl::PointXYZ>(*msg));
}

void DataCollectionNode::imageCallback(const sensor_msgs::Image::ConstPtr & msg)
{
  image_msg_ = msg;
}

void DataCollectionNode::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr & msg)
{
  camera_info_msg_ = msg;
}

//void DataCollectionNode::omnicameraInfoCallback(const calibration_msgs::OmnicameraInfo::ConstPtr & msg)
//{
//  omnicamera_info_msg_ = msg;
//}

void DataCollectionNode::actionCallback(const std_msgs::String::ConstPtr & msg)
{
  //if (msg->data == "stop")

}

void DataCollectionNode::checkerboardArrayCallback(const calibration_msgs::CheckerboardArray::ConstPtr & msg)
{
  checkerboard_array_msg_ = msg;
}

bool DataCollectionNode::waitForMessages() const
{
  ros::Rate rate(1.0);
  ros::spinOnce();
  while (ros::ok() and ((search_checkerboard_ and not checkerboard_array_msg_) or not image_msg_ or not cloud_msg_))
  {
    ROS_WARN("Not all messages received!");
    rate.sleep();
    ros::spinOnce();
  }
  return checkerboard_array_msg_;
}

void DataCollectionNode::spin()
{
  ros::Rate rate(2.0);
  while (ros::ok())
  {
    ros::spinOnce();

    try
    {
      cv_bridge::CvImage::Ptr image_ptr = cv_bridge::toCvCopy(image_msg_, sensor_msgs::image_encodings::BGR8);

      AutomaticCheckerboardFinder cb_finder;
      cb_finder.setImage(image_ptr->image);

      if (search_checkerboard_)
      {
        bool has_pattern = false;
        std::vector<cv::Point2f> corners;
        for (size_t i = 0; not has_pattern and i < cb_vec_.size(); ++i)
          has_pattern = cb_finder.find(*cb_vec_[i], corners);

        if (has_pattern)
          save(image_ptr->image, cloud_msg_);
      }
      else
        save(image_ptr->image, cloud_msg_);

    }
    catch (cv_bridge::Exception & ex)
    {
      ROS_ERROR("cv_bridge exception: %s", ex.what());
      return;
    }

    rate.sleep();
  }

}

void DataCollectionNode::save(const cv::Mat & image,
                              const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & cloud)
{
  pcl::PCDWriter pcd_writer;

  std::stringstream cloud_file_name;
  cloud_file_name << save_folder_ << cloud_filename_ << file_index_ << ".pcd";
  pcd_writer.writeBinary(cloud_file_name.str(), *cloud);

  std::stringstream image_file_name;
  image_file_name << save_folder_ << image_filename_ << file_index_ << "." << image_extension_;
  cv::imwrite(image_file_name.str(), image);

  file_index_++;
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
    collector_node.spin();
  }
  catch (std::runtime_error & error)
  {
    ROS_FATAL("Calibration error: %s", error.what());
    return 1;
  }

  return 0;
}
