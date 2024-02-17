#include <range_image_creator_node/range_image_creator.hpp>

class PCDtoRangeImageNode : public rclcpp::Node
{
public:
  PCDtoRangeImageNode() : Node("map_to_range_image")
  {
    this->declare_parameter("input_pose", "/current_pose");
    this->declare_parameter("output_image_topic", "/range_image");
    this->declare_parameter("image_width", 240);
    this->declare_parameter("image_height", 180);
    this->declare_parameter("map_path",        "/home/apollon-22/tomoki/ev_fusion/src/ev_fusion_ros2/ev_fusion/map/map.pcd");
    this->declare_parameter("camera_mat_yaml", "/home/apollon-22/tomoki/ev_fusion/src/ev_fusion_ros2/ev_fusion/config/dv_intri_calib1023.yaml");

    input_pose_ = this->get_parameter("input_pose").get_value<std::string>();
    output_image_topic_ = this->get_parameter("output_image_topic").get_value<std::string>();
    image_width_ = this->get_parameter("image_width").get_value<int>();
    image_height_ = this->get_parameter("image_height").get_value<int>();
    map_path_ = this->get_parameter("map_path").get_value<std::string>();
    // camera_info_topic_ = this->get_parameter("camera_info_topic").get_value<std::string>();
    camera_mat_yaml_ = this->get_parameter("camera_mat_yaml").get_value<std::string>();

    //Load camera yaml
    RCLCPP_INFO(this->get_logger(), "Loading yaml file. ::%s", camera_mat_yaml_.c_str());
    cv::FileStorage fs(camera_mat_yaml_, cv::FileStorage::READ);
    if(!fs.isOpened()){
      std::cout << "Failed to open " << camera_mat_yaml_ << std::endl;
      exit(1);
    }
    //Get 4x4 extrinsic matrix and convert to Eigen::Matrix4f
    // RCLCPP_ERROR(this->get_logger(), "get extrinsic_mat_ %s ", fs["cameraextrinsicmat"]."];
    //Load camera matrix
    fx_ = fs["CameraMat"].mat().at<double>(0,0);
    RCLCPP_INFO(this->get_logger(), "get camera mat ");
    fy_ = fs["CameraMat"].mat().at<double>(1,1);

    //Get 3x3 rotation matrix.
    cv::cv2eigen(fs["CameraExtrinsicMat"].mat(), camera_extrinsic_mat_);
    camera_rotation_mat_ = camera_extrinsic_mat_.block(0, 0, 3, 3);
    camera_translation_mat_ = camera_extrinsic_mat_.block(0, 3, 3, 1);

    RCLCPP_INFO(this->get_logger(), "Loaded yaml file. Start to load PCD file ::%s", map_path_.c_str());
    //PCDファイルを読み込む
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path_, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file \n");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Loaded %d data points from PCD file\n", cloud->width * cloud->height);
    }
    RCLCPP_INFO(this->get_logger(), "PCD file LOADED \n ");

    // subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        input_pose_ , 10, std::bind(&PCDtoRangeImageNode::callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(output_image_topic_, 10);
    visualize_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/debug_range_image", 10);
  }

private:
  // void callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
  void callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    //time measurement
    // auto start = std::chrono::system_clock::now();
    // RCLCPP_INFO(this->get_logger(), "On Velodyne callback");
    pcl::PCLPointCloud2 pcl_pc2;
    //Matrix始めの3つは平行移動、s ensor_orientationは回転(quaternion)
    Eigen::Quaternionf q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
    Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(msg->pose.pose.position.x,
                                                                      msg->pose.pose.position.y,
                                                                      msg->pose.pose.position.z)) *
                                  Eigen::Affine3f(q);
    //Convert to camera frame
    Eigen::Quaternionf q2(camera_rotation_mat_);
    sensorPose = sensorPose * q2 * Eigen::Affine3f(Eigen::Translation3f(camera_translation_mat_[0],
                                                                        camera_translation_mat_[1],
                                                                        camera_translation_mat_[2]));

    //Received from PointCloud data
    // Eigen::Affine3f sensorPose_ = (Eigen::Affine3f)Eigen::Translation3f(cloud->sensor_origin_[0], cloud->sensor_origin_[1], cloud->sensor_origin_[2]); the 6DOF position of the virtual sensor as the origin with roll=pitch=yaw=0.
    //Camera Frameだと、上から見下ろした形になる。Laser Frameだと、LiDARから見た形になる。
    float minRange = 0.0f;

    pcl::RangeImagePlanar range_image;
    range_image.createFromPointCloudWithFixedSize(*cloud, image_width_,image_height_, image_height_*0.5f, image_width_*0.5f,
                                                  fx_ ,fy_ ,sensorPose, pcl::RangeImage::LASER_FRAME, 0.0, minRange);

    // std::cout << range_image << "\n";
 
    // Create an Image message
    sensor_msgs::msg::Image range_image_msg;
    range_image_msg.header    = msg->header;
    range_image_msg.height    = range_image.height;
    range_image_msg.width     = range_image.width;
    range_image_msg.encoding  = "32FC1";
    range_image_msg.step      = range_image.width * sizeof(float);
    range_image_msg.data.resize(range_image_msg.step * range_image.height);
    //Find max range
    for (size_t y = 0; y < range_image.height; ++y) {
      for (size_t x = 0; x < range_image.width; ++x) {
        pcl::PointWithRange point = range_image.getPoint(static_cast<int>(x), static_cast<int>(y));
        uint16_t range = point.range;
        memcpy(&range_image_msg.data[(y * range_image.width + x) * sizeof(float)], &range, sizeof(float));
      }
    }
    publisher_->publish(range_image_msg);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualize_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  std::string input_pose_;
  std::string output_image_topic_;
  std::string camera_mat_yaml_;
  int image_width_;
  int image_height_;
  float fx_;
  float fy_;
  Eigen::Matrix4f camera_extrinsic_mat_;
  Eigen::Matrix3f camera_rotation_mat_;
  Eigen::Vector3f camera_translation_mat_;
  std::string map_path_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCDtoRangeImageNode>());
  rclcpp::shutdown();
  return 0;
}