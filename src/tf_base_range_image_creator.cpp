#include <range_image_creator_node/range_image_creator.hpp>
class PCDtoRangeImageNode : public rclcpp::Node
{
public:
  PCDtoRangeImageNode() : Node("range_image_creator_node"), tf_buffer_(this->get_clock()),tf_listener_(tf_buffer_)
  {
    this->declare_parameter("input_pose", "/current_pose");
    this->declare_parameter("output_image_topic", "/range_image");
    this->declare_parameter("image_width", 240);
    this->declare_parameter("image_height", 180);
    this->declare_parameter("map_path",        "/home/hermes-22/event_camera/ev_fusion_ros2/src/ev_fusion/map/pcd_0122_outward_tf.pcd");
    this->declare_parameter("camera_mat_yaml", "/home/hermes-22/event_camera/ev_fusion_ros2/src/ev_fusion/config/0126_davis_manual.yaml");

    input_pose_         = this->get_parameter("input_pose").get_value<std::string>();
    output_image_topic_ = this->get_parameter("output_image_topic").get_value<std::string>();
    image_width_        = this->get_parameter("image_width").get_value<int>();
    image_height_       = this->get_parameter("image_height").get_value<int>();
    map_path_           = this->get_parameter("map_path").get_value<std::string>();
    camera_mat_yaml_    = this->get_parameter("camera_mat_yaml").get_value<std::string>();

    //Load camera yaml
    cv::FileStorage fs(camera_mat_yaml_, cv::FileStorage::READ);
    if(!fs.isOpened()){
      std::cout << "Failed to open " << camera_mat_yaml_ << std::endl;
      exit(1);
    }
    RCLCPP_INFO(this->get_logger(), "Loaded yaml file. ::%s", camera_mat_yaml_.c_str());
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

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(output_image_topic_, 10);
    visualize_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/debug_range_image", 10);

    while(rclcpp::ok()){
      // auto start = std::chrono::system_clock::now();
      // rclcpp::spin_some(this->get_node_base_interface());
      callback();
      // auto end = std::chrono::system_clock::now();
      // auto dur = end - start;
      // auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
      // RCLCPP_INFO(this->get_logger(), "range image created in %lu msec", msec);
    }

      
  }

private:
  void callback()
  {

    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_.lookupTransform(
              "map", "vlp32c", rclcpp::Time(), rclcpp::Duration::from_seconds(0.1));
      RCLCPP_INFO(this->get_logger(), "Transform found");

    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
      return;
    }
    //Matrix始めの3つは平行移動、s ensor_orientationは回転(quaternion)
    //Convert to camera pose 
    Eigen::Quaternionf q(transform_stamped.transform.rotation.w,
                         transform_stamped.transform.rotation.x,
                         transform_stamped.transform.rotation.y,
                         transform_stamped.transform.rotation.z);
    Eigen::Affine3f cameraPose = Eigen::Affine3f(Eigen::Translation3f(transform_stamped.transform.translation.x,
                                                                      transform_stamped.transform.translation.y,
                                                                      transform_stamped.transform.translation.z)) *
                                  Eigen::Affine3f(q);
    //Convert to image pose 
    Eigen::Affine3f imagePose;
    Eigen::Quaternionf cam_rot(camera_rotation_mat_);

    imagePose = cameraPose * cam_rot*  Eigen::Affine3f(Eigen::Translation3f(camera_translation_mat_[0],
                                                                        camera_translation_mat_[1],
                                                                        camera_translation_mat_[2]));
    

    //Received from PointCloud data
    //Camera Frameだと、上から見下ろした形になる。Laser Frameだと、LiDARから見た形になる。

    pcl::RangeImagePlanar range_image;

    // auto start = std::chrono::system_clock::now();

    // TO-DO  Deal with Heavy CPU process
    range_image.createFromPointCloudWithFixedSize(*cloud, image_width_,image_height_, image_height_*0.5f, image_width_*0.5f,
                                                  fx_ ,fy_ , imagePose, pcl::RangeImage::LASER_FRAME, 0.0, minRange_);

    // auto end = std::chrono::system_clock::now();
    // auto dur = end - start;
    // auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
    // RCLCPP_INFO(this->get_logger(), "pub image created in %lu msec", msec);
 
    // Create an Image message
    sensor_msgs::msg::Image range_image_msg;
    range_image_msg.header    = transform_stamped.header;
    range_image_msg.height    = range_image.height;
    range_image_msg.width     = range_image.width;
    range_image_msg.encoding  = "32FC1";
    range_image_msg.step      = range_image.width * sizeof(float);
    range_image_msg.data.resize(range_image_msg.step * range_image.height); 
    for (size_t y = 0; y < range_image.height; ++y) {
      for (size_t x = 0; x < range_image.width; ++x) {
        pcl::PointWithRange point = range_image.getPoint(static_cast<int>(x), static_cast<int>(y));
        //for publish
        memcpy(&range_image_msg.data[(y * range_image.width + x) * sizeof(float)], &point.range, sizeof(float));
      }
    }

    //for debug use. make normalized image
    sensor_msgs::msg::Image debug_range_msg;
    debug_range_msg.header   = transform_stamped.header;
    debug_range_msg.height   = range_image.height;
    debug_range_msg.width    = range_image.width;
    debug_range_msg.encoding = "mono16";
    debug_range_msg.step     = range_image.width * sizeof(u_int16_t);
    debug_range_msg.data.resize(debug_range_msg.step * range_image.height);
    std::vector<uint16_t> debug_range_data(range_image.width * range_image.height);
    for (size_t y = 0; y < range_image.height; ++y) {
      for (size_t x = 0; x < range_image.width; ++x) {
        pcl::PointWithRange point_d = range_image.getPoint(static_cast<int>(x), static_cast<int>(y));
        uint16_t range16 = point_d.range;
        debug_range_data[y * range_image.width + x] = range16;
      }
    }
    memccpy(&debug_range_msg.data[0], &debug_range_data[0], sizeof(uint16_t), debug_range_data.size() * sizeof(uint16_t));
    

    publisher_->publish(range_image_msg);
    visualize_publisher_->publish(debug_range_msg);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualize_publisher_;
  std::string input_pose_;
  std::string output_image_topic_;
  std::string camera_mat_yaml_;
  int image_width_;
  int image_height_;
  float fx_;
  float fy_;
  float minRange_ = 0.0f;
  Eigen::Matrix4f camera_extrinsic_mat_;
  Eigen::Matrix3f camera_rotation_mat_;
  Eigen::Vector3f camera_translation_mat_;
  std::string map_path_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCDtoRangeImageNode>());
  rclcpp::shutdown();
  return 0;
}