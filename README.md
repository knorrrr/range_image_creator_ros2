# Range Image Creator for ROS 2 humble
This nodes subscribe ego's current pose(TF or Pose) and PointCloud, and publish depth image(msg::Image, 32FC1)

## Getting Started
### TF-based 
```bash
mkdir -p ros2_ws/src && cd ros2_ws/src
git clone git@github.com:knorrrr/range_image_creator_ros2.git
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Usage 
```bash
ros2 launch range_image_creator tf_base_creator_launch.py
```
### Pose-based 
```bash
ros2 launch range_image_creator pose_base_creator_launch.py
```

## IO

Convert PointCloud2 to rangeImage(msg::Image).
| Name                     | Type      | Description                            |Default                |
| ---------------          | --------- | ---------------------------------------|-----------------------|
| `output_image_topic`     | `String`  | output topic                           |/range_image           |
| `map_path`               | `String`  | input pointcloud(.pcd) file            |<ws>/map/pcd_0122_outward_tf.pcd|
| `camera_mat_path`         | `String`  | input camera matrix(distortion, camera matrix) made by OpenCV |<ws>/params/0126_davis_manual_no_rotation.yaml|
| `image_width`            | `int`   | rangeImage's Width(pixel)                |240                 |
| `maxAngleHeight`         | `int`   | rangeImage's maxAngleHeight(degree)      |180                |
