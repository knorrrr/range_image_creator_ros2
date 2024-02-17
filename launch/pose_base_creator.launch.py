from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    map_path = os.path.join(get_package_share_directory('range_image_creator_node'),'map/pcd_0122_outward_tf.pcd')
    camera_mat_yaml = os.path.join(get_package_share_directory('range_image_creator_node'),'params/0126_davis_manual_no_rotation.yaml')

    pose_range_image_node = Node(
            package='range_image_creator_node', executable = 'pose_base_range_image_creator', name = 'range_image_node',
            output = 'screen',
            parameters = [
                          {'input_pose': "/current_pose"},
                          {'output_image_topic': "/range_image"},
                          {'map_path' : map_path},
                          {'camera_mat_yaml': camera_mat_yaml},
                          {'image_width': 240},
                          {'image_height': 180}
                         ]
    )
    return LaunchDescription([
        pose_range_image_node
    ])