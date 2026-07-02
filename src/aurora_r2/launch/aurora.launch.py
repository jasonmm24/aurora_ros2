from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. El driver principal de SLAMTEC Aurora
        Node(
            package='aurora_r2',
            executable='aurora_bridge_node',
            name='aurora_bridge_node',
            output='screen'
        ),

        # 2. TF: Centro del Robot -> LiDAR (Re-habilitado según tu XML)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['--x', '0', '--y', '0', '--z', '0.0315', 
                       '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', 
                       '--frame-id', 'aurora_base_link', '--child-frame-id', 'aurora_lidar_frame']
        ),

        # 3. TF: Centro del Robot -> Cámara Izquierda
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_leftcam',
            arguments=['--x', '0.0418', '--y', '0.03', '--z', '0', 
                       '--qx', '-0.5', '--qy', '0.5', '--qz', '-0.5', '--qw', '0.5', 
                       '--frame-id', 'aurora_base_link', '--child-frame-id', 'aurora_camera_left_frame']
        ),

        # 4. TF: Cámara Izquierda -> Cámara Derecha
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='leftcam_to_rightcam',
            arguments=['--x', '0.06', '--y', '0', '--z', '0', 
                       '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', 
                       '--frame-id', 'aurora_camera_left_frame', '--child-frame-id', 'aurora_camera_right_frame']
        ),

        # 5. TF: Cámara Izquierda -> IMU
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='leftcam_to_imu',
            arguments=['--x', '0.03', '--y', '0', '--z', '0', 
                       '--qx', '0', '--qy', '0', '--qz', '-0.7071068', '--qw', '0.7071068', 
                       '--frame-id', 'aurora_camera_left_frame', '--child-frame-id', 'aurora_imu_frame']
        ),
        
        # 6. TF: Cámara Izquierda -> Cámara de Profundidad (Mismo origen)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='leftcam_to_depth',
            arguments=['--x', '0', '--y', '0', '--z', '0', 
                       '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1', 
                       '--frame-id', 'aurora_camera_left_frame', '--child-frame-id', 'aurora_depth_frame']
        ),

        # 7. Lanzar RViz2 automáticamente
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])

