from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Number of groups (N)
    N = 2
    nodes = []

    # Loop to create N nodes
    for i in range(1, N + 1):
        nodes.append(
            Node(
                package='my_sim_ur_pkg',
                executable='sim_ur_server',
                name=f'simulated_arm_server_{i}',
                parameters=[{
                    'mqtt_client_name': f'Group{i}Client',
                    'angles_topic': f'group{i}_joint_angles',
                    'status_topic': f'group{i}_status',
                    'action_name': f'simulated_arm_server_{i}/follow_joint_trajectory'  # Unique action name per node
                }]
            )
        )

    return LaunchDescription(nodes)