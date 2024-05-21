import launch
import launch_ros.actions

def generate_launch_description():
  carla_its_adapter_launch = launch.LaunchDescription([
    launch.actions.DeclareLaunchArgument(
      name='use_sim_time',
      default_value='True',
    ),
    launch.actions.DeclareLaunchArgument(
        name='center_to_baselink',
        default_value='-1.2645'
    ),
    launch.actions.DeclareLaunchArgument(
        name='fov_range',
        default_value='None'
    ),
    launch.actions.DeclareLaunchArgument(
        name='input_trajectory_topic',
        default_value='~/trajectory_topic' 
    ),
    launch_ros.actions.Node(
      package='carla_its_adapter',
      executable='carla_its_adapter_node',
      name='carla_its_adapter',
      output='screen',
      emulate_tty=True,
      parameters=[
        {
          'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
        },
        {
          'center_to_baselink': launch.substitutions.LaunchConfiguration('center_to_baselink')
        },
        {
          'fov_range': launch.substitutions.LaunchConfiguration('fov_range')
        }
      ],
      remappings=[('~/trajectory_topic', launch.substitution.LaunchConfiguration('input_trajectory_topic'))]
    )
  ])

  # Return full launch description

  return carla_its_adapter_launch
   

if __name__ == '__main__':
    generate_launch_description()