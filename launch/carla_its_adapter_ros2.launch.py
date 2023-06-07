import launch
import launch_ros.actions

#TODO center_to_baselink_
#TODO optional fov_range_ (ansonsten alles publishen)

def generate_launch_description():
  carla_its_adapter_launch = launch.LaunchDescription([
    launch.actions.DeclareLaunchArgument(
        name='center_to_baselink',
        default_value='1.2645'
    ),
    launch.actions.DeclareLaunchArgument(
        name='fov_range',
        default_value='None'
    ),
    launch_ros.actions.Node(
      package='carla_its_adapter',
      executable='carla_its_adapter_node',
      name='ItsAdapterNode',
      output='screen',
      emulate_tty=True,
      parameters=[
        {
          'center_to_baselink': launch.substitutions.LaunchConfiguration('center_to_baselink')
        },
        {
          'fov_range': launch.substitutions.LaunchConfiguration('fov_range')
        }
      ]
    )
  ])

  # Return full launch description

  return carla_its_adapter_launch
   

if __name__ == '__main__':
    generate_launch_description()