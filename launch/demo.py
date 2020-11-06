import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='e2e_demo', executable='sensor_dummy', output='screen',
            remappings=[
                ('input1', 'topic1'),
                ('input2', 'topic2')
            ]
        ),

        launch_ros.actions.Node(
            package='e2e_demo', executable='no_dependency', output='screen',
            remappings=[
                ('input', 'topic1'),
                ('output', 'topic3')
            ]
        ),

        launch_ros.actions.Node(
            package='e2e_demo', executable='sub_dependency', output='screen',
            remappings=[
                ('input1', 'topic3'),
                ('input2', 'topic2'),
                ('output', 'topic4')
            ]
        ),

        launch_ros.actions.Node(
            package='e2e_demo', executable='timer_dependency', output='screen',
            remappings=[
                ('input', 'topic4'),
                ('output', 'topic5')
            ]
        ),
    ])
