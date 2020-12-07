import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    period_ns = 500000000
    timer_period_ns = 100000000

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='communication_aggregate', executable='aggregate', output='screen'),
        launch_ros.actions.Node(
            package='e2e_demo', executable='sensor_dummy', output='screen',
            remappings=[
                ('input1', 'topic1'),
                ('input2', 'topic2')
            ],
            parameters=[
                {'period_ns': period_ns}
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
                ('output1', 'topic4'),
                ('output2', 'topic5'),
            ]
        ),

        launch_ros.actions.Node(
            package='e2e_demo', executable='timer_dependency', output='screen',
            remappings=[
                ('input', 'topic4'),
                ('output', 'topic6')
            ],
            parameters=[
                {'period_ns': timer_period_ns}
            ]
        ),

        launch_ros.actions.Node(
            package='e2e_demo', executable='actuator_dummy', output='screen',
            remappings=[
                ('input1', 'topic5'),
                ('input2', 'topic6'),
                ('output1', 'topic5_result'),
                ('output2', 'topic6_result'),
            ],
        ),
    ])
