import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    period1_ns = int(1.0/3.0*1.0e9)
    period2_ns = int(1.0/7.0*1.0e9)
    timer_period_ns = int(1.0/11.0*1.0e9)
    callback_duration_ns = int(0.01*1.0e9)


    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='e2e_demo', executable='sensor_dummy', output='screen',
            remappings=[
                ('input1', 'topic1'),
                ('input2', 'topic2')
            ],
            parameters=[
                {'period1_ns': period1_ns},
                {'period2_ns': period2_ns},
                {'callback_duration_ns': callback_duration_ns}
            ]
        ),

        launch_ros.actions.Node(
            package='e2e_demo', executable='no_dependency', output='screen',
            remappings=[
                ('input', 'topic1'),
                ('output', 'topic3'),
            ],
            parameters=[
                {'callback_duration_ns': callback_duration_ns}
            ]
        ),

        launch_ros.actions.Node(
            package='e2e_demo', executable='sub_dependency', output='screen',
            remappings=[
                ('input1', 'topic3'),
                ('input2', 'topic2'),
                ('output1', 'topic5'),
                ('output2', 'topic4'),
            ],
            parameters=[
                {'callback_duration_ns': callback_duration_ns}
            ]
        ),

        launch_ros.actions.Node(
            package='e2e_demo', executable='timer_dependency', output='screen',
            remappings=[
                ('input', 'topic4'),
                ('output', 'topic6')
            ],
            parameters=[
                {'period_ns': timer_period_ns},
                {'callback_duration_ns': callback_duration_ns}
            ]
        ),

        launch_ros.actions.Node(
            package='e2e_demo', executable='actuator_dummy', output='screen',
            remappings=[
                ('input1', 'topic5'),
                ('input2', 'topic6')
            ],
            parameters=[
                {'callback_duration_ns': callback_duration_ns}
            ]
        ),
    ])
