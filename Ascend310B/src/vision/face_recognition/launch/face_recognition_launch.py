from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    face_recognition = Node(
        package='face_recognition',
        executable='face_recognition',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'error']
    )

    socket = Node(
        package='socket',
        executable='socket2PC',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'error']
    )

    serial = Node(
        package='ros2_serial',
        executable='OrangePiSerial',
        output='screen',
        arguments=['--ros-args', '--log-level', 'error']
    )

    return LaunchDescription([
        socket,
        # TimerAction(period=2.0, actions=[yolov5s])
        face_recognition,
        # serial
    ])
