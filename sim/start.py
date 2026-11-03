import argparse
from pathlib import Path
import shlex
import sys
import time

HERE = Path(__file__).resolve().parent
KEY_TIMEOUT = 0.6  # Terminal key-repeat delay is commonly about half a second.

# SLAM settings
SLAM = dict(
    use_sim_time=True, mode='mapping', odom_frame='odom', map_frame='map',
    base_frame='base_footprint', scan_topic='/scan', resolution=0.05,
    min_laser_range=0.12, max_laser_range=3.5, throttle_scans=1,
    transform_publish_period=0.02, map_update_interval=1.0,
    minimum_time_interval=0.1, transform_timeout=0.5, tf_buffer_duration=30.0,
    minimum_travel_distance=0.05, minimum_travel_heading=0.05,
    use_scan_matching=True, do_loop_closing=True,
)


def positive_number(value):
    number = float(value)
    if not 0 < number < float('inf'):
        raise argparse.ArgumentTypeError('Wartość musi być dodatnią, skończoną liczbą.')
    return number


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Burger + Gazebo + SLAM, bez colcon. Domyślny tryb: manual.',
    )
    parser.add_argument('mode', nargs='?', default='manual',
                        choices=['manual', 'goals', 'explore', 'keys'],
                        help='manual: jazda ręczna; goals: cel w RViz; explore: eksploracja; keys: WASD')
    parser.add_argument('--no-gui', action='store_true', help='Ukryj okno Gazebo; fizyka i sensory nadal działają.')
    parser.add_argument('--no-rviz', action='store_true', help='Nie uruchamiaj RViz.')
    parser.add_argument('--hardware-rendering', action='store_true', help='Nie wymuszaj renderowania na CPU.')
    parser.add_argument('--world', type=Path, default=HERE / 'room.sdf', help='Ścieżka świata SDF; domyślnie room.sdf.')
    parser.add_argument('--x', type=float, default=0.0, help='Początkowe X w metrach.')
    parser.add_argument('--y', type=float, default=0.0, help='Początkowe Y w metrach.')
    parser.add_argument('--speed', type=positive_number, default=0.15,
                        help='Limit jazdy w goals/explore lub prędkość WASD, m/s (maks. 0.22).')
    parser.add_argument('--turn', type=positive_number, default=0.7,
                        help='Limit skrętu w goals/explore lub skręt WASD, rad/s (maks. 2.84).')
    args = parser.parse_args(argv)
    if args.speed > 0.22 or args.turn > 2.84:
        parser.error('Dla Burgera użyj --speed <= 0.22 oraz --turn <= 2.84.')
    return args


def bridge_arguments():
    data = [
        ('clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock'),
        ('joint_states', 'sensor_msgs/msg/JointState', 'gz.msgs.Model'),
        ('odom', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry'),
        ('tf', 'tf2_msgs/msg/TFMessage', 'gz.msgs.Pose_V'),
        ('scan', 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan'),
        ('imu', 'sensor_msgs/msg/Imu', 'gz.msgs.IMU'),
    ]
    return [f'/{topic}@{ros}[{gz}' for topic, ros, gz in data] + [
        '/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist',
    ]


class WASDControl:
    """Terminal controls"""
    def __init__(self, speed, turn, timeout=KEY_TIMEOUT):
        self.speed, self.turn, self.timeout = speed, turn, timeout
        self.linear, self.angular = 0.0, 0.0
        self.last_key_time = None
        self.quit = False
        self.escape_state = ''

    def stop(self):
        self.linear, self.angular = 0.0, 0.0
        self.last_key_time = None

    def feed(self, key, now):
        if key in ('\x03', '\x04'):
            self.stop()
            self.quit = True
            return
        if self.quit:
            return
        if key == '\x1b':
            self.stop()
            self.escape_state = 'escape'
            return
        if self.escape_state == 'escape':
            self.escape_state = 'sequence' if key in ('[', 'O') else ''
            return
        if self.escape_state == 'sequence':
            if '@' <= key <= '~':
                self.escape_state = ''
            return
        key = key.lower()
        if key == 'q':
            self.stop()
            self.quit = True
            return
        movement = {'w': (self.speed, 0.0), 's': (-self.speed, 0.0),
                    'a': (0.0, self.turn), 'd': (0.0, -self.turn)}
        if key in movement:
            self.linear, self.angular = movement[key]
            self.last_key_time = now
        else:
            self.stop()  # stop movement.

    def command(self, now):
        if self.last_key_time is None or now - self.last_key_time >= self.timeout:
            self.stop()
        return self.linear, self.angular


def run_keyboard(args):
    """Publish TwistStamped"""
    if not sys.stdin.isatty():
        print('WASD wymaga interaktywnego terminala Ubuntu.', file=sys.stderr)
        return 1
    import os
    import select
    import termios
    import tty
    import rclpy
    from geometry_msgs.msg import TwistStamped

    descriptor = sys.stdin.fileno()
    old_settings = termios.tcgetattr(descriptor)
    node, publisher = None, None
    control = WASDControl(args.speed, args.turn)

    def publish(linear, angular):
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.twist.linear.x, msg.twist.angular.z = linear, angular
        publisher.publish(msg)

    try:
        rclpy.init(args=[])
        node = rclpy.create_node('wasd_teleop')
        publisher = node.create_publisher(TwistStamped, '/cmd_vel', 10)
        print(f'WASD: W przód | S tył | A/D obrót | SPACJA STOP | Q wyjście\n'
              f'Przytrzymuj klawisz. Prędkość: {args.speed:.2f} m/s, skręt: {args.turn:.2f} rad/s.\n', flush=True)
        tty.setcbreak(descriptor)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            ready, _, _ = select.select([descriptor], [], [], 0.05)
            if ready:
                data = os.read(descriptor, 64)
                if not data:
                    break
                for key in data.decode('utf-8', errors='ignore'):
                    control.feed(key, time.monotonic())
            publish(*control.command(time.monotonic()))
            if control.quit:
                break
    except KeyboardInterrupt:
        pass
    finally:
        # publish STOP before restoring the terminal
        try:
            if publisher is not None and rclpy.ok():
                for _ in range(3):
                    publish(0.0, 0.0)
                    rclpy.spin_once(node, timeout_sec=0.0)
                    time.sleep(0.02)
        finally:
            try:
                termios.tcsetattr(descriptor, termios.TCSADRAIN, old_settings)
            finally:
                if node is not None:
                    node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
    return 0


def build_launch(args):
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchDescription
    from launch.actions import (AppendEnvironmentVariable, EmitEvent, ExecuteProcess,
                                IncludeLaunchDescription, RegisterEventHandler,
                                SetEnvironmentVariable)
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown, matches_action
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch_ros.actions import LifecycleNode, Node
    from launch_ros.event_handlers import OnStateTransition
    from launch_ros.events.lifecycle import ChangeState
    from lifecycle_msgs.msg import Transition

    tb3 = Path(get_package_share_directory('turtlebot3_gazebo'))
    gz = Path(get_package_share_directory('ros_gz_sim'))
    get_package_share_directory('slam_toolbox')
    world = args.world.expanduser().resolve()
    if not world.is_file():
        raise FileNotFoundError(f'Nie ma pliku świata: {world}')
    model = tb3 / 'models/turtlebot3_burger/model.sdf'
    if 'gz-sim-diff-drive-system' not in model.read_text(encoding='utf-8'):
        raise RuntimeError('Model pochodzi z Gazebo Classic. Potrzebny jest turtlebot3_gazebo dla Jazzy/Harmonic.')

    actions = [
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(tb3 / 'models')),
    ]
    if not args.hardware_rendering:
        actions.append(SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'))

    gz_args = '-r -v 3 ' + ('-s ' if args.no_gui else '') + shlex.quote(str(world))
    actions.extend([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gz / 'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
        ),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'use_sim_time': True, 'robot_description':
                          (tb3 / 'urdf/turtlebot3_burger.urdf').read_text(encoding='utf-8')}],
             output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=bridge_arguments(), parameters=[{'use_sim_time': True}], output='screen'),
    ])
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-name', 'burger', '-file', str(model), '-z', '0.01',
                            '-x', str(args.x), '-y', str(args.y)])
    slam = LifecycleNode(package='slam_toolbox', executable='async_slam_toolbox_node',
                         name='slam_toolbox', namespace='', parameters=[SLAM], output='screen')
    # Configure and activate SLAM automatically, without a YAML or another launcher.
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_ACTIVATE))],
    ))
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    after_spawn = [activate, slam, configure]
    if not args.no_rviz:
        after_spawn.append(Node(package='rviz2', executable='rviz2',
                                arguments=['-d', str(HERE / 'view.rviz')],
                                parameters=[{'use_sim_time': True}], output='screen'))
    if args.mode in ('goals', 'explore'):
        after_spawn.append(ExecuteProcess(
            cmd=[sys.executable, str(HERE / 'navigation.py'), '--ros-args',
                 '-p', 'use_sim_time:=true', '-p', f'explore:={str(args.mode == "explore").lower()}',
                 '-p', f'max_linear_vel:={args.speed}',
                 '-p', f'max_angular_vel:={args.turn}',
                 '-p', f'min_angular_vel:={-args.turn}'], output='screen',
        ))

    def on_spawn_exit(event, _context):
        if event.returncode:
            return [EmitEvent(event=Shutdown(reason='Nie udało się wstawić Burgera do świata.'))]
        return after_spawn

    actions.append(RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=on_spawn_exit)))
    actions.append(spawn)
    return LaunchDescription(actions)


def main(argv=None):
    args = parse_args(argv)
    if sys.platform != 'linux':
        print('Uruchom ten skrypt w Ubuntu w maszynie wirtualnej.', file=sys.stderr)
        return 1
    if args.mode == 'keys':
        try:
            return run_keyboard(args)
        except ImportError as error:
            print(f'{error}\nWykonaj source /opt/ros/jazzy/setup.bash.', file=sys.stderr)
            return 1
    try:
        from launch import LaunchService
        description = build_launch(args)
    except ImportError as error:
        print(f'{error}\nWykonaj source /opt/ros/jazzy/setup.bash i sprawdź zależności z README.md.', file=sys.stderr)
        return 1
    service = LaunchService(argv=[])
    service.include_launch_description(description)
    return service.run()


if __name__ == '__main__':
    raise SystemExit(main())
