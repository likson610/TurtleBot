import argparse, os, select, subprocess, sys, termios, time, tty
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped

HERE = Path(__file__).resolve().parent
KEYS = {'w': (1, 0), 's': (-1, 0), 'a': (0, 1), 'd': (0, -1)}  # (liniowa, kątowa)
TIMEOUT = 0.6  # po tylu sekundach bez klawisza robot staje

p = argparse.ArgumentParser()
p.add_argument('--world', type=Path, default=HERE / 'room.sdf')
p.add_argument('--no-gui', action='store_true')
p.add_argument('--speed', type=float, default=0.15)  # m/s, maks. 0.22
p.add_argument('--turn', type=float, default=0.7)    # rad/s, maks. 2.84
args = p.parse_args()
if not (0 < args.speed <= 0.22 and 0 < args.turn <= 2.84):
    sys.exit('Dla Burgera: --speed <= 0.22 i --turn <= 2.84.')

tb3 = Path(get_package_share_directory('turtlebot3_gazebo'))
env = dict(os.environ, TURTLEBOT3_MODEL='burger', LIBGL_ALWAYS_SOFTWARE='1',
           GZ_SIM_RESOURCE_PATH=f"{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}:{tb3 / 'models'}")

procs = [
    subprocess.Popen(['gz', 'sim', '-r', '-v', '3', *(['-s'] if args.no_gui else []),
                      str(args.world.resolve())], env=env),
    subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                      '/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist'], env=env),
]

fd = sys.stdin.fileno()
old = termios.tcgetattr(fd)
node = None
try:
    for _ in range(30):
        time.sleep(1)
        if subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-name', 'burger', '-z', '0.01',
                           '-file', str(tb3 / 'models/turtlebot3_burger/model.sdf')],
                          env=env).returncode == 0:
            break
    else:
        sys.exit('Nie udało się wstawić robota.')

    rclpy.init()
    node = rclpy.create_node('wasd_teleop')
    pub = node.create_publisher(TwistStamped, '/cmd_vel', 10)

    def send(lin, ang):
        m = TwistStamped()
        m.header.stamp = node.get_clock().now().to_msg()
        m.twist.linear.x, m.twist.angular.z = lin * args.speed, ang * args.turn
        pub.publish(m)

    print('W przód | S tył | A/D obrót | inny klawisz STOP | Q wyjście (przytrzymuj klawisz)')
    tty.setcbreak(fd)
    cmd, last = (0, 0), 0.0
    while True:
        if select.select([fd], [], [], 0.05)[0]:
            data = os.read(fd, 64)
            key = data.decode(errors='ignore')[:1].lower()
            if key == 'q' or data == b'\x03':
                break
            cmd, last = (KEYS.get(key, (0, 0)) if not data.startswith(b'\x1b') else (0, 0)), time.monotonic()
        send(*(cmd if time.monotonic() - last < TIMEOUT else (0, 0)))
finally:
    if node:
        for _ in range(3):
            send(0, 0)
        node.destroy_node()
        rclpy.shutdown()
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    for pr in procs:
        pr.terminate()