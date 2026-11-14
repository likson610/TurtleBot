"""Burger + Gazebo + SLAM z autonomiczną nawigacją (explore / goal_pose).

Cienka nakładka na start.py: cała konfiguracja (SLAM, most ROS/Gazebo, launch)
jest w jednym miejscu, żeby nie trzeba było jej zmieniać w dwóch plikach.
"""
import start

MODES = {'explore': 'explore', 'goal_pose': 'goals'}


def main(argv=None):
    args = start.parse_args(
        argv, modes=MODES, default='explore',
        description='Burger + Gazebo + SLAM. Tryby: explore i goal_pose. Domyślnie: explore.',
        mode_help='explore: automatyczna eksploracja; goal_pose: cel wskazany w RViz',
    )
    args.mode = MODES[args.mode]
    return start.run(args)


if __name__ == '__main__':
    raise SystemExit(main())
