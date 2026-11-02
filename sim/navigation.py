import math

import numpy as np
from scipy.ndimage import binary_dilation
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from astar import a_star, reachable_cells, world_to_grid, grid_to_world, traversable_trajectory
from dwa import DWAConfig, dwa_control, braking_clear


def pose_xyyaw(transform):
    q = transform.transform.rotation
    return (transform.transform.translation.x, transform.transform.translation.y,
            math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z)))


class RobotNavigation(Node):
    """Obsługa ROS, celów i nadzorowanie autonomicznej jazdy."""
    def __init__(self):
        super().__init__('robot_navigation_node')
        self.dwa_cfg = DWAConfig()

        # auto-explore flag
        self.declare_parameter('explore', False)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('scan_timeout', 0.5)
        self.declare_parameter('tf_timeout', 1.0)
        self.declare_parameter('odom_timeout', 0.5)
        self.base_frame = self.get_parameter('base_frame').value
        self.scan_timeout = self.get_parameter('scan_timeout').value
        self.tf_timeout = self.get_parameter('tf_timeout').value
        self.odom_timeout = self.get_parameter('odom_timeout').value
        for field in ('max_linear_vel', 'min_linear_vel', 'max_angular_vel', 'min_angular_vel',
                      'max_linear_acc', 'max_angular_acc', 'robo_radius',
                      'inflation_radius', 'lidar_max_range', 'safety_margin', 'reaction_time'):
            self.declare_parameter(field, getattr(self.dwa_cfg, field))
            setattr(self.dwa_cfg, field, self.get_parameter(field).value)
        self.is_exploring = self.get_parameter('explore').value
        # store init map as unknown
        self.init_map_data = None

        # stuck detection
        self.last_position = None
        self.last_position_time = self.get_clock().now()
        self.stuck_timeout = 3.0 
        self.stuck_distance = 0.05 # maximum distance 
        self.replan_cooldown = 5.0 # replan after this time
        self.last_replan_time = None
        self.current_goal = None
        self.last_plan_time = None
        self.last_stop_reason = None

        # tf2 - localisation in a space
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)) # map
        self.scan_sub = self.create_subscription(LaserScan,     '/scan',      self.scan_callback, qos_profile_sensor_data) # lidar
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.goal_sub = self.create_subscription(PoseStamped,   '/goal_pose', self.goal_callback, 10) # pointing a goal point from rviz2

        # publishers
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel',      10) # velocities
        self.path_pub = self.create_publisher(Path,  '/planned_path', 10) # planned path

        # robot and map state
        self.grid_map = None
        self.map_resolution = 0.05 # grid size [meters]
        self.map_origin = [0.0, 0.0, 0.0] # left-bottom corner
        self.latest_scan = None
        self.last_scan_time = None
        self.last_odom_time = None
        self.measured_v, self.measured_w = 0.0, 0.0

        self.path = []
        self.target_idx = 0
        self.near_target_pos = 0.15 # avoid skipping too far ahead around corners

        self.current_v = 0.0
        self.current_w = 0.0
        self._recovery_dir = 1.0 # turn left (1.0) or right (-1.0) when stucks

        self.timer = self.create_timer(0.1, self.control_loop) # callback control loop every 0.1 s
        self.get_logger().info("Waiting for map...")

    def map_callback(self, msg):
        '''Transform SLAM into map'''
        self.map_resolution = msg.info.resolution
        q = msg.info.origin.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y, yaw]
        data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

        # -1 -> unknown, 0 -> free, >50 -> obstacle
        self.init_map_data = data 

        # binary mask (1 -> obstacle)
        raw = np.where(data > 50, 1, 0).astype(bool)
        # dilatation (bold)
        # dilation underestimates clearance in diagonal directions.
        radius = max(self.dwa_cfg.inflation_radius,
                     self.dwa_cfg.robo_radius + self.dwa_cfg.safety_margin)
        inflation_cells = max(1, math.ceil(radius/self.map_resolution + math.sqrt(2)/2))
        yy, xx = np.ogrid[-inflation_cells:inflation_cells+1, -inflation_cells:inflation_cells+1]
        inflated = binary_dilation(raw, structure=(xx*xx+yy*yy <= inflation_cells**2))
        # Only observed free cells may be used by A* and DWA.
        self.grid_map = (inflated | (data != 0)).astype(np.uint8)

        remaining = [(x, y, 0.0) for x, y in self.path[self.target_idx:]]
        if remaining and not traversable_trajectory(remaining, self.grid_map, self.map_resolution, self.map_origin):
            self.stop()
            self.path = []
            self.target_idx = 0
            self.publish_path_for_rviz()
            self.get_logger().warning('Mapa zmieniła przebieg trasy: STOP i ponowne planowanie.')

        # auto-drive mode, lack of goal and current pose is none -> explore
        if self.is_exploring and self.current_goal is None and self.get_robot_pose() is not None:
            self.trigger_exploration()

    def scan_callback(self, msg):
        '''Fcn for LiDAR subscriber'''
        self.latest_scan = msg
        self.last_scan_time = Time.from_msg(msg.header.stamp)

    def odom_callback(self, msg):
        v, w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        if math.isfinite(v) and math.isfinite(w):
            self.measured_v, self.measured_w = v, w
            self.last_odom_time = Time.from_msg(msg.header.stamp)

    def goal_callback(self, msg):
        '''Manual mode'''
        if self.grid_map is None or msg.header.frame_id not in ('', 'map'):
            self.get_logger().warning('Goal requires a map and frame_id=map.')
            return
        self.path = []
        self.stop()
        self.is_exploring = False
        self.get_logger().info("Manual goal received - auto-drive mode disable.")

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        self.current_goal = (goal_x, goal_y)

        current_pose = self.get_robot_pose()
        if current_pose is not None:
            self.run_astar(current_pose[0], current_pose[1], goal_x, goal_y)


    def trigger_exploration(self):
        """The farthest distance of robot"""
        pose = self.get_robot_pose()
        if pose is None or self.init_map_data is None:
            return

        self.stop()
        self.path = []
        self.target_idx = 0
        self.publish_path_for_rviz()
        self.current_goal = None
        self.last_plan_time = self.get_clock().now()

        x, y, _ = pose
        start_grid = self.nearest_free_cell(*self.world_to_grid(x, y))
        if start_grid is None:
            self.halt('Eksploracja: brak wolnego pola startowego obok robota.')
            return

        free_space = (self.init_map_data == 0)
        unknown_space = (self.init_map_data == -1)
        unknown_dilated = binary_dilation(unknown_space, iterations=1)
        frontiers = free_space & unknown_dilated

        reachable = reachable_cells(self.grid_map, start_grid)
        frontier_coords = np.argwhere(frontiers & reachable)
        if len(frontier_coords) == 0:
            self.halt(
                f'Eksploracja: brak osiągalnej granicy mapy '
                f'(pola dostępne: {np.count_nonzero(reachable)}, '
                f'pola granicy: {np.count_nonzero(frontiers)}). Czekam na nową mapę.')
            return

        dists = np.linalg.norm(frontier_coords - np.array(start_grid), axis=1)
        target_grid = tuple(frontier_coords[np.argmax(dists)])
        path = a_star(self.grid_map, start_grid, target_grid)
        if not path:
            self.halt('Eksploracja: nie udało się wyznaczyć trasy do wybranej granicy.')
            return

        goal_x, goal_y = self.grid_to_world(*target_grid)
        self.current_goal = (goal_x, goal_y)
        self.path = [self.grid_to_world(p[0], p[1]) for p in path]
        self.target_idx = 0
        self.last_stop_reason = None
        self.publish_path_for_rviz()
        self.get_logger().info(f'Auto-Exploring to reachable frontier at ({goal_x:.2f}, {goal_y:.2f})')


    def run_astar(self, start_x, start_y, goal_x, goal_y):
        '''Operate A* algorithm'''
        # stop before planning a long calculation must not retain a drive command.
        self.stop()
        self.path = []
        self.target_idx = 0
        self.last_plan_time = self.get_clock().now()
        # convert meteres to matrix indicies
        raw_start = self.world_to_grid(start_x, start_y)
        raw_goal  = self.world_to_grid(goal_x,  goal_y)

        # if too close to obstacles, find the nearest free point
        start_grid = self.nearest_free_cell(*raw_start)
        goal_grid  = self.nearest_free_cell(*raw_goal)

        if start_grid is None or goal_grid is None:
            return False
        grid_path = a_star(self.grid_map, start_grid, goal_grid)

        if not grid_path:
            self.get_logger().error("A* found no path.")
            return False

         # convert matrix indicies to meters
        self.path       = [self.grid_to_world(p[0], p[1]) for p in grid_path]
        self.target_idx = 0
        self.publish_path_for_rviz()
        return True

    def nearest_free_cell(self, grid_row, grid_col, search_radius=8):
        '''Find nearest free cell'''
        rows, cols = self.grid_map.shape
        # if git return the current index
        if (0 <= grid_row < rows and 0 <= grid_col < cols and self.grid_map[grid_row, grid_col] == 0):
            return (grid_row, grid_col)

        # check neighborhood to find free cell
        for r in range(1, search_radius + 1):
            for dr in range(-r, r + 1):
                for dc in range(-r, r + 1):
                    if abs(dr) != r and abs(dc) != r: continue
                    nr, nc = grid_row + dr, grid_col + dc
                    if 0 <= nr < rows and 0 <= nc < cols:
                        if self.grid_map[nr, nc] == 0:
                            return (nr, nc)
        return None




    def control_loop(self):
        '''Main control function'''
        if (self.latest_scan is None or self.last_scan_time is None
                or not 0 <= (self.get_clock().now()-self.last_scan_time).nanoseconds/1e9 <= self.scan_timeout):
            self.stop()
            return
        now = self.get_clock().now()
        scan_age = (now-self.last_scan_time).nanoseconds/1e9
        if (self.last_odom_time is None
                or not 0 <= (now-self.last_odom_time).nanoseconds/1e9 <= self.odom_timeout
                or not self.latest_scan.ranges
                or not any((math.isfinite(r) and r >= self.latest_scan.range_min)
                           or r == math.inf for r in self.latest_scan.ranges)):
            self.stop()
            return
        try:
            nearby = self.scan_in_base(self.latest_scan)
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.stop()
            return
        # independent of the SLAM map: a bad map must not override a nearby wall.
        if not braking_clear(self.measured_v, self.measured_w, nearby, self.dwa_cfg, scan_age):
            self.halt('Przeszkoda na drodze hamowania: STOP na podstawie LIDAR-u.')
            return
        if not self.path:
            self.stop()
            if (self.grid_map is not None and (self.last_plan_time is None
                    or (now-self.last_plan_time).nanoseconds/1e9 >= 1.0)):
                if self.is_exploring:
                    self.trigger_exploration()
                elif self.current_goal is not None:
                    pose = self.get_robot_pose()
                    if pose is not None:
                        self.run_astar(pose[0], pose[1], *self.current_goal)
            return

        current_pose = self.get_robot_pose()
        if current_pose is None:
            self.stop()
            return
        x, y, theta = current_pose



        target_x, target_y = self.path[self.target_idx]
        # goal reached
        if math.hypot(target_x - x, target_y - y) < self.near_target_pos:
            self.target_idx += 1 # next point    

            if self.target_idx >= len(self.path):
                self.path = []
                self.current_goal = None
                self.publish_velocity(Twist())
                self.current_v, self.current_w = 0.0, 0.0

                # explore mode -> find new goal point
                if self.is_exploring:
                    self.trigger_exploration()
                else:
                    self.get_logger().info("Manual goal reached.")
                return

            target_x, target_y = self.path[self.target_idx]

        # avoid obstacles
        try:
            obstacles = self.scan_to_coords(x, y, theta, self.latest_scan)
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.stop()
            return
        best_v, best_w = dwa_control(
            current_pose, (self.measured_v, self.measured_w),
            (target_x, target_y), obstacles,
            self.grid_map, self.map_resolution, self.map_origin, self.dwa_cfg)
        if not braking_clear(best_v, best_w, nearby, self.dwa_cfg, scan_age):
            self.halt('Planowany ruch nie zapewnia miejsca na hamowanie: STOP.')
            return

        msg = Twist()
        msg.linear.x, msg.angular.z = best_v, best_w
        self.publish_velocity(msg)
        self.last_stop_reason = None

        self.current_v, self.current_w = best_v, best_w


    def scan_to_coords(self, rx, ry, th, scan):
        # lidar offset must come from TF
        t = self.tf_buffer.lookup_transform('map', scan.header.frame_id, Time.from_msg(scan.header.stamp))
        q = t.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        sx, sy = t.transform.translation.x, t.transform.translation.y
        return [(sx+r*math.cos(yaw+scan.angle_min+i*scan.angle_increment),
                 sy+r*math.sin(yaw+scan.angle_min+i*scan.angle_increment))
                for i, r in enumerate(scan.ranges)
                if math.isfinite(r) and scan.range_min <= r < min(scan.range_max, self.dwa_cfg.lidar_max_range)]

    def scan_in_base(self, scan):
        # compensate movement since the scan using odom, without map -> odom.
        laser = self.tf_buffer.lookup_transform('odom', scan.header.frame_id, Time.from_msg(scan.header.stamp))
        base = self.tf_buffer.lookup_transform('odom', self.base_frame, Time())
        lx, ly, la = pose_xyyaw(laser)
        bx, by, ba = pose_xyyaw(base)
        dx, dy = lx-bx, ly-by
        sx, sy = math.cos(ba)*dx + math.sin(ba)*dy, -math.sin(ba)*dx + math.cos(ba)*dy
        return [(sx+r*math.cos(la-ba+scan.angle_min+i*scan.angle_increment),
                 sy+r*math.sin(la-ba+scan.angle_min+i*scan.angle_increment))
                for i, r in enumerate(scan.ranges)
                if math.isfinite(r) and scan.range_min <= r < min(scan.range_max, self.dwa_cfg.lidar_max_range)]

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', self.base_frame, Time())
            age = (self.get_clock().now()-Time.from_msg(t.header.stamp)).nanoseconds/1e9
            if not 0 <= age <= self.tf_timeout:
                return None
            q = t.transform.rotation
            return (t.transform.translation.x, t.transform.translation.y,
                    math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def world_to_grid(self, x, y):
        return world_to_grid(x, y, self.map_resolution, self.map_origin)

    def grid_to_world(self, r, c):
        return grid_to_world(r, c, self.map_resolution, self.map_origin)

    # public robot path

    def publish_path_for_rviz(self):
        '''Visualize planned path'''
        msg = Path(); msg.header.frame_id = 'map'; msg.header.stamp = self.get_clock().now().to_msg()
        for p in self.path:
            pose = PoseStamped(); pose.header = msg.header; pose.pose.orientation.w = 1.0; pose.pose.position.x = p[0]; pose.pose.position.y = p[1]; msg.poses.append(pose)
        self.path_pub.publish(msg)

    def publish_velocity(self, velocity):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist = velocity
        self.cmd_vel_pub.publish(msg)

    def stop(self):
        self.publish_velocity(Twist())
        self.current_v, self.current_w = 0.0, 0.0

    def halt(self, reason):
        if reason != self.last_stop_reason:
            self.get_logger().warning(reason)
            self.last_stop_reason = reason
        self.stop()


def main(args=None):
    rclpy.init(args=args)
    node = RobotNavigation()
    try: rclpy.spin(node)
    except KeyboardInterrupt: node.get_logger().info("Stopping...")
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
