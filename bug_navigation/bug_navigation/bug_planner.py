#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion, PoseStamped 
from sensor_msgs.msg import LaserScan
import math
import time
from enum import Enum
import tf2_ros # Added TF2 imports
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException, TransformException
from tf2_geometry_msgs import do_transform_pose_stamped # Function to transform PoseStamped
from nav_msgs.msg import Odometry, Path 


# Helper function to convert quaternion to Euler angles (yaw)
def euler_from_quaternion(quaternion):
    """
    Convert a eometry_msgs/Quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

# Define states for the Bug algorithm
class BugState(Enum):
    GO_TO_GOAL = 1    # Moving directly towards the goal.
    WALL_FOLLOW = 2   # Following an obstacle boundary.
    GOAL_REACHED = 3  # Goal has been successfully reached.

class BugPlannerNode(Node):
    """
    Implements the Tangent Bug navigation logic as a ROS 2 Node.
    Handles state transitions, obstacle detection, wall following,
    and goal management.
    """
    def __init__(self):
        super().__init__('bug_planner_node')

        # Declare node parameters with default values. These can be overridden at launch.
        self.declare_parameter('wall_follow_dist', 0.4)  # Desired distance to keep from the wall during WALL_FOLLOW.
        self.declare_parameter('max_scan_range', 10.0)   # Maximum laser scan range to consider valid. Readings above this are ignored or capped.
        self.declare_parameter('obstacle_threshold', 0.3) # Distance threshold: ranges below this trigger obstacle avoidance/wall following.
        self.declare_parameter('goal_tolerance', 0.2)    # Radius around the goal within which the goal is considered reached.
        self.declare_parameter('linear_speed', 0.2)      # Maximum forward speed (m/s).
        self.declare_parameter('angular_speed', 0.5)     # Maximum turning speed (rad/s).
        self.declare_parameter('control_loop_freq', 10.0) # Frequency (Hz) at which the main control logic runs.
        self.declare_parameter('target_frame', 'odom')   # The TF frame into which goal poses will be transformed for internal use. Usually the odometry frame.

        # --- Goal Handling Variables ---
        self.goal_pose_stamped = None # Stores the latest goal received (PoseStamped includes header/frame).
        self.goal_internal = None     # Stores the goal position (Point) after transformation into the target_frame.

        self.get_logger().info("Waiting for goal via /goal_pose topic...")

        # --- Robot State Variables ---
        self.current_pose = None      # Stores the robot's current position (Point) from odometry.
        self.current_yaw = 0.0        # Stores the robot's current orientation (yaw angle in radians) from odometry.
        self.odom_frame = None        # Stores the frame_id from incoming odometry messages (e.g., 'odom').
        self.scan_data = None         # Stores the latest received LaserScan message.

        # --- Bug Algorithm State Variables ---
        self.obstacle_hit_point = None # Stores the robot's position (Point) where an obstacle was first encountered, initiating WALL_FOLLOW.
        self.initial_goal_dist = None  # Stores the distance to the goal at the moment the obstacle was hit. Used for the Tangent Bug leave condition.
        self.state = BugState.GO_TO_GOAL # Initialize the state machine.

        # --- Path Visualization ---
        self.path_msg = Path()        # Stores the path message for visualization.

        # --- TF2 Setup ---
        self.tf_buffer = tf2_ros.Buffer() # Buffer to store TF transforms.
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self) # Listens for TF transforms.

        # --- ROS Publishers ---
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10) # Publishes velocity commands.
        self.path_publisher = self.create_publisher(Path, 'robot_path', 10) # Publishes the robot's trajectory.

        # --- ROS Subscribers ---
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            '/goal_pose',        # Standard topic for receiving goals from RViz.
            self.goal_callback,  # Function to handle incoming goals.
            10)
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',              # Topic for robot odometry.
            self.odom_callback,  # Function to process odometry data.
            10)
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',              # Topic for laser scan data.
            self.scan_callback,  # Function to store scan data.
            rclpy.qos.qos_profile_sensor_data) # Use QoS profile suitable for sensor data.

        # --- Control Loop Timer ---
        # Creates a timer that calls the control_loop method periodically.
        self.timer = self.create_timer(
            1.0 / self.get_parameter('control_loop_freq').get_parameter_value().double_value,
            self.control_loop)

        self.get_logger().info("Bug Planner Node Initialized. Ready for goal on /goal_pose.")

    def goal_callback(self, msg):
        """Callback when a new goal is received."""
        self.get_logger().info(f"New goal received in frame '{msg.header.frame_id}'")
        self.goal_pose_stamped = msg # Store the new goal
        self.goal_internal = None      # Reset transformed goal, will recalculate in loop
        self.state = BugState.GO_TO_GOAL # Reset state to go towards new goal
        self.obstacle_hit_point = None   # Clear previous obstacle encounter data
        self.initial_goal_dist = None    # Clear previous obstacle encounter data
        self.get_logger().info("State reset to GO_TO_GOAL for new target.")

    def odom_callback(self, msg):
        """
        Processes incoming odometry data. Updates the robot's current pose and yaw.
        Also publishes the robot's path for visualization.
        """
        self.current_pose = msg.pose.pose.position
        self.odom_frame = msg.header.frame_id # Store the frame ID ('odom')
        _, _, self.current_yaw = euler_from_quaternion(msg.pose.pose.orientation)
        
        # --- Path Visualization ---
        # Append current pose to the path message for visualization in RViz.
        if self.current_pose and self.odom_frame:
            current_pose_stamped = PoseStamped()
            current_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            current_pose_stamped.header.frame_id = self.odom_frame # Use the odom frame
            current_pose_stamped.pose = msg.pose.pose # Copy the pose
            
            # Append to path and limit length (optional)
            self.path_msg.poses.append(current_pose_stamped)
            # Limit path length to avoid RViz lag (e.g., keep last 500 poses)
            max_path_poses = 500
            if len(self.path_msg.poses) > max_path_poses:
                self.path_msg.poses = self.path_msg.poses[-max_path_poses:]
            
            self.path_msg.header = current_pose_stamped.header # Update path header
            self.path_publisher.publish(self.path_msg)

    def scan_callback(self, msg):
        """Stores the latest laser scan data."""
        self.scan_data = msg

    # --- Helper Functions ---
    def calculate_distance(self, p1, p2):
        """Calculates the Euclidean distance between two geometry_msgs/Point."""
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def calculate_angle_to_goal(self):
        angle = math.atan2(self.goal.y - self.current_pose.y,
                            self.goal.x - self.current_pose.x)
        return angle

    def normalize_angle(self, angle):
        """Normalizes an angle to the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def can_leave_wall(self):
        """
        Checks if the Tangent Bug conditions for leaving the wall are met.
        1. Direct path to goal must be clear.
        2. Must be closer to the goal than when the wall was initially hit.
        3. Must be approximately on the 'M-line' (line connecting hit point and goal).
        """
        # Ensure we have all necessary information.
        if self.goal_internal is None or self.obstacle_hit_point is None or self.initial_goal_dist is None:
            return False # Not enough info

        # --- Condition 1: Is the direct path to goal clear? ---
        angle_to_goal = math.atan2(self.goal_internal.y - self.current_pose.y,
                                    self.goal_internal.x - self.current_pose.x)
        angle_diff = self.normalize_angle(angle_to_goal - self.current_yaw)

        # Check ranges in the direction of the goal
        goal_clear = True
        obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        max_scan_range = self.get_parameter('max_scan_range').get_parameter_value().double_value
        num_ranges = len(self.scan_data.ranges)
        goal_index = int(((angle_diff + math.pi) / (2 * math.pi)) * num_ranges) % num_ranges # Index towards goal angle difference

        # Check a small arc around the goal index
        check_angle_range = math.radians(5.0) # Small check range
        num_steps_check = int(check_angle_range / self.scan_data.angle_increment)

        for i in range(num_steps_check + 1):
            idx_right = (goal_index + i) % num_ranges
            idx_left = (goal_index - i + num_ranges) % num_ranges # Ensure positive index
            range_right = self.scan_data.ranges[idx_right] if idx_right < len(self.scan_data.ranges) else max_scan_range
            range_left = self.scan_data.ranges[idx_left] if idx_left < len(self.scan_data.ranges) else max_scan_range

            # Use max_scan_range for comparison if reading is invalid/inf
            if range_right < self.scan_data.range_min or range_right > max_scan_range: range_right = max_scan_range
            if range_left < self.scan_data.range_min or range_left > max_scan_range: range_left = max_scan_range

            # If any point in the direction of the goal is blocked
            if range_right < obstacle_threshold or range_left < obstacle_threshold:
                goal_clear = False
                break

        if not goal_clear:
            self.get_logger().debug("Leave Wall Check: Goal direction is blocked.", throttle_duration_sec=1)
            return False

        # --- Condition 2: Is the robot closer to the goal than when it hit the wall? ---
        current_dist_to_goal = self.calculate_distance(self.current_pose, self.goal_internal)
        # Add a small epsilon to prevent leaving due to floating point inaccuracies when distance hasn't significantly decreased.
        distance_epsilon = 0.01 # Require being at least 1cm closer
        if current_dist_to_goal >= (self.initial_goal_dist - distance_epsilon): # ADDED Epsilon
            self.get_logger().debug(f"Leave Wall Check: Not closer enough to goal ({current_dist_to_goal:.2f} >= {self.initial_goal_dist:.2f} - {distance_epsilon})", throttle_duration_sec=1)
            return False
        # --- Condition 3: Is the robot approximately on the M-line? ---
        # M-line is the line segment connecting the obstacle_hit_point and the goal_internal.
        angle_hit_to_current = math.atan2(self.current_pose.y - self.obstacle_hit_point.y,
                                            self.current_pose.x - self.obstacle_hit_point.x)
        angle_hit_to_goal = math.atan2(self.goal_internal.y - self.obstacle_hit_point.y,
                                        self.goal_internal.x - self.obstacle_hit_point.x)
        m_line_angle_diff = self.normalize_angle(angle_hit_to_current - angle_hit_to_goal)

        # Allow some tolerance for being "on" the M-line
        m_line_tolerance = math.radians(5.0) # Changed to 5 degrees
        if abs(m_line_angle_diff) > m_line_tolerance:
            self.get_logger().debug(f"Leave Wall Check: Not on M-line (diff: {math.degrees(m_line_angle_diff):.1f} > {math.degrees(m_line_tolerance):.1f})", throttle_duration_sec=1)
            return False

        # All conditions met! the robot can leave the wall.
        self.get_logger().info(f"Leave Wall Conditions Met! Dist: {current_dist_to_goal:.2f} < {self.initial_goal_dist:.2f}, M-Line diff: {math.degrees(m_line_angle_diff):.1f} deg")
        return True
       
       
       
       
    def control_loop(self):
        """
        Main control loop executing the Bug state machine.
        Called periodically by the timer.
        Handles goal transformation, state transitions, and velocity command publishing.
        """
        # Wait for essential data
        if self.scan_data is None or self.current_pose is None or self.odom_frame is None:
            self.get_logger().warn("Waiting for scan, odom, or odom_frame data...", throttle_duration_sec=2)
            return

        # Wait for a goal to be published
        if self.goal_pose_stamped is None:
            self.get_logger().info("Waiting for goal to be set via /goal_pose...", throttle_duration_sec=5)
            # Ensure robot doesn't move if no goal is set
            if self.state != BugState.GOAL_REACHED: # Avoid spamming stop if already stopped
                self.stop_robot()
            return

        # --- Goal Transformation ---
        target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        if self.goal_internal is None: # Transform only if we haven't already or goal is new
            try:
                # Ensure odom frame matches target frame for simplicity in this example
                if self.odom_frame != target_frame:
                        self.get_logger().error(
                            f"Odometry frame '{self.odom_frame}' does not match target frame '{target_frame}' for goal transformation. "
                            f"This node currently expects goals to be transformed into the odom frame '{self.odom_frame}'. Adjust 'target_frame' parameter if needed.",
                            once=True)
                        self.stop_robot()
                        return

                # Use do_transform_pose_stamped helper
                transformed_goal_pose_stamped = self.tf_buffer.transform(
                    self.goal_pose_stamped,
                    target_frame,
                    timeout=rclpy.duration.Duration(seconds=0.5) # Reduced timeout
                )
                self.goal_internal = transformed_goal_pose_stamped.pose.position # Use the position part
                self.get_logger().info(f"Goal successfully transformed to '{target_frame}' frame.")

            except TransformException as e:
                self.get_logger().error(f"Could not transform goal from '{self.goal_pose_stamped.header.frame_id}' to '{target_frame}': {e}")
                self.stop_robot() # Stop if we can't transform the goal
                return
            except Exception as e:
                self.get_logger().error(f"An unexpected error occurred during goal transformation: {e}")
                self.stop_robot()
                return


        # --- Goal Reached Check ---
        dist_to_goal = self.calculate_distance(self.current_pose, self.goal_internal)
        goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value

        if dist_to_goal < goal_tolerance:
            if self.state != BugState.GOAL_REACHED:
                self.get_logger().info(f"Goal Reached! Distance: {dist_to_goal:.2f} < {goal_tolerance:.2f}")
                self.state = BugState.GOAL_REACHED
                self.stop_robot()
            return # Stay stopped if reached

        # --- State Machine Logic (using self.goal_internal now) ---
        twist_msg = Twist()
        linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        wall_follow_dist = self.get_parameter('wall_follow_dist').get_parameter_value().double_value
        max_scan_range = self.get_parameter('max_scan_range').get_parameter_value().double_value

        if self.state == BugState.GO_TO_GOAL:
            self.get_logger().debug("State: GO_TO_GOAL", throttle_duration_sec=1)

            # Obstacle Check
            front_clear = True
            num_ranges = len(self.scan_data.ranges)
            angle_increment = self.scan_data.angle_increment
            min_range = self.scan_data.range_min
            max_range = self.scan_data.range_max # Use actual scan max range
            obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value

            # Add check for valid laser scan parameters
            if num_ranges == 0 or angle_increment == 0.0:
                self.get_logger().warn("Invalid laser scan parameters...", throttle_duration_sec=5)
            else:
                # Calculate index corresponding to 0 angle (center)
                # Assuming angle_min is negative and angle_max is positive
                center_angle = 0.0
                center_index = int((-self.scan_data.angle_min) / angle_increment)

                check_angle_range = math.radians(15.0) # Check +/- 15 degrees
                num_steps_check = int(check_angle_range / angle_increment)

                self.get_logger().debug(f"Obstacle Check: CenterIdx={center_index}, NSteps={num_steps_check}, Thresh={obstacle_threshold:.2f}")

                for i in range(num_steps_check + 1):
                    # Check index i steps to the right of center
                    idx_right = (center_index + i) % num_ranges
                    # Check index i steps to the left of center
                    idx_left = (center_index - i + num_ranges) % num_ranges # Ensure positive index

                    # Get ranges, default to max_range if index is bad
                    range_right = self.scan_data.ranges[idx_right] if 0 <= idx_right < num_ranges else max_range
                    range_left = self.scan_data.ranges[idx_left] if 0 <= idx_left < num_ranges else max_range

                    # Handle inf/nan explicitly -> treat as no obstacle detected at this specific beam
                    if math.isinf(range_right) or math.isnan(range_right): range_right = max_range
                    if math.isinf(range_left) or math.isnan(range_left): range_left = max_range

                    # Log checked values
                    self.get_logger().debug(f"  Check i={i}: L_idx={idx_left}, L_val={range_left:.2f} | R_idx={idx_right}, R_val={range_right:.2f}", throttle_duration_sec=0.1)

                    # Check if valid range and AT or BELOW threshold
                    # Use <= threshold to catch obstacles right at the limit
                    # Ensure range is also above the minimum valid reading distance
                    if (min_range < range_right <= obstacle_threshold or
                        min_range < range_left <= obstacle_threshold):
                        front_clear = False
                        self.get_logger().info(f"Obstacle detected ahead! Check i={i}. Dist: min({range_right:.2f}, {range_left:.2f}) <= {obstacle_threshold:.2f}")
                        break # Obstacle found

            if not front_clear:
                self.get_logger().info("Transitioning to WALL_FOLLOW state.")
                self.state = BugState.WALL_FOLLOW
                self.obstacle_hit_point = self.current_pose # Record hit location
                # Recalculate dist_to_goal when hitting obstacle
                self.initial_goal_dist = self.calculate_distance(self.current_pose, self.goal_internal)
                self.stop_robot()
                return

            else:
                # Movement Logic (uses self.goal_internal)
                angle_to_goal = math.atan2(self.goal_internal.y - self.current_pose.y,
                                            self.goal_internal.x - self.current_pose.x)
                angle_diff = self.normalize_angle(angle_to_goal - self.current_yaw)
                turn = angular_speed * angle_diff
                turn = max(-angular_speed, min(angular_speed, turn))
                twist_msg.angular.z = turn

                if abs(angle_diff) < math.radians(10.0):
                    twist_msg.linear.x = linear_speed
                else:
                    twist_msg.linear.x = 0.0

        elif self.state == BugState.WALL_FOLLOW:
            self.get_logger().debug("State: WALL_FOLLOW", throttle_duration_sec=1)

            # First, check if we can leave the wall (Tangent Bug condition)
            if self.can_leave_wall():
                self.get_logger().info("Transitioning back to GO_TO_GOAL state.")
                self.state = BugState.GO_TO_GOAL

                # Give a slight turn towards goal before moving
                angle_to_goal = math.atan2(self.goal_internal.y - self.current_pose.y,
                                            self.goal_internal.x - self.current_pose.x)
                angle_diff = self.normalize_angle(angle_to_goal - self.current_yaw)
                twist_msg.angular.z = max(-angular_speed, min(angular_speed, angular_speed * angle_diff))
                twist_msg.linear.x = 0.0 # Just turn initially
                self.cmd_vel_publisher.publish(twist_msg)
                return # Exit loop for this cycle

            # --- Wall Following Control (Keep Wall to the Right) ---
            # Find the minimum distance in the front-right quadrant
            front_dist = max_scan_range
            right_dist = max_scan_range
            num_ranges = len(self.scan_data.ranges)

            # Define angular ranges for front and right sensors
            # Example: Front = -15 to +15 deg, Right = -90 to -45 deg
            front_range_start_angle = math.radians(-15.0)
            front_range_end_angle = math.radians(15.0)
            right_range_start_angle = math.radians(-90.0)
            right_range_end_angle = math.radians(-45.0)

            min_right_dist_idx = -1

            for i, scan_range in enumerate(self.scan_data.ranges):
                angle = self.scan_data.angle_min + i * self.scan_data.angle_increment
                # Use max range if scan is invalid
                if scan_range < self.scan_data.range_min or scan_range > max_scan_range:
                    scan_range = max_scan_range

                # Check Front
                if front_range_start_angle <= angle <= front_range_end_angle:
                    front_dist = min(front_dist, scan_range)

                # Check Right
                if right_range_start_angle <= angle <= right_range_end_angle:
                    if scan_range < right_dist:
                        right_dist = scan_range
                        min_right_dist_idx = i # Store index of closest point on right

            # ============================================================
            # TODO (Student Exercise): Implement Wall Following Behavior
            # ============================================================
            # Use a proportional controller to keep the robot at 'wall_follow_dist'
            # from the wall on its right side.
            #
            # Available variables:
            #   - right_dist:       closest distance measured on the robot's right side
            #   - front_dist:       closest distance measured in front of the robot
            #   - wall_follow_dist: desired distance to maintain from the wall
            #   - angular_speed:    maximum angular velocity (rad/s)
            #   - linear_speed:     maximum linear velocity (m/s)
            #
            # Steps:
            #   1. Compute the error between the desired wall distance and the
            #      actual right-side distance.
            #   2. Use the error to calculate a proportional turning command
            #      (twist_msg.angular.z). Clamp it to a reasonable range.
            #   3. Set the forward speed (twist_msg.linear.x), reducing it when
            #      the robot is close to a wall in front or turning sharply.
            #
            # Hint: error = wall_follow_dist - right_dist
            # ============================================================
            # 1) Distance error: positive => too far from wall, negative => too close
            error = wall_follow_dist - right_dist
            kp = 2.0
            turn = -kp * error  # negative because wall is on the right
            turn = max(-angular_speed, min(angular_speed, turn))
            twist_msg.angular.z = turn

            # 2) Slow down near front obstacles or during sharp turns
            if front_dist < wall_follow_dist * 1.5:
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = angular_speed  # in-place left turn at corner
            elif abs(turn) > 0.5 * angular_speed:
                twist_msg.linear.x = 0.5 * linear_speed
            else:
                twist_msg.linear.x = linear_speed

            self.get_logger().debug(
                f"[WF] R:{right_dist:.2f} F:{front_dist:.2f} Err:{error:.2f} "
                f"TurnCmd:{turn:.3f} LinCmd:{twist_msg.linear.x:.3f}",
                throttle_duration_sec=0.2)
            
            
        elif self.state == BugState.GOAL_REACHED:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0

        if self.state != BugState.GOAL_REACHED:
            self.cmd_vel_publisher.publish(twist_msg)

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)
        self.get_logger().info("Stopping robot.")

def main(args=None):
    rclpy.init(args=args)
    bug_planner_node = BugPlannerNode()
    try:
        rclpy.spin(bug_planner_node)
    except KeyboardInterrupt:
        bug_planner_node.stop_robot()
        bug_planner_node.get_logger().info("Bug planner node stopped.")
    finally:
        bug_planner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

