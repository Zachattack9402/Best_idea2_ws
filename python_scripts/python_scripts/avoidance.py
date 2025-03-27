import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry


class ObjectAvoidanceNode(Node):
    #read /scan, read /goal_topic, write /cmd_vel
    def __init__(self):
        super().__init__('object_avoidance_node')
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.goal_subsription = self.create_subscription( #subscribe to topic to get pose goal
            Pose,
            '/goal_topic',
            self.goal_callback,
            10
        )
        self.odometry_subsription = self.create_subscription(#need to track position of robot
            Odometry,
            '/odometry/filtered',
            self.odometry_callback,
            10
        )

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.safe_distance = 0.3  # Meters
        self.FOV = 22             # Front Field of View
        self.driveVel = 0.3       # forward/backward velocity
        self.turnVel = 0.4        # turn velocity

        self.robot_pose = [0,0,0]
        self.goal_pose = [6,0,0]
        self.goal_threshold = .1    #radius around goal for goal-completion

        self.state = "MOVING_FORWARD"  # Initial state
        self.get_logger().info('Object Avoidance Node Started')

    #When /goal_topic is published to, write down new goal and turn towards it
    def goal_callback(self, msg):
        self.goal_pose[0] = msg.pose.pose.position.x
        self.goal_pose[1] = msg.pose.pose.position.y
        self.goal_pose[2] = 2*np.arcsin(msg.pose.pose.orientation.z) #in radians?

        self.state = "TURNING"
        self.get_logger().info("New goal received: ({}, {})".format(str(self.goal_pose[0]), str(self.goal_pose[1])))

    #update robot pose
    def odometry_callback(self, msg):
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        self.robot_pose[2] = 2*np.arccos(msg.pose.pose.orientation.z) #in radians?

    #calculate what direction goal is in
    def rotate_to_goal(self):
        #calculate vector pointing from robot to goal
        magnitude = np.sqrt((self.goal_pose[0]-self.robot_pose[0])**2 + (self.goal_pose[1]-self.robot_pose[1]**2))
        direction = np.arccos((self.goal_pose[0]-self.robot_pose[0])/magnitude)   #in radians
        radianDiff = np.arctan2(np.sin(direction - self.robot_pose[2]), np.cos(direction - self.robot_pose[2])) #returns smallest difference radians from pose to goal
        if radianDiff < 0:   return -1      #turn left
        elif radianDiff > 0: return 1       #turn right


    ####main motion/decision-making every LiDAR publish
    def lidar_callback(self, msg):
        ranges = msg.ranges
        frontmin_distance = min(ranges[90 - self.FOV: 90 + self.FOV])  # 90 degrees is forward
        leftmin_distance = min(ranges[0: 90 - self.FOV])               # left closest object
        rightmin_distance = min(ranges[90 + self.FOV: 180])            # right closest object
        twist_msg = Twist()


        #check if goal met
        if (self.goal_pose[0]-self.robot_pose[0] <= self.goal_threshold and self.goal_pose[1] - self.robot_pose[1] <= self.goal_threshold):
            self.state = "GOAL_FOUND"


        if self.state == "MOVING_FORWARD":
            if frontmin_distance < self.safe_distance:
                # Obstacle detected, switch to turning state
                self.state = "TURNING"
            else:       #if moving forward and no object, drive forward
                twist_msg.linear.x = self.driveVel
                twist_msg.angular.z = 0.0

        if self.state == "TURNING":
            # Stop turning and wait for the next scan before deciding to move forward
            #twist_msg.linear.x = 0.0
            #twist_msg.angular.z = 0.0
            if leftmin_distance < self.safe_distance and rightmin_distance < self.safe_distance:
                twist_msg.linear.x = -self.driveVel  # Backup if surrounded
                twist_msg.angular.z = 0.0
            elif leftmin_distance < self.safe_distance+.1:      #Make it look a little bit further for side obstacles
                twist_msg.angular.z = self.turnVel  # Turn right
            elif rightmin_distance < self.safe_distance+.1:
                twist_msg.angular.z = -self.turnVel  # Turn left
            else:
                twist_msg.angular.z = self.turnVel*ObjectAvoidanceNode.rotate_to_goal()  #turn towards goal if no side obstacles

            if min(ranges[int(90-1*self.FOV):int(90+1*self.FOV)]) >= self.safe_distance:      #extra wide FOV to verify no objects in corners
                self.state = "MOVING_FORWARD"

        distance = np.sqrt((self.goal_pose[0]-self.robot_pose[0])**2 + (self.goal_pose[1]-self.robot_pose[1]**2))
        self.get_logger().info(self.state + str(distance))
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
