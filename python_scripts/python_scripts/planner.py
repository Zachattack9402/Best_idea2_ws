import sys
import rclpy
import time		#for findFire
import numpy as np	#for thermalCam
import busio
import adafruit_mlx90640

from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from python_scripts.thermalCam import fireInFrame

def create_pose(x, y, yaw):
    pose = Pose()
#    pose.header.frame_id = frame_id
#    pose.header.stamp = rclpy.time.Time().to_msg()

    pose.position.x = x
    pose.position.y = y

    quat = quaternion_from_euler(0, 0, yaw)
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    return pose

class PlannerNode(Node):
    #determine and remember goals, use cmd_vel to find fire, writes goal pose to /goal_topic
    def __init__(self):
        super().__init__('planner_node')
        self.odometry_subscription = self.create_subscription(  # need to track position of robot
            Odometry,
            '/odometry/filtered',
            self.odometry_callback,
            10
        )

        self.posePublisher = self.create_publisher(Pose, 'goal_topic', 10)
        self.drivePublisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.temperature_threshold = 30
        self.min_fire_pixels = 5
        self.fire_pose = Pose()
        self.homePose = create_pose(0.0, 0.0, 0.0)
        self.goal_pose = create_pose(0.0, 0.0, 0.0)

        self.robot_pose = [0.0, 0.0, 0.0]
        self.round = "ROUND1"
        self.goal_counter = 0
        self.goal_threshold = .2


    def odometry_callback(self, msg):
         # Constantly updates robot position, and checks if goal has been reached.
         # TODO: pushbutton to enable first state!!!

        goalx = self.goal_pose.position.x
        goaly = self.goal_pose.position.y
        self.get_logger().info(str(goalx)+' '+str(goaly))
        #if at goal
        if abs(goalx-self.robot_pose[0]) < self.goal_threshold and abs(goaly - self.robot_pose[1]) <= self.goal_threshold:
            self.get_logger().info('Goal reached')
            self.goal_counter += 1
            #Check what round it is
            if self.round == "ROUND1":
                if self.goal_counter == 1:  #first, go to fire
                    self.fire_pose = self.find_fire()
                    self.get_logger().info('Fire found, begin avoidance')
                    self.goal_pose = self.fire_pose
                    self.posePublisher.publish(self.fire_pose)
                elif self.goal_counter == 2:                       #second, go home
                    self.get_logger().info('Fire found, going home')
                    self.goal_pose = self.homePose
                    self.posePublisher.publish(self.homePose)

            if self.round == "ROUND2":
                if self.goal_counter == 1:  #first, go to fire
                    self.get_logger().info('Fire from memory, begin avoidance')
                    self.goal_pose = self.fire_pose
                    self.posePublisher.publish(self.fire_pose)
                elif self.goal_counter == 2:                       #second, go home
                    self.get_logger().info('Fire found, going home')
                    self.goal_pose = self.homePose
                    self.posePublisher.publish(self.homePose)

            if self.round == "ROUND3":
                if self.goal_counter == 1:
                    latchPose = create_pose(1.2, -0.49, 3.14)    #avoid to hose
                    self.get_logger().info('Fire from memory, begin avoidance (latch)')
                    self.goal_pose = latchPose
                    self.posePublisher.publish(latchPose)
                elif self.goal_counter == 2:
                    #TODO: see below. Develop hose methods
                    latch_hose()                        #latch hose, go to fire
                    self.get_logger().info('Hose latched, continue avoidance (fire)')
                    self.goal_pose = self.fire_pose
                    self.posePublisher.publish(self.fire_pose)
                elif self.goal_counter == 3:
                    self.get_logger().info('Fire found, dropping hose')     #made it to fire, go home
                    drop_hose()
                    self.get_logger().info('Hose dropped, going home')
                    self.goal_pose = self.homePose
                    self.posePublisher.publish(self.homePose)

                self.robot_pose[0] = msg.pose.pose.position.x
                self.robot_pose[1] = msg.pose.pose.position.y

        #TODO: Test
    def find_fire(self):
        self.get_logger().info('Finding fire')
        threshold = self.temperature_threshold
        min_pixels = self.min_fire_pixels
        twist_msg = Twist()

        #First, check if fire is ahead (Top location)
        if fireInFrame(threshold, min_pixels):
            firePose = create_pose(2.44,0.0,3.14)
            return firePose

        #if not:
        #turn until 45 degrees to right (Diagonal location)
        twist_msg.angular.z = 0.4
        self.drivePublisher.publish(twist_msg)
        time.sleep(1)
        twist_msg.angular.z = 0.0
        self.drivePublisher.publish(twist_msg)
        if fireInFrame(threshold, min_pixels):
            firePose = create_pose(2.44,2.44,3.14)
            return firePose

        #if not:
        #turn until 45 degrees to right (Right location)
        twist_msg.angular.z = 0.4
        self.drivePublisher.publish(twist_msg)
        time.sleep(1)
        twist_msg.angular.z = 0.0
        self.drivePublisher.publish(twist_msg)
        if fireInFrame(threshold, min_pixels):
            firePose = create_pose(0.0,2.44,3.14/2)
            return firePose

        #else: reattempt
        self.get_logger().info("Planner: Fire not found. Trying again.")
        twist_msg.angular.z = -0.4
        self.drivePublisher.publish(twist_msg)
        time.sleep(2)
        twist_msg.angular.z = 0.0
        self.drivePublisher.publish(twist_msg)
        return self.find_fire()

    #TODO: Develop hose methods
    def latch_hose(self):
        pass

    def drop_hose(self):
        pass


def main():
    rclpy.init()
    node = PlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
