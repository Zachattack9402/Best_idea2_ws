import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import numpy as np
from scipy.spatial import KDTree


# ICP Algorithm
class Align2D:
    def __init__(self, source_points, target_points, initial_T):
        self.source = np.hstack([source_points, np.ones((source_points.shape[0],1))])
        self.target = np.hstack([target_points, np.ones((target_points.shape[0],1))])
        self.init_T = initial_T
        self.target_tree = KDTree(target_points[:, :2])
        self.transform = self.AlignICP(20, 1.0e-4)

    def AlignICP(self, max_iter, min_delta_err):
        mean_sq_error = 1.0e6
        delta_err = 1.0e6
        T = self.init_T
        num_iter = 0
        tf_source = self.source

        while delta_err > min_delta_err and num_iter < max_iter:
            matched_trg_pts, matched_src_pts, indices = self.FindCorrespondences(tf_source)
            new_T = self.AlignSVD(matched_src_pts, matched_trg_pts)
            T = np.dot(T, new_T)
            tf_source = np.dot(self.source, T.T)
            new_err = 0
            for i in range(len(indices)):
                if indices[i] != -1:
                    diff = tf_source[i, :2] - self.target[indices[i], :2]
                    new_err += np.dot(diff, diff.T)
            new_err /= float(len(matched_trg_pts))
            delta_err = abs(mean_sq_error - new_err)
            mean_sq_error = new_err
            num_iter += 1

        return T

    def FindCorrespondences(self, src_pts):
        matched_src_pts = src_pts[:, :2]
        dist, indices = self.target_tree.query(matched_src_pts)
        unique = False
        while not unique:
            unique = True
            for i in range(len(indices)):
                if indices[i] == -1:
                    continue
                for j in range(i + 1, len(indices)):
                    if indices[i] == indices[j]:
                        if dist[i] < dist[j]:
                            indices[j] = -1
                        else:
                            indices[i] = -1
                            break
        point_list = []
        src_idx = 0
        for idx in indices:
            if idx != -1:
                point_list.append(self.target[idx, :])
                src_idx += 1
            else:
                matched_src_pts = np.delete(matched_src_pts, src_idx, axis=0)
        matched_pts = np.array(point_list)
        return matched_pts[:, :2], matched_src_pts, indices

    def AlignSVD(self, source, target):
        src_centroid = self.GetCentroid(source)
        trg_centroid = self.GetCentroid(target)
        source_centered = source - src_centroid
        target_centered = target - trg_centroid
        M = np.dot(target_centered.T, source_centered)
        U, W, V_t = np.linalg.svd(M)
        R = np.dot(U, V_t)
        t = np.expand_dims(trg_centroid, 0).T - np.dot(R, np.expand_dims(src_centroid, 0).T)
        T = np.identity(3)
        T[:2, 2] = np.squeeze(t)
        T[:2, :2] = R
        return T

    def GetCentroid(self, points):
        point_sum = np.sum(points, axis=0)
        return point_sum / float(len(points))


class ICPNode(Node):
    def __init__(self):
        super().__init__('icp_node')
        #initialize pub/sub for input laserscan and output odom
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10  # QoS
        )
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.last_transformed_points = None  # This will store the last transformed point cloud
        self.last_transform = np.identity(3)  # Identity matrix for the first iteration
        self.get_logger().info('ICP Node Initialized')

    def scan_callback(self, msg: LaserScan):
        self.get_logger().info(f"Received scan: {msg.header.stamp}")

        # Convert LaserScan to PointCloud
        point_cloud = self.laserscan_to_pointcloud(msg)

        # Use last transformed points as the reference for ICP, or identity in the first iteration
        reference_points = self.get_reference_points()
        if not reference_points.any():
                reference_points = point_cloud
        # Apply ICP to align the point cloud
        aligner = Align2D(source_points=point_cloud, target_points=reference_points, initial_T=self.last_transform)
        transform = aligner.transform

        # Compute the odometry based on the ICP transformation
        odom_msg = self.compute_odometry_from_icp(transform)

        # Publish the computed odometry
        self.odom_publisher.publish(odom_msg)
        self.get_logger().info('ICP Node Publishing Odometry')

        # Store the transformed points for the next iteration
       # self.last_transformed_points = np.dot(point_cloud, transform[:2, :2].T) + transform[:2, 2]
	#convert pointns to homogenous representation for matrix multiplication
        source_homogenous = np.hstack([point_cloud, np.ones((point_cloud.shape[0],1))])
        transformed_source = np.dot(source_homogenous, transform.T)
        self.last_transformed_points = transformed_source[:,:2]	#keep 2D points only

        # Update the last transformation for future reference
        self.last_transform = transform

    def laserscan_to_pointcloud(self, msg, voxel_size=0.1):
        """Convert LaserScan to 2D point cloud and apply voxel downsampling."""
        points = []
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        for i, distance in enumerate(msg.ranges):
            if msg.range_min < distance < msg.range_max:
                angle = angle_min + i * angle_increment
                x = distance * np.cos(angle)
                y = distance * np.sin(angle)
                points.append([x, y])

        points = np.array(points)

        if len(points) == 0:
            return points  # Error catching, return empty array if no valid points

        # Apply voxel grid filtering
        return self.voxel_grid_downsample(points, voxel_size)

    def voxel_grid_downsample(self, points, voxel_size):
        """Downsamples a 2D point cloud using a voxel grid filter."""
        if len(points) == 0:
            return points

        # Round points to voxel grid and use unique values
        grid = np.round(points / voxel_size) * voxel_size
        unique_grid = np.unique(grid, axis=0)

        return unique_grid

    def get_reference_points(self):
        """Return the last transformed points as the reference for the ICP algorithm."""
        if self.last_transformed_points is not None:
            return self.last_transformed_points
        else:
            # First iteration: use identity matrix, no transformation
            return np.zeros((1,1))

    def compute_odometry_from_icp(self, transform):
        """Compute odometry message from ICP transform."""
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = rclpy.time.Time().to_msg()
        odom_msg.header.frame_id = 'odom'

        # Apply rotation and translation from the ICP transformation
        position = transform[:2, 2]
        rotation = transform[:2, :2]

        # Convert the rotation matrix to a quaternion (simplified for 2D)
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
        quaternion = self.euler_to_quaternion(0.0, 0.0, yaw)

        # Set the pose (with translation and rotation)
        odom_msg.pose.pose.position.x = position[0]
        odom_msg.pose.pose.position.y = position[1]
        odom_msg.pose.pose.orientation.x = quaternion[0]
        odom_msg.pose.pose.orientation.y = quaternion[1]
        odom_msg.pose.pose.orientation.z = quaternion[2]
        odom_msg.pose.pose.orientation.w = quaternion[3]

        # Set velocity (linear and angular velocity)
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        return odom_msg

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
            yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
            yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
            yaw / 2)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node = ICPNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
