# *****************************************************************
#  Processes Map Data to find Frontiers
#
#  explorer.py
#  
#  Based on Autonomous-Explorer-and-Mapper-ros2-nav2 by Ani Arka
#  https://github.com/AniArka/Autonomous-Explorer-and-Mapper-ros2-nav2.git
#
# *****************************************************************

import rclpy , math
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
from collections import deque

class ExplorerNode(Node):
    def __init__(self, args=None):
        super().__init__('explorer')
        self.get_logger().info("Explorer Node Started")

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.pose_callback, 10)

        # Action client for Nav2 navigation
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # Configuration: minimum cell distance threshold
        # Any frontier closer than this (in grid cells) will be ignored
        self.min_frontier_cell_distance = 20  # adjust as needed
        self.max_frontier_cell_distance = 100

        self.min_cluster_size = 50
        self.max_cluster_size = 1600

        self.dist_weight = 1.0 #1.0
        self.size_weight = 1.5 # 1.5
        self.heading_weight = 4.0 #4.0

        self.robot_yaw      = 0.0

        # State
        self.map_data = None
        self.robot_position = (0, 0)    # in grid indices
        self.visited_frontiers = set()

        # Timer for exploration cycles
        self.timer = self.create_timer(5.0, self.explore)

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg
        self.get_logger().info("Map received")

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        # Extract world position and convert to grid indices
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.robot_yaw = math.atan2(siny, cosy)

        if self.map_data:
            origin = self.map_data.info.origin.position
            res = self.map_data.info.resolution
            row = int((y - origin.y) / res)
            col = int((x - origin.x) / res)
            self.robot_position = (row, col)
        else:
            self.get_logger().warning(
                "Pose received before map; cannot convert to grid indices.")

    def explore(self):

        if not self.map_data:
            self.get_logger().warning("No map data available")
            return

        # Build 2D map array
        h = self.map_data.info.height
        w = self.map_data.info.width
        grid = np.array(self.map_data.data, dtype=np.int8).reshape((h, w))

        # Detect frontiers
        frontiers = self.find_frontiers(grid)
        if not frontiers:
            self.get_logger().info("No frontiers found. Moving to Start!")
            chosen = (0,0) # origin / start
        else:
            # Select frontier with clustering + proximity, respecting min distance
            chosen = self.choose_frontier(frontiers)

        if not chosen:
            self.get_logger().warning("No valid frontier chosen within threshold. Looking further")
            self.max_frontier_cell_distance *= 3
            return
        else:
            self.max_frontier_cell_distance = 100


        # Convert chosen cell to world coordinates
        r, c = chosen
        goal_x = c * self.map_data.info.resolution + self.map_data.info.origin.position.x
        goal_y = r * self.map_data.info.resolution + self.map_data.info.origin.position.y
        self.navigate_to(goal_x, goal_y)
        return

    def find_frontiers(self, grid):
        frontiers = []
        rows, cols = grid.shape
        for r in range(1, rows-1):
            for c in range(1, cols-1):
                if grid[r, c] == 0:
                    neigh = grid[r-1:r+2, c-1:c+2].flatten()
                    if -1 in neigh:
                        frontiers.append((r, c))
        self.get_logger().info(f"Found {len(frontiers)} raw frontiers")
        return frontiers

    def bfs_clusters(self, frontiers):
        # Group contiguous frontier cells via BFS (4-connectivity)
        frontier_set = set(frontiers)
        clusters = []
        while frontier_set:
            seed = frontier_set.pop()
            queue = deque([seed])
            cluster = {seed}
            while queue:
                r, c = queue.popleft()
                for dr, dc in [(1,0),(-1,0),(0,1),(0,-1)]:
                    nb = (r+dr, c+dc)
                    if nb in frontier_set:
                        frontier_set.remove(nb)
                        cluster.add(nb)
                        queue.append(nb)
            clusters.append(cluster)
        return clusters

    def choose_frontier(self, frontiers):
        # 1) Cluster contiguous frontier cells
        clusters = self.bfs_clusters(frontiers)
        if not clusters:
            return None

        robot_row, robot_col = self.robot_position

        # 2) Score each cluster
        scored = []
        for cluster in clusters:
            pts = np.array(list(cluster))            # shape (N,2) of (row,col)
            size = pts.shape[0]                      # number of cells in cluster
            if size > self.max_cluster_size or size < self.min_cluster_size:
                continue

            size = size / self.max_cluster_size  # normalize 0-1

            centroid = pts.mean(axis=0)              # [mean_row, mean_col]
            # distance (in cells) from robot to centroid
            dy = centroid[0] - robot_row
            dx = centroid[1] - robot_col
            dist = np.linalg.norm([dy,dx])

            # ignore clusters too close
            if dist < self.min_frontier_cell_distance or dist > self.max_frontier_cell_distance:
                continue

            dist = dist/self.max_frontier_cell_distance # normalize 0-1

            desired = math.atan2(dy, dx)
            delta   = abs(self._normalize_angle(desired - self.robot_yaw))
            heading_score = math.cos(delta)

            score = self.size_weight * size - self.dist_weight * dist + self.heading_weight * heading_score
            scored.append((score, centroid, cluster, dist))

        if not scored:
            return None

        # 4) pick the best‐scoring cluster
        best_score, best_centroid, best_cluster, best_dist = max(
            scored, key=lambda x: x[0]
        )

        # 5) within that cluster, pick the cell nearest the centroid
        pts = np.array(list(best_cluster))
        # compute distance from each cell to that centroid
        d_to_centroid = np.linalg.norm(pts - best_centroid, axis=1)
        idx = int(np.argmin(d_to_centroid))
        chosen_cell = tuple(pts[idx].astype(int))  # (row, col)

        # 6) log and return
        self.visited_frontiers.add(chosen_cell)
        self.get_logger().info(
            f"Chosen frontier {chosen_cell} at distance "
            f"{best_dist:.2f} cells (score={best_score:.2f})"
        )
        return chosen_cell
    

    def navigate_to(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal

        self.get_logger().info(f"Navigating to x={x:.2f}, y={y:.2f}")
        self.nav_to_pose_client.wait_for_server()
        future = self.nav_to_pose_client.send_goal_async(nav_goal)
        future.add_done_callback(self.goal_response_callback)
        

    def goal_response_callback(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warning("Goal rejected")
            return
        self.get_logger().info("Goal accepted")
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(self.navigation_complete_callback)

    def navigation_complete_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f"Navigation result: {result}")
        except Exception as e:
            self.get_logger().error(f"Navigation failed: {e}")

    def _normalize_angle(self, a):
        return (a + math.pi) % (2*math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        node.get_logger().info("Explorer node running...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
