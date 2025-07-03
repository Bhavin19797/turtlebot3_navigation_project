#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import heapq

class GlobalPlanner:
    def __init__(self):
        rospy.init_node('global_planner')
        rospy.set_param('/use_sim_time', True)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=10, latch=True)

    def map_callback(self, msg):
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.width, self.height = msg.info.width, msg.info.height
        grid = np.array(msg.data).reshape((self.height, self.width))
        self.map_data = np.where(grid >= 100, 1, 0)
        rospy.loginfo("Map received")

        start_world = (-2.0, -0.5)
        goal_world = (1.5, 1.5)  # Keep inside map

        start = self.world_to_grid(start_world)
        goal = self.world_to_grid(goal_world)

        if not self.is_valid(start):
            rospy.logwarn("Start invalid.")
            return
        if not self.is_valid(goal):
            rospy.logwarn("Goal invalid.")
            return

        path = self.a_star(start, goal)
        if path:
            world_path = [self.grid_to_world(p) for p in path[::2]]  # Reduce waypoints
            rospy.loginfo("Path planned with %d waypoints", len(world_path))
            self.publish_path(world_path)
        else:
            rospy.logwarn("No path found.")

    def is_valid(self, cell):
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height and self.map_data[y][x] == 0

    def world_to_grid(self, pos):
        x = int((pos[0] - self.origin[0]) / self.resolution)
        y = int((pos[1] - self.origin[1]) / self.resolution)
        return (x, y)

    def grid_to_world(self, cell):
        x = cell[0] * self.resolution + self.origin[0]
        y = cell[1] * self.resolution + self.origin[1]
        return (x, y)

    def get_neighbors(self, cell):
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        return [(cell[0]+dx, cell[1]+dy) for dx, dy in directions
                if self.is_valid((cell[0]+dx, cell[1]+dy))]

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def a_star(self, start, goal):
        queue = []
        heapq.heappush(queue, (0, start))
        came_from = {}
        cost = {start: 0}

        while queue:
            _, current = heapq.heappop(queue)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            for neighbor in self.get_neighbors(current):
                new_cost = cost[current] + 1
                if neighbor not in cost or new_cost < cost[neighbor]:
                    cost[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor, goal)
                    heapq.heappush(queue, (priority, neighbor))
                    came_from[neighbor] = current
        return None

    def publish_path(self, waypoints):
        msg = Path()
        msg.header.frame_id = "map"
        for x, y in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)
        rospy.loginfo("Published path to /planned_path")

if __name__ == '__main__':
    GlobalPlanner()
    rospy.spin()
