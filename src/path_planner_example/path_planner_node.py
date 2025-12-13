#!/usr/bin/env python3
import time
import math
import heapq
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from create_plan_msgs.srv import CreatePlan
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.msg import Costmap


class PathPlannerNode(Node):

    def __init__(self):
        super().__init__("path_planner_node")
        self.basic_navigator = BasicNavigator()  # Can be uncommented to get Global Costmap in create_plan_cb

        # Creating a new service "create_plan", which is called by our Nav2 C++ planner plugin
        # to receive a plan from us.
        self.srv = self.create_service(CreatePlan, 'create_plan', self.create_plan_cb)

        # qos_profile = QoSProfile(
        #     reliability = ReliabilityPolicy.RELIABLE,
        #     durability = DurabilityPolicy.TRANSIENT_LOCAL,
        #     history = HistoryPolicy.KEEP_LAST,
        #     depth = 1
        # )
        # self.costmap_sub = self.create_subscription(
        #     OccupancyGrid,
        #     '/global_costmap/costmap',
        #     self.costmap_cb,
        #     qos_profile
        # )
        # self.latest_costmap = None
        #self.get_logger().info("PathPlannerNode started and waiting /global_costmap/costmap...")
        self.get_logger().info("PathPlannerNode started and Nav2 is active.")
    # def costmap_cb(self, msg):
    #     self.latest_costmap = msg

    def create_plan_cb(self, request, response):
        self.get_logger().info("Received plan request!")

        # Getting all the information to plan the path
        goal_pose = request.goal
        start_pose = request.start
        time_now = self.get_clock().now().to_msg()

        # if self.latest_costmap is None:
        #     self.get_logger().error("No costmap received yet, cannot plan!")

        #     response.path = Path()
        #     response.path.header.frame_id = start_pose.header.frame_id
        #     response.path.header.stamp = time_now
        #     return response

        global_costmap = self.basic_navigator.getGlobalCostmap()

        if global_costmap is None:
            self.get_logger().error("BasicNavigator failed to retrieve Costmap. Cannot plan!")

            response.path = Path()
            response.path.header.frame_id = start_pose.header.frame_id
            response.path.header.stamp = time_now
            return response

        path = create_astar_plan(start_pose, goal_pose, time_now, global_costmap, self)

        if path:
            response.path = path
            self.get_logger().info(f"A* path length: {len(path.poses)}. Path found and returned.")
        else:
            self.get_logger().warn("No path found.")

        # global_costmap = self.basic_navigator.getGlobalCostmap()  # Can be uncommented to get Global CostMap

        # response.path = create_astar_plan(start_pose, goal_pose, time_now, global_costmap, self)
        return response

def create_straight_plan(start, goal, time_now):
    """ 
    Creates a straight plan between start and goal points.
    Does not use the global costmap to plan around obstacles, as normal planners would.
    """
    path = Path()

    # Set the frame_id and stamp for the Path header. Use frame_id from the goal header,
    #  and time_now for the current time.
    path.header.frame_id = goal.header.frame_id
    path.header.stamp = time_now

    # Let's create a straight plan between our start and goal poses.
    # It is not enough if we provide only the start and end positions as a path.
    # For controller to follow path correctly, we will need to provide also
    # points along this straight path with small intervals. There is a function
    # "interpolate_coordinates" implemented for you that does this. It only needs
    # the coordinates in a tuple format, for example:
    # interpolate_coordinates((0, 0), (0, 0.5))
    # This will give you coordinates between these two points with 0.1 interval:
    # [(0.0, 0.0), (0.0, 0.1), (0.0, 0.2), (0.0, 0.3), (0.0, 0.4), (0.0, 0.5)]
    # Interpolate the coordinates between start and goal positions
    interpolated_coordinates = interpolate_coordinates(
        (start.pose.position.x, start.pose.position.y),
        (goal.pose.position.x, goal.pose.position.y),
    )
    
    # Loop through these interpolated coordinates and create a new PoseStamped()
    #  message for each of them. You can set the same stamp and frame_id as for the Path().
    #  Finally, add all of these points into the path.poses -array.
    for point in interpolated_coordinates:
        pose = PoseStamped()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.header.stamp = time_now
        pose.header.frame_id = goal.header.frame_id
        path.poses.append(pose)

    return path

def create_astar_plan(start: PoseStamped, goal: PoseStamped, time_now, costmap: OccupancyGrid, node: Node) -> Optional[Path]:
    grid, width, height, resolution, origin = convert_costmap_to_grid(costmap, node)
    if grid is None:
        node.get_logger().error("Costmap conversion failed.")
        return None

    start_xy = (start.pose.position.x, start.pose.position.y)
    goal_xy = (goal.pose.position.x, goal.pose.position.y)

    start_cell = world_to_map(start_xy, origin, resolution, width, height)
    goal_cell = world_to_map(goal_xy, origin, resolution, width, height)

    if start_cell is None or goal_cell is None:
        node.get_logger().error("start or goal outside of costmap bounds.")
        return None

    sx, sy = start_cell
    gx, gy = goal_cell

    if grid[sy][sx] == 1:
        node.get_logger().warn("Start cell is occupied.")
    if grid[gy][gx]== 1:
        node.get_logger().warn("Goal cell is occupied.")

    path_cells = astar(grid, start_cell, goal_cell, allow_diagonal=True)

    if path_cells is None:
        node.get_logger().warn("A* returned no path.")
        return None

    path_msg = Path()
    path_msg.header.frame_id = goal.header.frame_id
    path_msg.header.stamp = time_now

    for (mx, my) in path_cells:
        wx, wy = map_to_world((mx, my), origin, resolution)
        pose = PoseStamped()
        pose.header.frame_id = goal.header.frame_id
        pose.header.stamp = time_now
        pose.pose.position.x = wx
        pose.pose.position.y = wy
        pose.pose.orientation.w = 1.0
        path_msg.poses.append(pose)

    node.get_logger().info(f"A* path length: {len(path_msg.poses)}")
    return path_msg

def convert_costmap_to_grid(costmap: Costmap, node: Node) -> Tuple[Optional[List[List[int]]], int, int, float, Tuple[float,float]]:
    """
    Convert Path to 2D list grid[rows][cols] with 0 = free, 1 = obstacle.
    Returns (grid, width, height, resolution, origin_xy)
    """

    node.get_logger().info(f"Received costmap object type: {type(costmap)}")
    try:
        # Pura tiedot Costmap-viestin metadata-kentästä
        width = costmap.metadata.size_x
        height = costmap.metadata.size_y
        resolution = costmap.metadata.resolution
        origin_x = costmap.metadata.origin.position.x
        origin_y = costmap.metadata.origin.position.y
        
        # Data on costmap-viestin data-kentässä (1D lista)
        data = costmap.data  

    except Exception as e:
        node.get_logger().error(f"Costmap format error (Nav2 Costmap parsing): {e}")
        return None, 0, 0, 0.0, (0.0, 0.0)

    # occupancy values: -1 = unknown, 0 = free, 100 = occupied (typical)
    # grid: List[List[int]] = [[0 for _ in range(width)] for _ in range(height)]
    grid = [[0 for _ in range(width)] for _ in range(height)]

    for y in range(height):
        for x in range(width):
            idx = x + y * width
            v = data[idx]
            if v == -1:
                val = 1
            elif v >= 50:   # threshold for occupied
                val = 1
            else:
                val = 0
            grid[y][x] = val

    return grid, width, height, resolution, (origin_x, origin_y)


def world_to_map(world_xy: Tuple[float,float], origin: Tuple[float,float], resolution: float, width: int, height: int) -> Optional[Tuple[int,int]]:
    """
    Convert world coordinates (x,y) to map cell (mx,my).
    Returns None if outside bounds.
    """
    wx, wy = world_xy
    ox, oy = origin
    mx_f = (wx - ox) / resolution
    my_f = (wy - oy) / resolution
    mx = int(math.floor(mx_f))
    my = int(math.floor(my_f))
    if mx < 0 or my < 0 or mx >= width or my >= height:
        return None
    return mx, my


def map_to_world(cell: Tuple[int,int], origin: Tuple[float,float], resolution: float) -> Tuple[float,float]:
    """
    Convert map cell (mx,my) to world coordinates at cell center.
    """
    mx, my = cell
    ox, oy = origin
    wx = ox + (mx + 0.5) * resolution
    wy = oy + (my + 0.5) * resolution
    return wx, wy


def astar(grid: List[List[int]],
          start: Tuple[int,int],
          goal: Tuple[int,int],
          allow_diagonal: bool = True) -> Optional[List[Tuple[int,int]]]:
    """
    A* implementation on a 2D grid.
    grid[y][x] : 0 free, 1 obstacle
    start/goal are (x,y) tuples (column, row)
    Returns list of cells from start to goal inclusive as (x,y), or None if not found.
    """

    def heuristic(a: Tuple[int,int], b: Tuple[int,int]) -> float:
        (x1,y1) = a
        (x2,y2) = b
        # Euclidean
        return math.hypot(x2 - x1, y2 - y1)

    width = len(grid[0])
    height = len(grid)

    (sx, sy) = start
    (gx, gy) = goal

    # Early exit
    if start == goal:
        return [start]

    open_heap = []
    heapq.heappush(open_heap, (0.0, start))
    came_from = {}
    g_score = {start: 0.0}
    f_score = {start: heuristic(start, goal)}

    visited = set()

    # neighbor offsets
    if allow_diagonal:
        neighbors = [ (1,0), (-1,0), (0,1), (0,-1), (1,1), (1,-1), (-1,1), (-1,-1) ]
    else:
        neighbors = [ (1,0), (-1,0), (0,1), (0,-1) ]

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            # reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        (cx, cy) = current

        for dx, dy in neighbors:
            nx = cx + dx
            ny = cy + dy
            if nx < 0 or ny < 0 or nx >= width or ny >= height:
                continue
            if grid[ny][nx] == 1:
                continue

            # cost: straight = 1, diagonal = sqrt(2)
            step_cost = math.hypot(dx, dy)
            tentative_g = g_score[current] + step_cost

            neighbor = (nx, ny)
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_heap, (f, neighbor))

    return None


def interpolate_coordinates(start, end, increment=0.1):
    """
    Interpolate coordinates between two points with a fixed increment.
    This method calculates the coordinates of the points on the straight-line path that we are computing.
    
    Args:
        start (tuple): Starting coordinate (x1, y1).
        end (tuple): Ending coordinate (x2, y2).
        increment (float): Distance between interpolated points.

    Returns:
        list: List of interpolated points as (x, y) tuples.
    """
    x1, y1 = start
    x2, y2 = end

    # Calculate total distance using the Euclidean formula
    dx = x2 - x1
    dy = y2 - y1
    distance = (dx ** 2 + dy ** 2) ** 0.5

    # Calculate the number of steps
    num_steps = int(distance / increment)

    # Generate interpolated points
    points = []
    for i in range(num_steps + 1):  # +1 to include the end point
        t = i / num_steps  # Normalized step (0.0 to 1.0)
        x = x1 + t * dx  # Linear interpolation for x
        y = y1 + t * dy  # Linear interpolation for y
        points.append((x, y))

    return points


def main(args=None):
    print("Starting node astar")
    rclpy.init(args=args)
    path_planner_node = PathPlannerNode()

    try:
        rclpy.spin(path_planner_node)
    except KeyboardInterrupt:
        pass

    path_planner_node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
