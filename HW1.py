import argparse
import os
import numpy as np
import math
from typing import List, Tuple
from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString

def edge_angle(p1, p2):
    """Return the angle of the directed edge p1→p2."""
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])


# TODO
def get_minkowsky_sum(original_shape: Polygon, r: float) -> Polygon:
    """
    Get the polygon representing the Minkowsky sum
    :param original_shape: The original obstacle
    :param r: The radius of the rhombus
    :return: The polygon composed from the Minkowsky sums
    """

    polygon_sorted_vertices = sort_points_clockwise(original_shape.exterior.coords)
    robot_sorted_vertices = [(-r, 0), (0, r), (r, 0), (0, -r)]
    
    polygon_sorted_vertices.append(polygon_sorted_vertices[0])
    robot_sorted_vertices.append(robot_sorted_vertices[0])  

    minkowsky_points = []
    i, j = 0, 0
    while i < len(polygon_sorted_vertices) - 1 or j < len(robot_sorted_vertices) - 1:
        current_point = (polygon_sorted_vertices[i][0] + robot_sorted_vertices[j][0],
                         polygon_sorted_vertices[i][1] + robot_sorted_vertices[j][1])
        minkowsky_points.append(current_point)

        
        angle_polygon = edge_angle(polygon_sorted_vertices[i], polygon_sorted_vertices[i + 1])
        angle_robot = edge_angle(robot_sorted_vertices[j], robot_sorted_vertices[j + 1])

        if angle_polygon < angle_robot:
            i += 1
        elif angle_polygon > angle_robot:
            j += 1
        else:
            i += 1
            j += 1

    return Polygon(minkowsky_points)


def is_visible(obstacles, v1, v2):
    line = LineString([v1, v2])
    for obstacle in obstacles:
        if line.crosses(obstacle) or line.within(obstacle):
            return False
    return True


# TODO
def get_visibility_graph(obstacles: List[Polygon], source=None, dest=None) -> List[LineString]:
    """
    Get The visibility graph of a given map
    :param obstacles: A list of the C-space obstacles in the map
    :param source: The starting position of the robot. None for part 1.
    :param dest: The destination of the query. None for part 1.
    :return: A list of LineStrings holding the edges of the visibility graph
    """
    visibility_graph = []
    vertices = []

    for poly in obstacles:
        coords = list(poly.exterior.coords)
        vertices.extend(coords)

    if source is not None:
        vertices.append(tuple(source))      
    if dest is not None:
        vertices.append(tuple(dest))    

    for i in range(len(vertices)):
        for j in range(i + 1, len(vertices)):
            p = vertices[i]
            q = vertices[j]

            if is_visible(obstacles, p, q):
                visibility_graph.append(LineString([p, q]))

    # for i in range(len(obstacles)):
    #     vertices = list(obstacles[i].exterior.coords)  # Exclude the repeated last point
    #     for v in vertices:
    #         for obs in obstacles[i,:]:
    #             other_vertices = list(obs.exterior.coords)
    #             for u in other_vertices:
    #                 if v != u and is_visible(obstacles, v, u):
    #                     visibility_graph.append(LineString([v, u]))
    return visibility_graph


def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        parser.error("The file %s does not exist!" % arg)


def get_points_and_dist(line):
    source, dist = line.split(' ')
    dist = float(dist)
    source = tuple(map(float, source.split(',')))
    return source, dist


def sort_points_clockwise(points):
    center = np.mean(points, axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    return points[np.argsort(angles)]


def dijkstra_shortest_path(visibility_graph: List[LineString], source: Tuple[float, float], dest: Tuple[float, float])

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("Robot", help="A file that holds the starting position of the robot, and the distance from the center of the robot to any of its vertices")
    parser.add_argument("Obstacles", help="A file that contains the obstacles in the map")
    parser.add_argument("Query", help="A file that contains the ending position for the robot.")
    args = parser.parse_args()
    obstacles = args.Obstacles
    robot = args.Robot
    query = args.Query
    is_valid_file(parser, obstacles)
    is_valid_file(parser, robot)
    is_valid_file(parser, query)
    workspace_obstacles = []
    with open(obstacles, 'r') as f:
        for line in f.readlines():
            ob_vertices = line.split(' ')
            if ',' not in ob_vertices:
                ob_vertices = ob_vertices[:-1]
            points = [tuple(map(float, t.split(','))) for t in ob_vertices]
            workspace_obstacles.append(Polygon(points))
    with open(robot, 'r') as f:
        source, dist = get_points_and_dist(f.readline())

    # step 1:
    c_space_obstacles = [get_minkowsky_sum(p, dist) for p in workspace_obstacles]
    plotter1 = Plotter()

    plotter1.add_obstacles(workspace_obstacles)
    plotter1.add_c_space_obstacles(c_space_obstacles)
    plotter1.add_robot(source, dist)

    plotter1.show_graph()

    # step 2:

    lines = get_visibility_graph(c_space_obstacles)
    plotter2 = Plotter()

    plotter2.add_obstacles(workspace_obstacles)
    plotter2.add_c_space_obstacles(c_space_obstacles)
    plotter2.add_visibility_graph(lines)
    plotter2.add_robot(source, dist)

    plotter2.show_graph()

    # step 3:
    with open(query, 'r') as f:
        dest = tuple(map(float, f.readline().split(',')))

    lines = get_visibility_graph(c_space_obstacles, source, dest)
    #TODO: fill in the next line
    shortest_path, cost = None, None


    # step 4: Animate the shortest path
    if shortest_path:
        plotter_anim = Plotter()
        plotter_anim.add_obstacles(workspace_obstacles)
        plotter_anim.add_c_space_obstacles(c_space_obstacles)
        plotter_anim.add_visibility_graph(lines)
        plotter_anim.add_robot(source, dist)
        plotter_anim.add_robot(dest, dist)
        plotter_anim.add_shorterst_path(shortest_path)
        # plotter_anim.show_graph()
        plotter_anim.animate_path(shortest_path, dist)