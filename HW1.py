import argparse
import os
import numpy as np
import math
from typing import List, Tuple
from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString


def edge_angle(p1, p2):
    """Return the angle of the directed edge p1→p2."""
    # return math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    angle = math.atan2(dy, dx)
    if angle < 0:
        angle += 2 * math.pi
    return angle


def get_minkowsky_sum(original_shape: Polygon, r: float) -> Polygon:
    """
    Get the polygon representing the Minkowsky sum
    :param original_shape: The original obstacle
    :param r: The radius of the rhombus
    :return: The polygon composed from the Minkowsky sums
    """

    polygon_sorted_vertices = sort_points_clockwise(original_shape.exterior.coords)
    robot_sorted_vertices = [(0, -r), (r, 0), (0, r), (-r, 0)]

    poly_len = len(polygon_sorted_vertices)
    robot_len = len(robot_sorted_vertices)

    polygon_sorted_vertices.append(polygon_sorted_vertices[0])
    # polygon_sorted_vertices.append(polygon_sorted_vertices[1])
    robot_sorted_vertices.append(robot_sorted_vertices[0])
    # robot_sorted_vertices.append(robot_sorted_vertices[1])

    minkowsky_points = []

    i, j = 0, 0
    while i < poly_len and j < robot_len:
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

    while i < poly_len:
        p = polygon_sorted_vertices[i]
        r_pt = robot_sorted_vertices[-1]
        minkowsky_points.append((p[0] + r_pt[0], p[1] + r_pt[1]))
        i += 1

    while j < robot_len:
        p = polygon_sorted_vertices[-1]
        r_pt = robot_sorted_vertices[j]
        minkowsky_points.append((p[0] + r_pt[0], p[1] + r_pt[1]))
        j += 1

    return Polygon(minkowsky_points)


def is_visible(obstacles, v1, v2):
    line = LineString([v1, v2])
    for obstacle in obstacles:
        if line.crosses(obstacle) or line.within(obstacle):
            return False
    return True


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
    p = np.array(points)
    center = np.mean(p, axis=0)
    angles = np.arctan2(p[:, 1] - center[1], p[:, 0] - center[0])
    sorted_idx = np.argsort(angles)
    sorted_points = p[sorted_idx]
    lowest_idx = np.lexsort((sorted_points[:, 0], sorted_points[:, 1]))[0]
    reordered = np.concatenate((sorted_points[lowest_idx:], sorted_points[:lowest_idx]), axis=0)

    return [tuple(pt) for pt in reordered]


def get_graph(visibility_graph: List[LineString]):
    from math import dist
    graph = {}
    for edge in visibility_graph:
        v1 = edge.coords[0]
        v2 = edge.coords[-1]
        weight = dist(v1, v2)
        if v1 not in graph:
            graph[v1] = {}
        if v2 not in graph:
            graph[v2] = {}
        graph[v1][v2] = weight
        graph[v2][v1] = weight
    return graph


def dijkstra_shortest_path(visibility_graph: List[LineString], source: Tuple[float, float], dest: Tuple[float, float]):
    import heapq

    graph = get_graph(visibility_graph)

    queue = []
    heapq.heapify(queue)
    heapq.heappush(queue, (0.0, source))
    dist_to = {source: 0.0}
    parent = {source: None}

    while queue:
        curr_dist, u = heapq.heappop(queue)
        if u == dest:
            break
        if curr_dist > dist_to[u]:
            continue
        for adj, weight in graph[u].items():
            new_dist = curr_dist + weight
            if adj not in dist_to or new_dist < dist_to[adj]:
                dist_to[adj] = new_dist
                parent[adj] = u
                heapq.heappush(queue, (new_dist, adj))

    if dest not in parent:
        return None, math.inf

    path = []
    node = dest
    while node is not None:
        path.append(node)
        node = parent[node]
    path.reverse()

    return path, dist_to[dest]


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
    shortest_path, cost = dijkstra_shortest_path(lines, source, dest)


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