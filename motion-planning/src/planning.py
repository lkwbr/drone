#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''Planning resources for drone navigation.
'''

from enum import Enum
from queue import PriorityQueue
from math import sqrt

import numpy as np

from src.sampling import Sampler
from sklearn.neighbors import KDTree

def can_connect(a, b, polygons):
    '''Determine if two nodes `a` and `b` can be connected, given obstacles as
    `polygons`.
    '''
    l = LineString([a, b])
    for p in polygons:
        if p.crosses(l) and p.height >= min(a[2], b[2]):
            return False
    return True

def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)

def create_graph(data, nodecount=50, k=10):
    '''Generate graph for Probabilistic Roadmap (PRM) approach, with local clustering of
    nodes within this 3D environment, `data`.
    '''
    # Randomly sample graph vertices/nodes
    sampler = Sampler(data)
    polygons = sampler._polygons
    nodes = sampler.sample(300)
    # Create graph with each node connected to it's `k` nearest neighbors
    g = nx.Graph()
    tree = KDTree(nodes)
    for a in nodes:
        # Grab `k` neighbors, ommiting first element because that's `a` (`a` is closest to `a`)
        idxs = tree.query([a], k + 1, return_distance=False)[0][1:]
        for i in idxs:
            b = nodes[i]
            if can_connect(a, b, polygons):
                g.add_edge(a, b, weight=1)
    return g

class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTH_WEST = (1, -1, sqrt(2))
    NORTH_EAST = (-1, 1, sqrt(2))
    SOUTH_WEST = (-1, -1, sqrt(2))
    SOUTH_EAST = (1, 1, sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])

def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    # NORTH, EAST, SOUTH, WEST
    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > n or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    # DIAGONALS
    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if x + 1 > n or y + 1 > n or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)

    return valid_actions

def a_star_grid(grid, h, start, goal):
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost

def a_star_graph(graph, h, start, goal):
    '''Run A* algorithm through graph from start to goal, return path and cost.'''
    q = PriorityQueue()
    q.put(start)
    branch = { start: (None, 0) }
    while not q.empty():
        node = q.get()
        cost = branch[node][1]
        if node == goal:
            break
        for next_node in node.edges():
            if next_node not in branch:
                # Determine cost of this next node as sum between: cost of being at
                # `node`, cost from `node` to `next_node`, and cost of `next_node`
                # to `goal`
                next_cost = cost + h(node, next_node) + h(next_node, goal)
                branch[next_node] = (node, next_cost)
                q.put(next_node)
    path = []
    cost = 0
    if goal in branch:
        print('Path to goal found!')
        # Backtrack path from goal to start
        branch_node = goal
        while branch_node is not None:
            path.append(branch_node)

    else:
        print('No path found')
    return path, cost

def euclid_dist(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

def prune(path, epsilon=0.1):
    '''Remove redundant waypoints from path.'''
    pruned = [p for p in path]
    i = 0
    while i < len(pruned) - 2:
        p1 = pruned[i]
        p2 = pruned[i+1]
        p3 = pruned[i+2]
        det = p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1])
        if abs(det) <= epsilon:
            pruned.remove(pruned[i+1])
        else:
            i += 1
    return pruned
