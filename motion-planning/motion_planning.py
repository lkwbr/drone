#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''Drone planning solution within 3D space.
'''

__author__ = 'Udacity, Luke Weber'
__email__ = 'lukedottec@gmail.com'

import argparse
import time
import msgpack
import re
from enum import Enum, auto

import numpy as np
from src.planning import create_grid, euclid_dist, a_star_grid, prune
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()

class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # Starting state
        self.flight_state = States.MANUAL

        # Define callbacks for position, velocity, and internal state changes
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print('arming transition')
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print('takeoff transition')
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print('waypoint transition')
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print('landing transition')
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print('disarm transition')
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print('manual transition')
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print('Sending waypoints to simulator ...')
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        '''Return approximately optimal waypoints to arrive from start to goal.'''
        self.flight_state = States.PLANNING
        print('Searching for a path ...')
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        obstacles = 'resource/colliders.csv'

        # Read lat0, lon0 from colliders into floating point values
        with open(obstacles) as f:
            line = f.readline()
            match = re.match(r'^lat0 (.*), lon0 (.*)$', line)
            lat = match.group(1)
            lon = match.group(2)
        lat0, lon0 = (float(lat), float(lon))

        # Set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # Retrieve current local position
        local_north, local_east, local_down = global_to_local(self.global_position, self.global_home)
        print(f'Local => north : {local_north}, east : {local_east}, down : {local_down}')
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position, self.local_position))

        # Read in obstacle map; we trust that this data accurately describes the environment
        data = np.loadtxt(obstacles, delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print('North offset = {0}, east offset = {1}'.format(north_offset, east_offset))

        # Define starting point on the grid (this is just grid center)
        grid_start_north = int(np.ceil(local_north - north_offset))
        grid_start_east = int(np.ceil(local_east - east_offset))
        grid_start = (-north_offset, -east_offset)

        # Set goal as some arbitrary position on the grid
        goal_global_position = [-122.401912, 37.794409, self.global_home[2]]
        goal_north, goal_east, goal_alt = global_to_local(goal_global_position, self.global_home)
        grid_goal = (int(goal_north - north_offset), int(goal_east - east_offset))

        # Run A* from start- to end-cell
        path, cost = a_star_grid(grid, euclid_dist, grid_start, grid_goal)

        # Prune path
        path = prune(path)

        # Convert path to waypoints
        self.waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in path]

        # Send waypoints to simulator, just for visualization
        self.send_waypoints()

    def start(self):
        self.start_log('Logs', 'NavLog.txt')

        print('starting connection')
        self.connection.start()
        self.stop_log()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help='host address, i.e. "127.0.0.1"')
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)

    time.sleep(1)
    drone.start()
    