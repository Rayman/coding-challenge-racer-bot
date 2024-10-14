import json
from argparse import Namespace
from copy import deepcopy
from math import fmod, pi, radians
from socket import socket, AF_INET, SOCK_DGRAM
from typing import Tuple

import numpy as np
import pygame
from pygame import Vector2, Color, Surface

from ...bot import Bot
from ...car_info import CarPhysics
from ...constants import framerate
from ...linear_math import Transform
from ...track import Track

DEBUG = False


def normalize_angle(angle):
    result = fmod(angle + pi, 2.0 * pi)
    if result <= 0.0:
        return result + pi
    return result - pi


class Dustrider(Bot):
    def __init__(self, track):
        super().__init__(track)
        self.config = Namespace(
            corner_velocity=150,
            corner_slow_down=2,
        )
        self.target_speeds = []
        for line in track.lines:
            corner_angle = abs(
                normalize_angle(radians(line.angle_to(track.lines[(track.lines.index(line) + 1) % len(track.lines)]))))
            target_speed = self.config.corner_velocity * (1 - self.config.corner_slow_down * corner_angle / pi)
            self.target_speeds.append(target_speed)

        self.simulation = []
        if DEBUG:
            self.sock = socket(AF_INET, SOCK_DGRAM)
            self.server_address = ('127.0.0.1', 12389)

    @property
    def name(self):
        return "Dustrider"

    @property
    def contributor(self):
        return "Rayman [NC]"

    @property
    def color(self):
        return Color(200, 200, 0)

    def compute_commands(self, next_waypoint: int, position: Transform, velocity: Vector2) -> Tuple:
        # target = self.track.lines[next_waypoint]
        # # calculate the target in the frame of the robot
        # target = position.inverse() * target
        # # calculate the angle to the target
        # angle = target.as_polar()[1]
        #
        # # calculate the throttle and steering
        # if abs(angle) < 0.1:
        #     throttle, steering_command = 1, 0
        # else:
        #     if angle > 0:
        #         throttle, steering_command = 0.1, 1
        #     else:
        #         throttle, steering_command = 0.1, -1

        dt = 1 / framerate
        N = 10

        # print()
        best_cost = float('inf')
        best_throttle = 0
        best_steering_command = 0
        for throttle in np.linspace(-1, 1, 3):
            for steering_command in np.linspace(-1, 1, 5):
                waypoint, end_position, end_velocity = self.simulate(next_waypoint, position, velocity, throttle,
                                                                     steering_command, dt, N)
                waypoint_plus_one = (waypoint + 1) % len(self.track.lines)

                # cost 2
                distance_to_next_waypoint = (self.track.lines[waypoint] - end_position.p).length()

                # cost 3
                target_speed = self.target_speeds[waypoint_plus_one]
                velocity_diff = (target_speed - end_velocity.length()) ** 2 / 1000

                # total cost
                cost = -1000 * ((waypoint - next_waypoint) % len(
                    self.track.lines)) + distance_to_next_waypoint + velocity_diff
                if cost < best_cost:
                    # print(f'Better\tcost={cost:.3f} throttle={throttle} steering={steering_command} waypoint={waypoint} distance={distance_to_next_waypoint:.3f} speed={end_velocity.length():.3f} target_speed={target_speed:.3f} velocity_diff={velocity_diff:.3f}')
                    best_cost = cost
                    best_throttle = throttle
                    best_steering_command = steering_command
                # else:
                # print(f'\tcost={cost:.3f} throttle={throttle} steering={steering_command} waypoint={waypoint} distance={distance_to_next_waypoint:.3f} speed={end_velocity.length():.3f} target_speed={target_speed:.3f} velocity_diff={velocity_diff:.3f}')

        if DEBUG:
            data = {
                'target_speed': target_speed,
            }
            self.sock.sendto(json.dumps(data).encode('utf-8'), self.server_address)

        # Simulate the best throttle and steering command
        car = CarSimulator(self.track, next_waypoint, deepcopy(position), deepcopy(velocity))
        self.simulation = []
        self.simulation.append(deepcopy(car.car_physics.position))
        for i in range(N):
            car.update(dt, best_throttle, best_steering_command)
            self.simulation.append(deepcopy(car.car_physics.position))

        # Print simulation
        # print(f'Position: {position.p}')
        # print('\n')
        return best_throttle, best_steering_command

    def simulate(self, next_waypoint: int, position: Transform, velocity: Vector2, throttle, steering_command, dt, N):
        car = CarSimulator(self.track, next_waypoint, deepcopy(position), deepcopy(velocity))
        for i in range(N):
            car.update(dt, throttle, steering_command)
        return car.next_waypoint, car.car_physics.position, car.car_physics.velocity

    def draw(self, map_scaled: Surface, zoom):
        # Draw the simulation on the scaled map
        # print(f'Simulation: {[p.p for p in self.simulation]}')
        if self.simulation:
            pygame.draw.lines(map_scaled, (0, 0, 0), False, [zoom * p.p for p in self.simulation], 2)


class CarSimulator:
    def __init__(self, track: Track, next_waypoint: int, position: Transform, velocity: Vector2):
        self.track = track
        self.next_waypoint = next_waypoint
        self.car_physics = CarPhysics(position, velocity)

    def update(self, dt: float, throttle: float, steering_command: float):
        self.car_physics.update(dt, throttle, steering_command)

        # Update next waypoint
        if (self.track.lines[self.next_waypoint] - self.car_physics.position.p).length() < self.track.track_width:
            self.next_waypoint = self.next_waypoint + 1
            if self.next_waypoint >= len(self.track.lines):
                self.next_waypoint = 0
