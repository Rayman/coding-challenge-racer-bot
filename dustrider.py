import json
from argparse import Namespace
from copy import deepcopy
from math import fmod, pi, sqrt
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
            corner_slow_down=1.3474480201529782,
            deceleration=122.01115713324654,

            w_waypoint=8797.335306281711,
            w_speed=3.496329938262048,
            n=25,
        )
        self.target_speeds = []
        self.calculate_target_speeds(track)
        self.simulation = []
        self.font = pygame.font.SysFont('', 20)
        if DEBUG:
            self.sock = socket(AF_INET, SOCK_DGRAM)
            self.server_address = ('127.0.0.1', 12389)

    def calculate_target_speeds(self, track: Track):
        self.target_speeds = []
        for i in range(len(track.lines)):
            p0 = self.track.lines[(i - 1) % len(self.track.lines)]
            p1 = self.track.lines[i]
            p2 = self.track.lines[(i + 1) % len(self.track.lines)]
            a = (p2 - p1).length()
            b = (p0 - p2).length()
            c = (p0 - p1).length()
            area = 0.5 * abs(p0.x * (p1.y - p2.y) + p1.x * (p2.y - p0.y) + p2.x * (p0.y - p1.y))
            R = a * b * c / (4 * area)

            target_speed = self.config.corner_slow_down * R
            self.target_speeds.append(target_speed)

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
        dt = 1 / framerate

        # print()
        best_cost = float('inf')
        best_throttle = 0
        best_steering_command = 0
        for throttle in np.linspace(-1, 1, 3):
            for steering_command in np.linspace(-1, 1, 5):
                waypoint, end_position, end_velocity = self.simulate(next_waypoint, position, velocity, throttle,
                                                                     steering_command, dt, self.config.n)
                waypoint_plus_one = (waypoint + 1) % len(self.track.lines)

                # cost 2
                distance_to_next_waypoint = (self.track.lines[waypoint] - end_position.p).length()

                # cost 3
                target_speed_at_waypoint = self.target_speeds[waypoint_plus_one]
                target_speed = sqrt(
                    target_speed_at_waypoint ** 2 + 2 * self.config.deceleration * distance_to_next_waypoint)
                velocity_diff = max(0., end_velocity.length() - target_speed) * self.config.w_speed

                # total cost
                cost = -self.config.w_waypoint * ((waypoint - next_waypoint) % len(
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
        for i in range(self.config.n):
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

        # Draw the target speeds
        for i, target_speed in enumerate(self.target_speeds):
            text = self.font.render(f'{target_speed:.2f}', True, (0, 0, 0))
            map_scaled.blit(text, (self.track.lines[i].x * zoom, self.track.lines[i].y * zoom))


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
