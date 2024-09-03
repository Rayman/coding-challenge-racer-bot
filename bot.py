from copy import deepcopy
from typing import Tuple

import numpy as np
import pygame
from pygame import Vector2, Color, Surface

from ...bot import Bot
from ...car_info import CarPhysics
from ...constants import framerate
from ...linear_math import Transform
from ...track import Track


class Dustrider(Bot):
    def __init__(self, track):
        super().__init__(track)
        self.simulation = []

    @property
    def name(self):
        return "Dustrider"

    @property
    def contributor(self):
        return "Rayman"

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
        N = 50

        best_cost = float('inf')
        best_throttle = 0
        best_steering_command = 0
        for throttle in np.linspace(-1, 1, 5):
            for steering_command in np.linspace(-1, 1, 5):
                waypoint, end_position, end_velocity = self.simulate(next_waypoint, position, velocity, throttle,
                                                                     steering_command, dt, N)
                distance_to_next_waypoint = (self.track.lines[waypoint] - end_position.p).length()
                cost = -1000 * ((waypoint - next_waypoint) % len(self.track.lines)) + distance_to_next_waypoint
                if cost < best_cost:
                    # print(
                    #     f'Better\tcost={cost:.3f} throttle={throttle} steering={steering_command} waypoint={waypoint} distance={distance_to_next_waypoint}')
                    best_cost = cost
                    best_throttle = throttle
                    best_steering_command = steering_command
                # else:
                # print(
                #     f'\tcost={cost:.3f} throttle={throttle} steering={steering_command} waypoint={waypoint} distance={distance_to_next_waypoint}')

        # Simulate the best throttle and steering command
        car = CarSimulator(self.track, next_waypoint, deepcopy(position), deepcopy(velocity))
        self.simulation = []
        self.simulation.append(deepcopy(car.car_physics.position))
        for i in range(N):
            car.update(0, dt, best_throttle, best_steering_command)
            self.simulation.append(deepcopy(car.car_physics.position))

        # Print simulation
        # print(f'Position: {position.p}')
        # print('\n')
        return best_throttle, best_steering_command

    def simulate(self, next_waypoint: int, position: Transform, velocity: Vector2, throttle, steering_command, dt, N):
        car = CarSimulator(self.track, next_waypoint, deepcopy(position), deepcopy(velocity))
        for i in range(N):
            car.update(0, dt, throttle, steering_command)
        return car.next_waypoint, car.car_physics.position, car.car_physics.velocity

    def draw(self, map_scaled: Surface, zoom):
        # Draw the simulation on the scaled map
        # print(f'Simulation: {[p.p for p in self.simulation]}')
        pygame.draw.lines(map_scaled, (0, 0, 0), False, [zoom * p.p for p in self.simulation], 2)


class CarSimulator:
    def __init__(self, track: Track, next_waypoint: int, position: Transform, velocity: Vector2):
        self.track = track
        self.next_waypoint = next_waypoint
        self.car_physics = CarPhysics(position, velocity)

    def update(self, time: float, dt: float, throttle: float, steering_command: float):
        self.car_physics.update(time, dt, throttle, steering_command)

        # Update next waypoint
        if (self.track.lines[self.next_waypoint] - self.car_physics.position.p).length() < self.track.track_width:
            self.next_waypoint = self.next_waypoint + 1
            if self.next_waypoint >= len(self.track.lines):
                self.next_waypoint = 0
