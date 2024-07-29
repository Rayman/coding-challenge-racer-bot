from copy import deepcopy
from typing import Tuple

import numpy as np
import pygame
from pygame import Vector2, Color, Surface

from ...bot import Bot
from ...car_info import CarInfo
from ...linear_math import Transform


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

        dt = 1 / 60
        N = 50

        best_cost = float('inf')
        best_throttle = 0
        best_steering_command = 0
        for throttle in np.linspace(-1, 1, 5):
            for steering_command in np.linspace(-1, 1, 5):
                waypoint, end_position, end_velocity = self.simulate(next_waypoint, position, velocity, throttle,
                                                                     steering_command, dt, N)
                distance_to_next_waypoint = (self.track.lines[waypoint] - end_position.p).length()
                cost = -1000 * waypoint + distance_to_next_waypoint
                if cost < best_cost:
                    print(
                        f'Better\tcost={cost:.3f} throttle={throttle} steering={steering_command} waypoint={waypoint} distance={distance_to_next_waypoint}')
                    best_cost = cost
                    best_throttle = throttle
                    best_steering_command = steering_command
                else:
                    print(
                        f'\tcost={cost:.3f} throttle={throttle} steering={steering_command} waypoint={waypoint} distance={distance_to_next_waypoint}')

        # Simulate the best throttle and steering command
        car = CarInfo(None, self.track)
        car.position = deepcopy(position)
        car.velocity = deepcopy(velocity)
        car.next_waypoint = next_waypoint
        self.simulation = []
        self.simulation.append(deepcopy(car.position))
        for i in range(N):
            car.update(dt, best_throttle, best_steering_command)
            self.simulation.append(deepcopy(car.position))

        # Print simulation
        print(f'Position: {position.p}')
        print('\n')
        return best_throttle, best_steering_command

    def simulate(self, next_waypoint: int, position: Transform, velocity: Vector2, throttle, steering_command, dt, N):
        car = CarInfo(None, self.track)
        car.position = deepcopy(position)
        car.velocity = deepcopy(velocity)
        car.next_waypoint = next_waypoint
        for i in range(N):
            car.update(dt, throttle, steering_command)
        return car.next_waypoint, car.position, car.velocity

    def draw(self, map_scaled: Surface, zoom):
        # Draw the simulation on the scaled map
        print(f'Simulation: {[p.p for p in self.simulation]}')
        pygame.draw.lines(map_scaled, (0, 0, 0), False, [zoom * p.p for p in self.simulation], 2)
