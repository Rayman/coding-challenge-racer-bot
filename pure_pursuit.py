from argparse import Namespace
from typing import Tuple

import pygame
from pygame import Vector2, Color

from .utils import calculate_target_speeds, calculate_target_speed
from ...bot import Bot
from ...linear_math import Transform


class PurePursuit(Bot):
    def __init__(self, track):
        super().__init__(track)
        self.config = Namespace(
            corner_slow_down=1.2785291990662067,
            deceleration=122.35751522686678,
        )
        self.target_speeds = calculate_target_speeds(track, self.config.corner_slow_down)

    @property
    def name(self):
        return "PurePursuit"

    @property
    def contributor(self):
        return "Rayman [NC]"

    @property
    def color(self):
        return Color(200, 200, 0)

    def compute_commands(self, next_waypoint: int, position: Transform, velocity: Vector2) -> Tuple:
        target = self.track.lines[next_waypoint]
        # calculate the target in the frame of the robot
        target = position.inverse() * target
        # calculate the angle to the target
        angle = target.as_polar()[1]

        target_speed = calculate_target_speed(self.track, position, next_waypoint, self.target_speeds,
                                              self.config.deceleration)
        try:
            gamma = 2 * target.y / target.length_squared()
        except ZeroDivisionError:
            gamma = 0
        angular_velocity = gamma * velocity.length()

        if target_speed < velocity.length():
            throttle = -1
        else:
            throttle = 1

        self.position = position
        self.velocity = velocity
        self.angular_velocity = angular_velocity

        return throttle, angular_velocity

    def draw(self, map_scaled, zoom):
        target = self.position.p + self.velocity
        pygame.draw.line(map_scaled, (255, 0, 0), self.position.p * zoom, target * zoom, 2)

        # calculate circle radius based on the angular velocity
        try:
            radius = self.velocity.length() / self.angular_velocity
            if abs(radius) > 5000:
                return
            center = self.position.p + self.position.M.cols[1] * radius
            pygame.draw.circle(map_scaled, (0, 255, 0), center * zoom, abs(radius * zoom), 2)
        except (ZeroDivisionError, ValueError):
            pass
