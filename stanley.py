from typing import Tuple

import pygame
from pygame import Surface
from pygame import Vector2, key, K_LEFT, K_RIGHT, K_UP, K_DOWN, Color
from math import atan, radians
from ...bot import Bot
from ...linear_math import Transform


class Stanley(Bot):
    def __init__(self, track):
        super().__init__(track)

    @property
    def name(self):
        return "Stanley"

    @property
    def contributor(self):
        return "Rayman"

    @property
    def color(self):
        return Color(200, 200, 0)

    def compute_commands(self, next_waypoint: int, position: Transform, velocity: Vector2) -> Tuple:
        previous_waypoint = next_waypoint - 1 if next_waypoint > 0 else len(self.track.lines) - 1
        a = self.track.lines[previous_waypoint]
        b = self.track.lines[next_waypoint]

        # project the position onto the line ab
        projected = (position.p - a).project(b - a) + a
        self.projected = projected

        track_angle = radians((b-a).as_polar()[1])



        e = (projected - position.p).length()
        k = 1
        v = 50
        phi = position.M.angle - track_angle
        delta = phi + atan(k * e / v)
        # print(track_angle, delta, position.M.angle)

        # calculate the throttle
        target_velocity = 50
        if velocity.length() < target_velocity:
            throttle = 1
        else:
            throttle = -1

        print(throttle, delta)


        keys = key.get_pressed()
        throttle = 0
        steering_command = 0
        if keys[K_LEFT]:
            steering_command = -1
        if keys[K_RIGHT]:
            steering_command = 1
        if keys[K_UP]:
            throttle = 1
        if keys[K_DOWN]:
            throttle = -1

        return throttle, steering_command

    def draw(self, map_scaled: Surface, zoom):
        pygame.draw.circle(map_scaled, (255, 0, 0), self.projected * zoom, 10)
