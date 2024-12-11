from argparse import Namespace
from typing import Tuple

import numpy as np
import pygame
from pygame import Vector2, Color

from .utils import calculate_target_speeds, crange, calculate_target_speed
from ...bot import Bot
from ...linear_math import Transform


class CatmullRomSpline:
    def __init__(self, p0, p1, p2, p3, alpha: float = 0.5):
        self.p0 = p0
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.alpha = alpha

        self.t0 = 0
        self.t1 = self.get_t(self.t0, p0, p1)
        self.t2 = self.get_t(self.t1, p1, p2)
        self.t3 = self.get_t(self.t2, p2, p3)

    def get_t(self, ti, pi, pj):
        xi, yi = pi
        xj, yj = pj
        dx, dy = xj - xi, yj - yi
        l = (dx ** 2 + dy ** 2) ** 0.5
        return ti + l ** self.alpha

    def progress(self, t):
        t = t * (self.t2 - self.t1) + self.t1
        a1 = (self.t1 - t) / (self.t1 - self.t0) * self.p0 + (t - self.t0) / (self.t1 - self.t0) * self.p1
        a2 = (self.t2 - t) / (self.t2 - self.t1) * self.p1 + (t - self.t1) / (self.t2 - self.t1) * self.p2
        a3 = (self.t3 - t) / (self.t3 - self.t2) * self.p2 + (t - self.t2) / (self.t3 - self.t2) * self.p3
        b1 = (self.t2 - t) / (self.t2 - self.t0) * a1 + (t - self.t0) / (self.t2 - self.t0) * a2
        b2 = (self.t3 - t) / (self.t3 - self.t1) * a2 + (t - self.t1) / (self.t3 - self.t1) * a3

        return (self.t2 - t) / (self.t2 - self.t1) * b1 + (t - self.t1) / (self.t2 - self.t1) * b2


class SplineBot2(Bot):
    def __init__(self, track):
        super().__init__(track)
        self.config = Namespace(
            corner_slow_down=1.327142934320906,
            deceleration=138.24680851477612,
            alpha=0.5320820464438623,
            lookahead=41.354129918761494,
            min_segment_length=20.0
        )
        self.init()

    def init(self):
        self.target_speeds = calculate_target_speeds(self.track, self.config.corner_slow_down)

        splines = []
        for i in range(len(self.track.lines)):
            p0 = self.track.lines[(i - 1) % len(self.track.lines)]
            p1 = self.track.lines[i]
            p2 = self.track.lines[(i + 1) % len(self.track.lines)]
            p3 = self.track.lines[(i + 2) % len(self.track.lines)]
            splines.append(CatmullRomSpline(p0, p1, p2, p3, self.config.alpha))

        # interpolate each spline
        self.points = []
        for spline in splines:
            for n in (2 ** i for i in range(2, 10)):
                points = [spline.progress(t) for t in np.linspace(0, 1, n)]
                points = [Vector2(*p) for p in points]
                longest_segment = max((points[i] - points[i - 1]).length() for i in range(1, len(points)))
                if longest_segment < self.config.min_segment_length:
                    break
            self.points.append(points)

    @property
    def name(self):
        return "SplineBot"

    @property
    def contributor(self):
        return "Rayman [NC]"

    @property
    def color(self):
        return Color(200, 200, 0)

    def compute_commands(self, next_waypoint: int, position: Transform, velocity: Vector2) -> Tuple:
        # first search for the closest point on the spline to the car
        points = self.points[(next_waypoint - 1) % len(self.points)]
        closest = min(range(len(points)), key=lambda i: (points[i] - position.p).length())

        i, j = self.find_lookahead(next_waypoint, closest)
        lookahead_point = self.points[i][j]
        target = position.inverse() * lookahead_point
        try:
            gamma = 2 * target.y / target.length_squared()
        except ZeroDivisionError:
            gamma = 0
        angular_velocity = gamma * velocity.length()

        target_speed = calculate_target_speed(self.track, position, next_waypoint, self.target_speeds,
                                              self.config.deceleration)
        if target_speed < velocity.length():
            throttle = -1
        else:
            throttle = 1

        # debug drawing
        self.closest = points[closest]
        self.lookahead = self.points[i][j]

        return throttle, angular_velocity

    def find_lookahead(self, next_waypoint: int, closest: int) -> Tuple[int, int]:
        closest_point = self.points[(next_waypoint - 1) % len(self.points)][closest]
        for i in crange(next_waypoint - 1, next_waypoint - 1 + len(self.track.lines), len(self.track.lines)):
            if i == (next_waypoint - 1) % len(self.track.lines):
                start_index = closest
            else:
                start_index = 0

            for j in range(start_index, len(self.points[i])):
                distance = (self.points[i][j] - closest_point).length()
                if distance > self.config.lookahead:
                    return i, j

        raise RuntimeError('Could not find lookahead')

    def draw(self, map_scaled, zoom):
        for segment in self.points:
            for p in segment:
                pygame.draw.circle(map_scaled, (0, 0, 0), p * zoom, 2)

        pygame.draw.circle(map_scaled, (200, 0, 0), self.closest * zoom, 5)
        pygame.draw.circle(map_scaled, (0, 200, 0), self.lookahead * zoom, 5)
