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


class SplineBot(Bot):
    def __init__(self, track):
        super().__init__(track)
        self.config = Namespace(
            corner_slow_down=1.2785291990662067,
            deceleration=122.35751522686678,
            alpha=1.0,
            lookahead=50.0,
        )
        self.target_speeds = calculate_target_speeds(track, self.config.corner_slow_down)

        self.splines = []
        for i in range(len(track.lines)):
            p0 = track.lines[(i - 1) % len(track.lines)]
            p1 = track.lines[i]
            p2 = track.lines[(i + 1) % len(track.lines)]
            p3 = track.lines[(i + 2) % len(track.lines)]
            self.splines.append(CatmullRomSpline(p0, p1, p2, p3, self.config.alpha))

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
        def distance(t):
            p = Vector2(*self.splines[(next_waypoint - 1) % len(self.splines)].progress(t))
            return (p - position.p).length()

        closest_t = min(np.linspace(0, 1, 10), key=distance)
        closest = Vector2(*self.splines[(next_waypoint - 1) % len(self.splines)].progress(closest_t))

        lookahead_point = self.find_lookahead(next_waypoint, closest, closest_t)
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
        self.closest = closest
        self.lookahead_point = lookahead_point

        return throttle, angular_velocity

    def find_lookahead(self, next_waypoint, closest, closest_t):
        for i in crange(next_waypoint, next_waypoint + len(self.splines), len(self.splines)):
            spline = self.splines[i - 1]
            # look for the first point further away than the lookahead distance
            if i == next_waypoint:
                start_t = closest_t
            else:
                start_t = 0

            for t in np.linspace(start_t, 1, 100):
                lookahead_point = spline.progress(t)
                lookahead_point = Vector2(*lookahead_point)
                if (lookahead_point - closest).length() > self.config.lookahead:
                    return lookahead_point
            else:
                continue
        raise NotImplementedError("No lookahead point found")

    def draw(self, map_scaled, zoom):
        for spline in self.splines:
            points = []
            for t in np.linspace(0, 1, 10):
                points.append(spline.progress(t))
            points = np.array(points)
            pygame.draw.lines(map_scaled, (0, 0, 200), False, points * zoom, 2)

        pygame.draw.circle(map_scaled, (200, 0, 0), self.closest * zoom, 5)
        pygame.draw.circle(map_scaled, (0, 200, 0), self.lookahead_point * zoom, 5)
