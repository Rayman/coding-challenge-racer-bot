from argparse import Namespace
from math import sqrt
from typing import Tuple, Iterable

import numpy as np
import pygame
from pygame import Vector2, Color

from .utils import calculate_radius
from ...bot import Bot
from ...linear_math import Transform

DEBUG = False


def crange(start, end, length) -> Iterable[int]:
    if start < end:
        yield from range(start, end)
    else:
        yield from range(start, length)
        yield from range(0, end)


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


class RoadSprinter(Bot):
    def __init__(self, track):
        super().__init__(track)
        self.config = Namespace(
            deceleration=100.0,
            corner_slow_down=2.0,
            alpha=1.0,
            lookahead=55.0,
            speed_lookahead=100,
            min_segment_length=20.0
        )
        self.points = []
        self.spline_starts = []
        self.target_speeds = []
        self.init()

    def init(self):
        splines = []
        for i in range(len(self.track.lines)):
            p0 = self.track.lines[(i - 1) % len(self.track.lines)]
            p1 = self.track.lines[i]
            p2 = self.track.lines[(i + 1) % len(self.track.lines)]
            p3 = self.track.lines[(i + 2) % len(self.track.lines)]
            splines.append(CatmullRomSpline(p0, p1, p2, p3, self.config.alpha))

        # interpolate each spline
        self.points = []
        self.spline_starts = []
        for spline in splines:
            for n in (2 ** i for i in range(2, 10)):
                points = [spline.progress(t) for t in np.linspace(0, 1, n, endpoint=False)]
                points = [Vector2(*p) for p in points]
                longest_segment = max((points[i] - points[i - 1]).length() for i in range(1, len(points)))
                if longest_segment < self.config.min_segment_length:
                    break
            self.spline_starts.append(len(self.points))
            self.points.extend(points)

        for i in range(len(self.points)):
            p0 = self.points[(i - 1) % len(self.points)]
            p1 = self.points[i]
            p2 = self.points[(i + 1) % len(self.points)]
            R = calculate_radius(p0, p1, p2)
            target_speed = self.config.corner_slow_down * R
            self.target_speeds.append(target_speed)

    @property
    def name(self):
        return "Road Sprinter 2"

    @property
    def contributor(self):
        return "Rayman [NC]"

    @property
    def color(self):
        return Color(200, 0, 0)

    def compute_commands(self, next_waypoint: int, position: Transform, velocity: Vector2) -> Tuple:
        # first search for the closest point on the spline to the car
        search_start = self.spline_starts[(next_waypoint - 1) % len(self.track.lines)]
        search_end = self.spline_starts[next_waypoint]

        closest = min(crange(search_start, search_end, len(self.points)),
                      key=lambda i: (self.points[i] - position.p).length())

        i = self.find_lookahead(position, closest)
        lookahead_point = self.points[i]
        target = position.inverse() * lookahead_point
        try:
            gamma = 2 * target.y / target.length_squared()
        except ZeroDivisionError:
            gamma = 0
        angular_velocity = gamma * velocity.length()

        # print()
        # print(f'{"i":>3} {"dist":>7} {"target":>7} {"max":>8}')
        distance = 0
        min_speed = float('inf')
        corner_index = 0
        for i in crange(closest, (closest + 100) % len(self.points), len(self.points)):
            distance += (self.points[i] - self.points[(i - 1) % len(self.points)]).length()
            if distance < self.config.speed_lookahead:
                continue
            target_speed = self.target_speeds[i]
            max_speed = sqrt(target_speed ** 2 + 2 * self.config.deceleration * distance)
            if max_speed < min_speed:
                # print(f'{i:3} {distance:7.2f} {target_speed:7.2f} {max_speed:8.2f} *')
                min_speed = max_speed
                corner_index = i
            # else:
            # print(f'{i:3} {distance:7.2f} {target_speed:7.2f} {max_speed:8.2f}')

        target_speed = min_speed

        if target_speed < velocity.length():
            throttle = -1
        else:
            throttle = 1

        # debug drawing
        self.closest = self.points[closest]
        self.lookahead = lookahead_point
        self.corner = self.points[corner_index]

        return throttle, 3 * angular_velocity

    def find_lookahead(self, position: Transform, closest: int) -> int:
        for i in crange(closest, (closest + len(self.points)) % len(self.points), len(self.points)):
            distance = (self.points[i] - position.p).length()
            if distance > self.config.lookahead:
                return i

        raise RuntimeError('Could not find lookahead')

    def draw(self, map_scaled, zoom):
        if not DEBUG:
            return

        for i in range(len(self.points)):
            p = self.points[i]
            target_speed = self.target_speeds[i]
            target_speed = target_speed / 5 / self.config.corner_slow_down
            color = (0 if target_speed > 255 else 255 - target_speed, 0, 255 if target_speed > 255 else target_speed)
            pygame.draw.circle(map_scaled, color, p * zoom, 2)

        pygame.draw.circle(map_scaled, (200, 0, 0), self.closest * zoom, 5)
        pygame.draw.circle(map_scaled, (0, 200, 0), self.lookahead * zoom, 5)
        pygame.draw.circle(map_scaled, (0, 0, 200), self.corner * zoom, 5)
