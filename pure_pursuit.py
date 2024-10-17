from argparse import Namespace
from math import sqrt
from typing import Tuple

from pygame import Vector2, Color

from ...bot import Bot
from ...linear_math import Transform
from ...track import Track


def crange(start, end, modulo):
    for i in range(start, end):
        yield i % modulo


class PurePursuit(Bot):
    def __init__(self, track):
        super().__init__(track)
        self.config = Namespace(
            corner_slow_down=1.2785291990662067,
            deceleration=122.35751522686678,
        )
        self.target_speeds = []
        self.calculate_target_speeds(track)

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

        target_speed = self.calculate_target_speed(next_waypoint, position)

        try:
            gamma = 2 * target.y / target.length() ** 2
        except ZeroDivisionError:
            gamma = 0
        angular_velocity = gamma * target_speed

        if target_speed < velocity.length():
            throttle = -1
        else:
            throttle = 1

        return throttle, angular_velocity

    def calculate_target_speed(self, next_waypoint: int, position: Transform):
        min_speed = float('inf')
        waypoint_distance = 0
        # print()
        # print(f'current speed: {velocity.length():.2f}')
        for i in crange(next_waypoint, next_waypoint + 10, len(self.track.lines)):
            if i == next_waypoint:
                waypoint_distance = (self.track.lines[i] - position.p).length()
            else:
                waypoint_distance += (self.track.lines[i] - self.track.lines[i - 1]).length()
            target_speed = self.target_speeds[i]
            max_speed = sqrt(target_speed ** 2 + 2 * self.config.deceleration * waypoint_distance)
            if max_speed < min_speed:
                # print(f'{i}\t{waypoint_distance:.2f}\t{target_speed:.2f}\t{max_speed:.2f} *')
                min_speed = max_speed
            # else:
            #     print(f'{i}\t{waypoint_distance:.2f}\t{target_speed:.2f}\t{max_speed:.2f}')
        # print(f'min speed: {min_speed:.2f}')

        return min_speed
