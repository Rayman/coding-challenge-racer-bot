from argparse import Namespace
from math import sqrt, atan2, fmod, pi
from typing import Tuple

from pygame import Vector2, Color

from ...bot import Bot
from ...linear_math import Transform
from ...track import Track


def crange(start, end, modulo):
    for i in range(start, end):
        yield i % modulo


def normalize_angle(angle):
    result = fmod(angle + pi, 2.0 * pi)
    if result <= 0.0:
        return result + pi
    return result - pi


class VectorPursuit(Bot):
    def __init__(self, track):
        super().__init__(track)
        self.config = Namespace(
            corner_slow_down=30,
            deceleration=122.35751522686678,
            k=2,
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
        return "VectorPursuit"

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
        target_angle = 0
        distance = target.length()

        try:
            phi_1 = atan2(
                (2 * pow(target.y, 2) - pow(distance, 2)),
                (2 * target.x * target.y)
            )
            phi_2 = atan2(pow(distance, 2), (2 * target.y))
            phi = normalize_angle(phi_1 - phi_2)

            term_1 = (self.config.k * phi) / (((self.config.k - 1) * phi) + target_angle)
            term_2 = pow(distance, 2) / (2 * target.y)

            turning_radius = abs(term_1 * term_2)
            print(
                f'phi_1={phi_1:.2f}, phi_2={phi_2:.2f}, phi={phi:.2f}, term_1={term_1:.2f}, term_2={term_2:.2f}, turning_radius={turning_radius:.2f}')
        except ZeroDivisionError:
            turning_radius = float('inf')

        target_speed = 100  # self.calculate_target_speed(next_waypoint, position)
        angular_velocity = target_speed / turning_radius

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
