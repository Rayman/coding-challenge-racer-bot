import json
from argparse import Namespace
from math import sqrt, fmod, pi, radians
from socket import socket, AF_INET, SOCK_DGRAM
from typing import Tuple

from pygame import Vector2, Color, Surface, font

from ...bot import Bot
from ...linear_math import Transform
from ...track import Track

DEBUG = False


def normalize_angle(angle):
    result = fmod(angle + pi, 2.0 * pi)
    if result <= 0.0:
        return result + pi
    return result - pi


def crange(start, end, modulo):
    for i in range(start, end):
        yield i % modulo


class RoadRunner(Bot):
    def __init__(self, track: Track):
        super().__init__(track)
        self.config = Namespace(
            corner_velocity=400.8158473899843,
            corner_slow_down=2.304897045426168,
            deceleration=114.31474884582616
        )
        self.target_speeds = []
        self.calculate_target_speeds(track)
        self.font = font.SysFont('', 20)
        if DEBUG:
            self.sock = socket(AF_INET, SOCK_DGRAM)
            self.server_address = ('127.0.0.1', 12389)

    def calculate_target_speeds(self, track: Track):
        self.target_speeds = []
        for i in range(len(track.lines)):
            previous = track.lines[i] - track.lines[(i - 1) % len(track.lines)]
            next = track.lines[(i + 1) % len(track.lines)] - track.lines[i]
            corner_angle = abs(normalize_angle(radians(previous.angle_to(next))))
            target_speed = self.config.corner_velocity * (1 - self.config.corner_slow_down * corner_angle / pi)
            self.target_speeds.append(target_speed)

    @property
    def name(self):
        return "Road Runner"

    @property
    def contributor(self):
        return "Rayman [NC]"

    @property
    def color(self):
        return Color(200, 200, 0)

    def compute_commands(self, next_waypoint: int, position: Transform, velocity: Vector2) -> Tuple:
        target = self.track.lines[next_waypoint]
        next_target = self.track.lines[(next_waypoint + 1) % len(self.track.lines)]

        # calculate the target in the frame of the robot
        relative_target = position.inverse() * target
        # calculate the angle to the target
        angle = relative_target.as_polar()[1]

        min_velocity = float('inf')
        waypoint_distance = 0
        # print()
        # print(f'current velocity: {velocity.length():.2f}')
        for i in crange(next_waypoint, next_waypoint + 10, len(self.track.lines)):
            if i == next_waypoint:
                waypoint_distance = relative_target.length()
            else:
                waypoint_distance += (self.track.lines[i] - self.track.lines[i - 1]).length()
            target_speed = self.target_speeds[i]
            max_velocity = sqrt(target_speed ** 2 + 2 * self.config.deceleration * waypoint_distance)
            if max_velocity < min_velocity:
                # print(f'{i}\t{waypoint_distance:.2f}\t{target_speed:.2f}\t{max_velocity:.2f} *')
                min_velocity = max_velocity
            # else:
            #     print(f'{i}\t{waypoint_distance:.2f}\t{target_speed:.2f}\t{max_velocity:.2f}')
        # print(f'min velocity: {min_velocity:.2f}')

        if DEBUG:
            data = {
                'angle': angle,
                'velocity': velocity.length(),
                'min_velocity': min_velocity,
            }
            self.sock.sendto(json.dumps(data).encode('utf-8'), self.server_address)

        if velocity.length() < min_velocity:
            throttle = 1
        else:
            throttle = -1

        # calculate the steering
        if angle > 0:
            return throttle, 1
        else:
            return throttle, -1

    def draw(self, map_scaled: Surface, zoom):
        if DEBUG:
            # Draw the target speeds
            for i, target_speed in enumerate(self.target_speeds):
                text = self.font.render(f'{target_speed:.2f}', True, (0, 0, 0))
                map_scaled.blit(text, (self.track.lines[i].x * zoom, self.track.lines[i].y * zoom))
