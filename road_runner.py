import json
from argparse import Namespace
from math import sqrt, fmod, pi
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
            corner_slow_down=1.3474480201529782,
            deceleration=122.01115713324654,
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

        max_speed = self.calculate_target_speed(next_waypoint, position)

        if DEBUG:
            data = {
                'angle': angle,
                'speed': velocity.length(),
                'max_speed': max_speed,
            }
            self.sock.sendto(json.dumps(data).encode('utf-8'), self.server_address)

        if velocity.length() < max_speed:
            throttle = 1
        else:
            throttle = -1

        # calculate the steering
        if angle > 0:
            return throttle, 1
        else:
            return throttle, -1

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

    def draw(self, map_scaled: Surface, zoom):
        if DEBUG:
            # Draw the target speeds
            for i, target_speed in enumerate(self.target_speeds):
                text = self.font.render(f'{target_speed:.2f}', True, (0, 0, 0))
                map_scaled.blit(text, (self.track.lines[i].x * zoom, self.track.lines[i].y * zoom))

            for i, target_speed in enumerate(self.target_speeds):
                p0 = self.track.lines[(i - 1) % len(self.track.lines)]
                p1 = self.track.lines[i]
                p2 = self.track.lines[(i + 1) % len(self.track.lines)]
                a = (p2 - p1).length()
                b = (p0 - p2).length()
                c = (p0 - p1).length()
                area = 0.5 * abs(p0.x * (p1.y - p2.y) + p1.x * (p2.y - p0.y) + p2.x * (p0.y - p1.y))
                R = a * b * c / (4 * area)
                text = self.font.render(f'{R:.2f}', True, (200, 0, 0))
                map_scaled.blit(text, (p1.x * zoom, (p1.y + 20) * zoom))
