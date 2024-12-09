import json
from argparse import Namespace
from math import sqrt, radians
from socket import socket, AF_INET, SOCK_DGRAM
from typing import Tuple

import pygame
from pygame import Vector2, Color

from .utils import normalize_angle
from ...bot import Bot
from ...linear_math import Transform
from ...track import Track

DEBUG = False


def crange(start, end, modulo):
    for i in range(start, end):
        yield i % modulo


class PID(Bot):
    def __init__(self, track):
        super().__init__(track)
        self.config = Namespace(
            deceleration=127.17799956741216,
            corner_slow_down=1.4250013202302734,
            p=4.642214730859962,
            d=11.364402700385446,
        )
        self.previous_error = 0
        self.target_speeds = []
        self.calculate_target_speeds(track)
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
        return "PID"

    @property
    def contributor(self):
        return "Rayman [NC]"

    @property
    def color(self):
        return Color(0, 200, 0)

    def compute_commands(self, next_waypoint: int, position: Transform, velocity: Vector2) -> Tuple:
        target = self.track.lines[next_waypoint]

        reference = radians((target - position.p).as_polar()[1])
        measured = radians((velocity.as_polar()[1]))
        error = normalize_angle(reference - measured)

        target_speed = self.calculate_target_speed(next_waypoint, position)
        if target_speed < velocity.length():
            throttle = -1
        else:
            throttle = 1

        steering_command = self.config.p * error + self.config.d * (error - self.previous_error)

        self.previous_error = error
        self.position = position
        self.velocity = velocity

        if DEBUG:
            data = {
                'reference': reference,
                'measured': measured,
                'steering_command': steering_command,
            }
            print(data)
            self.sock.sendto(json.dumps(data).encode('utf-8'), self.server_address)

        return throttle, steering_command

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

    def draw(self, map_scaled, zoom):
        target = self.position.p + self.velocity
        pygame.draw.line(map_scaled, (255, 0, 0), self.position.p * zoom, target * zoom, 2)
