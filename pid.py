import json
from argparse import Namespace
from math import radians
from socket import socket, AF_INET, SOCK_DGRAM
from typing import Tuple

import pygame
from pygame import Vector2, Color

from .utils import normalize_angle, calculate_target_speeds, calculate_target_speed
from ...bot import Bot
from ...linear_math import Transform

DEBUG = False


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
        self.target_speeds = calculate_target_speeds(track, self.config.corner_slow_down)
        if DEBUG:
            self.sock = socket(AF_INET, SOCK_DGRAM)
            self.server_address = ('127.0.0.1', 12389)

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

        target_speed = calculate_target_speed(self.track, position, next_waypoint, self.target_speeds,
                                              self.config.deceleration)
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

    def draw(self, map_scaled, zoom):
        target = self.position.p + self.velocity
        pygame.draw.line(map_scaled, (255, 0, 0), self.position.p * zoom, target * zoom, 2)
