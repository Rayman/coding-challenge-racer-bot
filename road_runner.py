import json
from math import sqrt
from socket import socket, AF_INET, SOCK_DGRAM
from typing import Tuple

from pygame import Vector2, Color

from ...bot import Bot
from ...linear_math import Transform


class RoadRunner(Bot):
    def __init__(self, track):
        super().__init__(track)
        self.sock = socket(AF_INET, SOCK_DGRAM)
        self.server_address = ('127.0.0.1', 12345)

    @property
    def name(self):
        return "Road Runner"

    @property
    def contributor(self):
        return "Rayman"

    @property
    def color(self):
        return Color(200, 200, 0)

    def compute_commands(self, next_waypoint: int, position: Transform, velocity: Vector2) -> Tuple:
        target = self.track.lines[next_waypoint]
        # calculate the target in the frame of the robot
        target = position.inverse() * target
        # calculate the angle to the target
        angle = target.as_polar()[1]

        # calculate the throttle
        corner_velocity = 150
        deceleration = 150

        max_velocity = sqrt(corner_velocity ** 2 + 2 * deceleration * target.length())

        data = {
            'angle': angle,
            'velocity': velocity.length(),
            'max_velocity': max_velocity
        }
        self.sock.sendto(json.dumps(data).encode('utf-8'), self.server_address)

        if velocity.length() < max_velocity:
            throttle = 1
        else:
            throttle = -1

        # calculate the steering
        if angle > 0:
            return throttle, 1
        else:
            return throttle, -1
