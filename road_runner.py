import json
from argparse import Namespace
from math import sqrt, fmod, pi, radians
from socket import socket, AF_INET, SOCK_DGRAM
from typing import Tuple

from pygame import Vector2, Color

from ...bot import Bot
from ...linear_math import Transform
from ...track import Track

DEBUG = False


def normalize_angle(angle):
    result = fmod(angle + pi, 2.0 * pi)
    if result <= 0.0:
        return result + pi
    return result - pi


class RoadRunner(Bot):
    def __init__(self, track: Track):
        super().__init__(track)
        self.config = Namespace(corner_velocity=285.1612416010679, deceleration=114.44908662365383,
                                corner_slow_down=1.9127399408946921)
        if DEBUG:
            self.sock = socket(AF_INET, SOCK_DGRAM)
            self.server_address = ('127.0.0.1', 12389)

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

        corner_angle = abs(normalize_angle(radians((target - position.p).angle_to(next_target - target))))
        corner_velocity = self.config.corner_velocity * (1 - self.config.corner_slow_down * corner_angle / pi)

        max_velocity = sqrt(corner_velocity ** 2 + 2 * self.config.deceleration * relative_target.length())

        if DEBUG:
            data = {
                'angle': angle,
                'velocity': velocity.length(),
                'max_velocity': max_velocity,
                'corner_angle': corner_angle,
                'corner_velocity': corner_velocity,
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
