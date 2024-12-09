import json
from argparse import Namespace
from math import atan2, radians
from socket import socket, AF_INET, SOCK_DGRAM
from typing import Tuple

from pygame import Vector2, Color

from .utils import normalize_angle, calculate_target_speeds, calculate_target_speed
from ...bot import Bot
from ...linear_math import Transform

DEBUG = True


class VectorPursuit(Bot):
    def __init__(self, track):
        super().__init__(track)
        self.config = Namespace(
            corner_slow_down=1,
            deceleration=50,
            k=6,
        )
        self.target_speeds = calculate_target_speeds(track, self.config.corner_slow_down)
        if DEBUG:
            self.sock = socket(AF_INET, SOCK_DGRAM)
            self.server_address = ('127.0.0.1', 12389)

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

        # angle of next segment
        target_angle = normalize_angle(radians((self.track.lines[(next_waypoint + 1) % len(self.track.lines)] -
                                                self.track.lines[next_waypoint]).as_polar()[1]) - position.M.angle)
        target_angle = -target_angle
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
            if DEBUG:
                data = {
                    'target_angle': target_angle,
                    'phi_1': phi_1,
                    'phi_2': phi_2,
                    'phi': phi,
                    'term_1': term_1,
                    'term_2': term_2,
                    'turning_radius': turning_radius,
                    'curvature': 1 / turning_radius,
                }
                self.sock.sendto(json.dumps(data).encode('utf-8'), self.server_address)
        except ZeroDivisionError:
            turning_radius = float('inf')

        target_speed = calculate_target_speed(self.track, position, next_waypoint, self.target_speeds,
                                              self.config.deceleration)
        angular_velocity = target_speed / turning_radius

        if target.y < 0:
            angular_velocity *= -1

        if target_speed < velocity.length():
            throttle = -1
        else:
            throttle = 1

            if DEBUG:
                data = {
                    'throttle': throttle,
                    'angular_velocity': angular_velocity,
                }
                self.sock.sendto(json.dumps(data).encode('utf-8'), self.server_address)

        return throttle, angular_velocity
