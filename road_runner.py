import json
from argparse import Namespace
from socket import socket, AF_INET, SOCK_DGRAM
from typing import Tuple

from pygame import Vector2, Color, Surface, font

from .utils import calculate_target_speeds, calculate_target_speed
from ...bot import Bot
from ...linear_math import Transform
from ...track import Track

DEBUG = False


class RoadRunner(Bot):
    def __init__(self, track: Track):
        super().__init__(track)
        self.config = Namespace(
            corner_slow_down=1.3344255280275334,
            deceleration=125.64971221205201,
        )
        self.target_speeds = calculate_target_speeds(track, self.config.corner_slow_down)
        self.font = font.SysFont('', 20)
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

        max_speed = calculate_target_speed(self.track, position, next_waypoint, self.target_speeds,
                                           self.config.deceleration)

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
