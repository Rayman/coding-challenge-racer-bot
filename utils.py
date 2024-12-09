from math import fmod, pi, sqrt
from typing import List

from ...linear_math import Transform
from ...track import Track


def normalize_angle(angle):
    result = fmod(angle + pi, 2.0 * pi)
    if result <= 0.0:
        return result + pi
    return result - pi


def calculate_target_speeds(track: Track, corner_slow_down: float):
    target_speeds = []
    for i in range(len(track.lines)):
        p0 = track.lines[(i - 1) % len(track.lines)]
        p1 = track.lines[i]
        p2 = track.lines[(i + 1) % len(track.lines)]
        a = (p2 - p1).length()
        b = (p0 - p2).length()
        c = (p0 - p1).length()
        area = 0.5 * abs(p0.x * (p1.y - p2.y) + p1.x * (p2.y - p0.y) + p2.x * (p0.y - p1.y))
        R = a * b * c / (4 * area)

        target_speed = corner_slow_down * R
        target_speeds.append(target_speed)
    return target_speeds


def crange(start, end, modulo):
    for i in range(start, end):
        yield i % modulo


def calculate_target_speed(track: Track, position: Transform, next_waypoint: int, target_speeds: List[float],
                           deceleration: float):
    min_speed = float('inf')
    waypoint_distance = 0
    # print()
    # print(f'current speed: {velocity.length():.2f}')
    for i in crange(next_waypoint, next_waypoint + 10, len(track.lines)):
        if i == next_waypoint:
            waypoint_distance = (track.lines[i] - position.p).length()
        else:
            waypoint_distance += (track.lines[i] - track.lines[i - 1]).length()
        target_speed = target_speeds[i]
        max_speed = sqrt(target_speed ** 2 + 2 * deceleration * waypoint_distance)
        if max_speed < min_speed:
            # print(f'{i}\t{waypoint_distance:.2f}\t{target_speed:.2f}\t{max_speed:.2f} *')
            min_speed = max_speed
        # else:
        #     print(f'{i}\t{waypoint_distance:.2f}\t{target_speed:.2f}\t{max_speed:.2f}')
    # print(f'min speed: {min_speed:.2f}')

    return min_speed
