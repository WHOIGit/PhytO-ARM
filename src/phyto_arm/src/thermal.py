#!/usr/bin/env python3
'''
Trip logic and lockout persistence for IFCB thermal shutdown. Kept free of ROS
so that it can be unit tested.
'''
import json
import math
import os
import time


class ThermalMonitor:
    '''
    Decides when the IFCB should be shut down based on its environmental sensor.

    Trips when either:
      - every valid temperature (or humidity, if enabled) reading has been over
        the limit for at least `trip_duration` seconds and `min_readings`
        readings, so that a single blip does not trip it; or
      - the IFCB is powered and connected but no valid temperature reading has
        arrived for `stale_timeout` seconds (0 disables this check). A lost
        connection is left to the arm's own handling rather than treated as a
        thermal problem.

    Temperatures outside `valid_range`, humidities outside 0-100 %, and
    non-finite values are treated as sensor
    errors: they neither trip nor reset an over-limit streak, but they also do
    not count as a valid reading for the staleness check.
    '''

    def __init__(self, max_temperature, trip_duration, min_readings,
                 stale_timeout, valid_range=(-20.0, 85.0), max_humidity=None,
                 now=0.0):
        self.max_temperature = max_temperature
        self.max_humidity = max_humidity
        self.trip_duration = trip_duration
        self.min_readings = min_readings
        self.stale_timeout = stale_timeout
        self.valid_range = valid_range

        self.powered = True  # assume powered until told otherwise
        self.powered_since = now
        self.connected = False
        self.connected_since = now
        self.last_valid_time = None
        self.last_temperature = None
        self.last_humidity = None

        self._streaks = {'temperature': None, 'humidity': None}

    @staticmethod
    def _valid(value, valid_range):
        lo, hi = valid_range
        return value is not None and math.isfinite(value) and lo <= value <= hi

    def _update_streak(self, name, value, limit, now):
        if value > limit:
            since, count = self._streaks[name] or (now, 0)
            self._streaks[name] = (since, count + 1)
        else:
            self._streaks[name] = None

    def set_powered(self, powered, now):
        if powered and not self.powered:
            self.powered_since = now
        if not powered:
            self._streaks = {k: None for k in self._streaks}
        self.powered = powered

    def set_connected(self, connected, now):
        if connected and not self.connected:
            self.connected_since = now
        self.connected = connected

    def reading(self, temperature, humidity, now):
        if self._valid(temperature, self.valid_range):
            self.last_valid_time = now
            self.last_temperature = temperature
            self._update_streak('temperature', temperature, self.max_temperature, now)

        if self.max_humidity is not None and self._valid(humidity, (0.0, 100.0)):
            self.last_humidity = humidity
            self._update_streak('humidity', humidity, self.max_humidity, now)

    def check(self, now):
        '''Returns a human-readable reason to trip, or None.'''
        limits = {'temperature': self.max_temperature, 'humidity': self.max_humidity}
        for name, streak in self._streaks.items():
            if streak is None:
                continue
            since, count = streak
            if now - since >= self.trip_duration and count >= self.min_readings:
                return (f'{name} over {limits[name]} for {now - since:.0f} s '
                        f'({count} readings)')

        if self.powered and self.connected and self.stale_timeout > 0:
            last = max(self.last_valid_time or 0, self.powered_since, self.connected_since)
            if now - last >= self.stale_timeout:
                return f'no valid temperature reading for {now - last:.0f} s'

        return None


# The lockout file is the persistent record that a thermal shutdown happened.
# While it exists, nothing in PhytO-ARM may power the IFCB or pump back on. It
# is only removed by a person, which is what makes the restart manual.

def lockout_active(path):
    return bool(path) and os.path.exists(path)


def read_lockout(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def write_lockout(path, reason, completed=False, **extra):
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    record = {
        'reason': reason,
        'time': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'completed': completed,
        **extra,
    }
    tmp = f'{path}.tmp'
    with open(tmp, 'w') as f:
        json.dump(record, f, indent=2)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    return record
