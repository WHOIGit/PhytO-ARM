#!/usr/bin/env python3
"""
Standalone scheduling logic for digital logger power switch outlets.
This module contains pure functions with no ROS dependencies for easy testing.
"""
from datetime import datetime, time
from typing import List, Optional


def parse_time_string(time_str: str) -> time:
    return datetime.strptime(time_str, "%H:%M").time()


def should_outlet_be_on(switch_on_times: List[str], switch_off_times: List[str],
                       current_time: Optional[time] = None) -> Optional[bool]:
    if not switch_on_times and not switch_off_times:
        return None

    if current_time is None:
        current_time = datetime.now().time()

    on_times = [parse_time_string(t) for t in switch_on_times] if switch_on_times else []
    off_times = [parse_time_string(t) for t in switch_off_times] if switch_off_times else []

    # Find the most recent scheduled action
    should_be_on = None
    most_recent_schedule = None

    # Check on times
    for on_time in on_times:
        if on_time <= current_time:
            # This on_time happened today and is before current time
            if most_recent_schedule is None or on_time > most_recent_schedule:
                most_recent_schedule = on_time
                should_be_on = True

    # Check off times
    for off_time in off_times:
        if off_time <= current_time:
            # This off_time happened today and is before current time
            if most_recent_schedule is None or off_time > most_recent_schedule:
                most_recent_schedule = off_time
                should_be_on = False

    # If no action found for today, assume the last action from yesterday
    # This handles the case where current time is before any scheduled times
    if should_be_on is None:
        # Find the latest time from yesterday (highest time value)
        all_times = on_times + off_times
        if all_times:
            latest_time = max(all_times)
            # Determine if this latest time was an on or off action
            if latest_time in on_times:
                should_be_on = True
            else:
                should_be_on = False

    return should_be_on
