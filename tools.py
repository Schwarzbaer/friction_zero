from math import copysign


def adjust_within_limit(target_value, base_value, limit):
    if abs(target_value - base_value) <= limit:
        return target_value
    else:
        return base_value + copysign(limit, target_value - base_value)
