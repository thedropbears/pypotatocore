import math


def apply_deadzone(value: float, threshold: float) -> float:
    """Apply a deadzone to a value in [-1,1].

    This ensures that the rest of the input space maps to [-1,1].
    """
    assert 0 <= threshold < 1
    magnitude = abs(value)
    if magnitude < threshold:
        return 0
    if threshold < 1e-3:
        # Avoid instability when threshold is small.
        return (value - math.copysign(threshold, value)) / (1 - threshold)
    scaled = 1 - (1 - magnitude) / (1 - threshold)
    return math.copysign(scaled, value)


def map_exponential(value: float, base: float) -> float:
    """Takes a value in [-1,1] and maps it to an exponential curve."""
    assert base > 1
    return math.copysign((base ** abs(value) - 1) / (base - 1), value)


def scale_value(
    value: float,
    input_lower: float,
    input_upper: float,
    output_lower: float,
    output_upper: float,
) -> float:
    """Scales a value based on the input range and output range.
    For example, to scale a joystick throttle (1 to -1) to 0-1, we would:
        scale_value(joystick.getThrottle(), 1, -1, 0, 1)
    """
    input_distance = input_upper - input_lower
    output_distance = output_upper - output_lower
    ratio = (value - input_lower) / input_distance
    result = ratio * output_distance + output_lower
    return result


def lerp(value: float, lo: float, hi: float) -> float:
    return value * (hi - lo) + lo
