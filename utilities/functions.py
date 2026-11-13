def clamp(val: float, low: float, high: float) -> float:
    return max(min(val, high), low)
