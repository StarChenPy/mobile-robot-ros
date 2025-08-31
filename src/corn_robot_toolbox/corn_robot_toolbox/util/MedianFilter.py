from collections import deque
import numpy as np

class MedianFilter:
    def __init__(self, window_size=7):
        self.values = deque(maxlen=window_size)

    def update(self, new_value: float) -> float:
        self.values.append(new_value)
        return float(np.median(self.values))