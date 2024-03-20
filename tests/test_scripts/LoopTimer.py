''' A general stopwatch class to measure time taken between start and stop calls and store them in a circular buffer for analysis'''

import time
import numpy as np

class LoopTimer:
    def __init__(self, buffer_size=100):
        self.execution_times = np.zeros(buffer_size)
        self.start_time = None
        self.time_index = 0
        self.buffer_size = buffer_size

    def start(self):
        if self.start_time is not None:
            raise RuntimeError("Timer is already started")
        self.start_time = time.perf_counter()

    def stop(self):
        if self.start_time is None:
            raise RuntimeError("Timer has not been started")
        end_time = time.perf_counter()
        self.execution_times[self.time_index] = end_time - self.start_time
        self.time_index = (self.time_index + 1) % self.buffer_size  # Circular buffer
        self.start_time = None  # Reset start time

    def get_execution_times(self):
        return self.execution_times

    def average_time(self):
        return np.mean(self.execution_times[:self.time_index])