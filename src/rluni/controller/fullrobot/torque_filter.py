from collections import deque
from scipy.signal import butter, filtfilt
import numpy as np

from scipy.signal import iirnotch

class TorqueFilter:
    def __init__(self, fs=80, target_freq=19.5, buffer_size=20):
        """
        fs: Sampling frequency in Hz.
        target_freq: Frequency of interest in Hz.
        buffer_size: Number of samples in the buffer.
        """
        self.fs = fs
        self.target_freq = target_freq
        self.buffer = deque(maxlen=buffer_size)
        
    def process_torque(self, torque):
        """
        Filters the torque signal and returns the amplitude and phase at the target frequency.
        """
        self.add_sample(torque)
        amp, phas = self.compute_amp_phase()
        torque = torque + 2*amp * np.sin(phas)
        print(amp, phas)
        
        return torque

    def add_sample(self, sample):
        """
        Adds a new sample to the buffer.
        When the buffer is full, returns (amplitude, phase).
        Otherwise, returns None.
        """
        self.buffer.append(sample)

    def compute_amp_phase(self):
        """
        Computes the amplitude and phase for the target frequency using the dot product method.
        The method assumes the samples are uniformly spaced in time.
        """
        data = np.array(self.buffer)
        N = len(data)
        # Create a time vector based on the number of samples and sampling rate.
        t = np.arange(N) / self.fs

        # Create cosine and sine reference waves at the target frequency.
        cos_wave = np.cos(2 * np.pi * self.target_freq * t)
        sin_wave = np.sin(2 * np.pi * self.target_freq * t)

        # Compute the dot products (projections) onto the cosine and sine waves.
        dot_cos = np.dot(data, cos_wave)
        dot_sin = np.dot(data, sin_wave)

        # The factor 2/N accounts for the averaging and the fact that we’re using both sine and cosine.
        amplitude = (2.0 / N) * np.sqrt(dot_cos**2 + dot_sin**2)
        phase = np.arctan2(dot_sin, dot_cos)

        return amplitude, phase