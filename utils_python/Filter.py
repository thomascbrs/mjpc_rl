
from argparse import ArgumentTypeError
import numpy as np
from scipy.signal import butter


class FilterMean():
    def __init__(self, period, dt):
        self._Nx = int(period / dt)
        self._x_queue = []  # List of precedent values

    def filter(self, q):
        """ Moving mean over period * dt sample.
        No need to handle modulo, should not get jump in angles (yaw) during one period ?
        Args :
            - q (list) : List of size 6.
        """
        if len(q) != 6:
            raise ArgumentTypeError("q should be size 6")
        if len(self._x_queue) == self._Nx:
            self._x_queue.pop(0)

        # Handle modulo for orientation
        if len(self._x_queue) > 0 and abs(q[5] - self._x_queue[0][5]) > 1.5 * np.pi:
            self.handle_modulo(q[5] - self._x_queue[0][5] > 0)

        if type(q) == np.ndarray:
            self._x_queue.append(q.tolist())
        else:
            self._x_queue.append(q)

        return np.mean(self._x_queue, axis=0)

    def handle_modulo(self, dir):
        """ Add or remove 2 PI to all elements in the queue for yaw angle.
        """
        for x in self._x_queue:
            if dir:
                x[5] += 2 * np.pi
            else:
                x[5] += -2 * np.pi


class Filter():
    """ Simple implementation of a lowpass filter.
    """
    def __init__(self, cutoff, fs, order=1):
        """
        Args:
            - cutoff (list): Cutoff frequencies for each axis.
            - fs (float): Sampling frequency.
            - order (int): Order of the filter.
        """
        _b, _a = [], []
        for k in range(6):
            b, a = self.butter_lowpass(cutoff[k], fs, order)
            _b.append(b)
            _a.append(a)
        self._nb = len(_b[0])
        self._na = len(_a[0])
        self._b = np.array(_b)
        self._a = np.array(_a)

        self._x_queue, self._y_queue = [], []
        self._is_initialized = False

    def butter_lowpass(self, cutoff, fs, order=1):
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        return b, a

    def filter(self, q):
        """ Filter.
        Args:
            - q (array x6): State to filter.

        Returns:
            - param1 (array x6): State filtered.
        """
        if not self._is_initialized:
            self._x_queue = self._nb * [q]
            self._y_queue = (self._na - 1) * [q]
            self._is_initialized = True

        # Handle modulo for orientation
        if abs(q[5] - self._y_queue[0][5]) > 1.5 * np.pi:
            self.handle_modulo(q[5] - self._y_queue[0][5] > 0)

        self._x_queue.pop()
        self._x_queue.insert(0, q)

        acc_ = 0
        for i in range(self._nb):
            acc_ += self._b[:, i] * self._x_queue[i]
        for i in range(self._na - 1):
            acc_ -= self._a[:, i + 1] * self._y_queue[i]
        self._y_queue.pop()
        self._y_queue.insert(0, acc_ / self._a[:, 0])
        return np.array(self._y_queue[0])

    def handle_modulo(self, dir):
        """ Add or remove 2 PI to all elements in the queue for yaw angle.
        """
        for x in self._x_queue:
            if dir:
                x[5] += 2 * np.pi
            else:
                x[5] += -2 * np.pi

        for y in self._y_queue:
            if dir:
                y[5] += 2 * np.pi
            else:
                y[5] += -2 * np.pi