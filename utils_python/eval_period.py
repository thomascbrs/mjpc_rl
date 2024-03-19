import os
import numpy as np
import matplotlib.pyplot as plt
from build_release.libmjpc_rl_pywrap import loadData

plt.ion()

# Load the data.
current_dir = os.path.dirname(os.path.abspath(__file__))
relative_path = "../../mjpc_rl/logs/logger/data.bin"
filename = os.path.join(current_dir, relative_path)

# Check if the file exists
if not os.path.exists(filename):
    error = "File does not exist: {}".format(filename) + ". Run simulation with LOGGER boolean turn on."
    raise RuntimeError(error)
else:
    data = loadData(filename)

# Generate some sample data
np.random.seed(0)
t = np.linspace(0, data.dt_simu * data.size, 1000)
# y = np.sin(2 * np.pi * 1 * t) + 0.5 * np.sin(2 * np.pi * 2.5 * t)  # Combination of two sinusoids
y = [pos[1] for pos in data.qvel]

# Perform Fourier Transform
fft_y = np.fft.fft(y)
freqs = np.fft.fftfreq(len(y), t[1] - t[0])

# Plot the magnitude of the Fourier Transform
plt.figure(figsize=(10, 6))
plt.plot(freqs[:len(freqs) // 2], np.abs(fft_y)[:len(freqs) // 2])
plt.xlabel('Frequency')
plt.ylabel('Magnitude')
plt.title('Fourier Transform')
plt.grid(True)
plt.show()
