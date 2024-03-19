import os
import numpy as np
from scipy.signal import butter, filtfilt
import matplotlib.pyplot as plt
from build_release.libmjpc_rl_pywrap import loadData
from utils_python.Filter import Filter, FilterMean
plt.ion()


# Load the data.
current_dir = os.path.dirname(os.path.abspath(__file__))
relative_path = "../../mjpc_rl/log/tmp.bin"

# Construct the absolute path
filename = os.path.join(current_dir, relative_path)
data = loadData(filename)

def online_butter(xs, i):
    return butter(order, fc / (fs/2), btype="low")


# Generate some sample data
np.random.seed(0)
t = np.linspace(0, data.dt_simu * (data.size - 1), data.size)

# Define the filter parameters
order = 1  # Filter order
fs = 1 / data.dt_simu  # Sampling frequency (Hz)
fc = [2. for _ in range(6)]  # Cutoff frequency for each axis(Hz)
filter = Filter(fc, fs,order)
# filter = FilterMean(0.05,data.dt_simu)

filtered_states = []
for vel in data.qvel:
    filtered_states.append(filter.filter(vel[:6]))

# Compute the filter coefficients
# b, a = butter(order, fc / (fs / 2), btype='low')

# Apply the filter to the data
filtered_data = [x[2] for x in filtered_states]
y =  [pos[2] for pos in data.qvel]

# Plot the original and filtered data
plt.figure(figsize=(10, 6))
plt.plot(t, y, label='Original Data')
plt.plot(t, filtered_data, color='red', label='Filtered Data')
plt.xlabel('Time')
plt.ylabel('Amplitude')
plt.title('Butterworth Low-pass Filter')
plt.legend()
plt.grid(True)
plt.show()
