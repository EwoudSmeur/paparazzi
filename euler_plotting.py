# ---- This file numerically differentiates position and velocity data to check the zero points of the 
# measured velocity and acceleration. This is to understand the shifts of the reference (desired) and 
# measured values, and why there is no shift between acceleration curves in the built-in methods of 
# paparazzi.
#
# This is a temporary file that can be deleted after the acceleration shift issue is solved. ---------


import matplotlib.pyplot as plt


# Initialise variables
time = []
pos_a = []
pos_ref = []
vel_a = []
vel_ref = []
accel_a = []
accel_ref = []


# Read in data from file
file_path = "output.txt" 
with open(file_path, "r") as f:
    lines = f.readlines()

for line in lines[1:]:
    if line.strip():
        parts = line.strip().replace(',', ' ').split()
        nums = list(map(float, parts))
        time.append(nums[0])
        pos_a.append(nums[1])
        pos_ref.append(nums[2])
        vel_a.append(nums[3])
        vel_ref.append(nums[4])
        accel_a.append(nums[5])     # 'accel_a'
        accel_ref.append(nums[6])   # 'accel_ref'


# Get normal time and time increment increment
time_norm = [t / 500 for t in time]
time_incr = time_norm[0]


# Compute backwards Euler for velocity
vel_diff_scaled = [0] # Assume the first value is 0
for i in range(1, len(pos_a)):
    diff = 0.02 * (pos_a[i] - pos_a[i - 1])  / time_incr  # An arbitrary gain is used here to keep the numerically differentiated velocity at the same amplitude as the pprz-recorded velocity. Atm the focus is on the shifting of curves and not the magnitude, so I did not investigate why the amplitude is different.
    vel_diff_scaled.append(diff)

# Compute backwards Euler for acceleration
accel_diff_scaled = [0]  # Assume the first value is 0
for i in range(1, len(vel_a)):
    diff = 0.02 * (vel_a[i] - vel_a[i - 1])  / time_incr # An arbitrary gain is used here to keep the numerically differentiated velocity at the same amplitude as the pprz-recorded velocity. Atm the focus is on the shifting of curves and not the magnitude, so I did not investigate why the amplitude is different.
    accel_diff_scaled.append(diff)


# Plotting velocity
plt.figure(figsize=(12, 6))
plt.plot(time_norm, vel_ref, label="vel_ref", color="green")
plt.plot(time_norm, vel_a, label="vel_a: from paparazzi function", color="blue")
plt.plot(time_norm, vel_diff_scaled, label="vel_a: from numerically differentiated position", color="red")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity Comparison")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# Plotting acceleration
plt.figure(figsize=(12, 6))
plt.plot(time_norm, accel_ref, label="accel_ref", color="green")
plt.plot(time_norm, accel_a, label="accel_a: from paparazzi function", color="blue")
plt.plot(time_norm, accel_diff_scaled, label="acceleration_a: from numerically differentiated velocity", color="red")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (m/s**2)")
plt.title("Acceleration Comparison")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
