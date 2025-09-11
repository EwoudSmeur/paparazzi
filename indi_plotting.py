import pandas as pd
import matplotlib.pyplot as plt
import sys

filepath = 'var/logs/20250909-124435.csv'

try:
    # Load CSV file with headers. Must update the csv being logged
    df = pd.read_csv(filepath)   
except FileNotFoundError:
    print(f"Error: Filepath '{filepath}' does not exist. Please update filepath.")
    sys.exit(1) 



# Plotting
plt.figure(figsize=(12, 6))
plt.plot(df['time'], df['pos_x_ref'], label='Position Ref', linewidth=2)
plt.plot(df['time'], df['pos_x_actual'], label='Position Actual', linewidth=2)
# plt.plot(df['time'], df['vel_x_ref'], label='Velocity Ref', linewidth=2)
# plt.plot(df['time'], df['vel_x_actual'], label='Velocity Actual', linewidth=2)
plt.plot(df['time'], df['acc_x_ref'], label='Acceleration Ref', linewidth=2)
plt.plot(df['time'], df['acc_x_actual'], label='Acceleration Actual', linewidth=2)
# plt.plot(df['time'], df['rate_p'], label='Rate p', linewidth=2)
# plt.plot(df['time'], df['rate_q'], label='Rate q', linewidth=2)
# plt.plot(df['time'], df['rate_r'], label='Rate r', linewidth=2)
# plt.plot(df['time'], df['T_calculated'], label='T calculated', linewidth=2)
# plt.plot(df['time'], df['roll_rate_cmd'], label='Commanded roll rate', linewidth=2)
# plt.plot(df['time'], df['pitch_rate_cmd'], label='Commanded pitch rate', linewidth=2)


plt.xlabel('Time (s)')
plt.ylabel('Value')
plt.title('Logging')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()