import pandas as pd
import numpy as np
# from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

# Read the CSV file as tab-separated
df = pd.read_csv("output.csv", delimiter="\t", header=None)

# Create empty lists to store values
Rx = []
# Rx = []
RXX = []
Ty = []
Ry = []
TYY = []
Tz = []
Rz = []
TZZ = []

def exponential_func(x, a, b, c):
    return a * np.exp(b * x) + c

# Iterate over the DataFrame
for index, row in df.iterrows():
    row = df.loc[index, :].values.flatten().tolist()
    row = row[0].split(',')
    if row[0] == 'Rx':
        Rx.append(float(row[1]))
    # elif row[0] == 'Rx':
        Rx.append(float(row[1]))
    elif row[0] == 'RXX':
        RXX.append(float(row[1]))
    # Similarly for Ty, Ry, Tyy, etc.
    if row[2] == 'Ty':
        Ty.append(float(row[3]))
    elif row[2] == 'Ry':
        Ry.append(float(row[3]))
    elif row[2] == 'TYY':
        TYY.append(float(row[3]))
    if row[4] == 'Tz':
        Tz.append(float(row[5]))
    elif row[4] == 'Rz':
        Rz.append(float(row[5]))
    elif row[4] == 'TZZ':
        TZZ.append(float(row[5]))

Rx  = np.array(Rx )
# Rx  = np.array(Rx )
RXX = np.array(RXX)
Ty  = np.array(Ty )
Ry  = np.array(Ry )
TYY = np.array(TYY)
Tz  = np.array(Tz )
Rz  = np.array(Rz )
TZZ = np.array(TZZ)

# print("TX", Tx ,"\n\n\n\n\n")
# print("TXX",TXX)

# popt, _ = curve_fit(exponential_func, TXX, Tx)


# plt.scatter(TXX, Tx, color='blue', marker='o')

# # Plotting the fitted exponential curve
# x_vals = np.linspace(min(TXX), max(TXX), 1000)
# y_vals = exponential_func(x_vals, *popt)
# plt.plot(x_vals, y_vals, color='red', linestyle='-', linewidth=1)

# plt.title('Scatter plot with exponential fit')
plt.plot(Rx,)
plt.xlabel('Tx')
plt.ylabel('TXX')
plt.grid(True)
plt.show()
# class ExponentialSmoothing:
#     def __init__(self, alpha):
#         self.alpha = alpha
#         self.last_smoothed_value = None

#     def smooth(self, current_value):
#         if self.last_smoothed_value is None:
#             self.last_smoothed_value = current_value
#             return current_value

#         smoothed_value = self.alpha * current_value + (1 - self.alpha) * self.last_smoothed_value
#         self.last_smoothed_value = smoothed_value

#         return smoothed_value

# # Usage:
# alpha = 0.2
# smoother = ExponentialSmoothing(alpha)

# readings = [5, 6, 7, 6, 5, 6, 7, 8, 7, 6, 5]  # example data

# for reading in readings:
#     smoothed_value = smoother.smooth(reading)
#     print(f"Actual: {reading}, Smoothed: {smoothed_value:.2f}")



