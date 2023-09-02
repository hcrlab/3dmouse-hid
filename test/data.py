import pandas as pd
import numpy as np
# Read the CSV file as tab-separated
df = pd.read_csv("output - 2023-09-01T155646.793.csv", delimiter="\t", header=None)

# Create empty lists to store values
Tx = []
Rx = []
TXX = []
Ty = []
Ry = []
TYY = []
Tz = []
Rz = []
TZZ = []

# Iterate over the DataFrame
for index, row in df.iterrows():
    row = df.loc[index, :].values.flatten().tolist()
    row = row[0].split(',')
    if row[0] == 'Tx':
        Tx.append(float(row[1]))
    elif row[0] == 'Rx':
        Rx.append(float(row[1]))
    elif row[0] == 'TXX':
        TXX.append(float(row[1]))
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

Tx  = np.array(Tx )
Rx  = np.array(Rx )
TXX = np.array(TXX)
Ty  = np.array(Ty )
Ry  = np.array(Ry )
TYY = np.array(TYY)
Tz  = np.array(Tz )
Rz  = np.array(Rz )
TZZ = np.array(TZZ)

import matplotlib.pyplot as plt

plt.plot(Tx)
plt.plot(TXX)

plt.show()
