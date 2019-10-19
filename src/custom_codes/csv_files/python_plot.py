import csv
import numpy as np
# import matplotlib
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

with open('Joint_states.csv') as csvfile:
    reader = list(csv.reader(csvfile))

    rownum = 0
    joint_angles = []
    Joint1 = []
    index = []

    for row in reader[1:]:
        new_row = row[0].split()[1:]
        # raw_input(new_row)
        joint_angles.append(new_row)
        Joint1.append(new_row[0])
        index.append(rownum)
        rownum += 1

# print(np.asarray(index[:100]))
# print(np.asarray(Joint1[:100]))
index = np.asarray(index)
Joint1 = np.asarray(Joint1)
Joint1 = Joint1.astype(np.float)
plt.plot(index, Joint1, label='true')

red_index = index[0:-1:15]
red_Joint1 = Joint1[0:-1:15]
cs = CubicSpline(red_index, red_Joint1)
xs = np.arange(0, 983, 0.5)
plt.plot(red_index, cs(red_index), label='Cubic Spline - S')
plt.legend(loc='upper left', ncol=2)

plt.xlabel("Iterations")
plt.ylabel("Joint angles [rad]")
plt.title("AAPF method with Orientation Control")
plt.ylim(min(Joint1)-0.1, max(Joint1)+0.1)
plt.xlim(0, 983)

plt.show()
