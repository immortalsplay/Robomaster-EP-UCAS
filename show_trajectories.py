import numpy as np
import matplotlib.pyplot as plt

ekf_pos = []
odom_pos = []
uwb_pos = []

with open("ekf_pos.txt", "r") as f:
    while True:
        line = f.readline()
        if not line:
            break
        pos_x, pos_y, _, _ = [float(i) for i in line.split(',')]
        ekf_pos.append([pos_x, pos_y])
with open("odom_pos.txt", "r") as f:
    while True:
        line = f.readline()
        if not line:
            break
        pos_x, pos_y = [float(i) for i in line.split(',')]
        odom_pos.append([pos_x, pos_y])
with open("uwb_pos.txt", "r") as f:
    while True:
        line = f.readline()
        if not line:
            break
        pos_x, pos_y = [float(i) for i in line.split(',')]
        uwb_pos.append([pos_x, pos_y])

ekf_pos = np.array(ekf_pos)
odom_pos = np.array(odom_pos)
uwb_pos = np.array(uwb_pos)

plt.plot(ekf_pos[:, 0], ekf_pos[:, 1], ".")
# plt.plot(odom_pos[:, 0], odom_pos[:, 1], "r.")
plt.plot(uwb_pos[:, 0], uwb_pos[:, 1], "g.")
plt.xlim([0, 5])
plt.ylim([0, 8])
plt.show()