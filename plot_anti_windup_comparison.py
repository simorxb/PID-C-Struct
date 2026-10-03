"""Plot the classical vs general back-calculation comparison.

Reads data_PID_anti_windup_comparison.txt written by Test_PID_anti_windup_comparison.
Columns: t, setpoint, z_classic, u_classic, u_unsat_classic, z_general, u_general, u_unsat_general
"""

import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt("data_PID_anti_windup_comparison.txt")
t = data[:, 0]
setpoint = data[:, 1]
z_classic = data[:, 2]
u_classic = data[:, 3]
u_unsat_classic = data[:, 4]
z_general = data[:, 5]
u_general = data[:, 6]
u_unsat_general = data[:, 7]

fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 8))

axes[0].plot(t, setpoint, "k--", linewidth=1.5, label="Setpoint")
axes[0].plot(t, z_classic, linewidth=1.8, label="Classical back-calculation")
axes[0].plot(t, z_general, linewidth=1.8, label="General back-calculation")
axes[0].set_ylabel("Position (m)")
axes[0].legend(loc="best")
axes[0].grid(True, alpha=0.4)

axes[1].plot(t, u_classic, linewidth=1.8, label="Classical back-calculation")
axes[1].plot(t, u_general, linewidth=1.8, label="General back-calculation")
axes[1].set_ylabel("Force (N)")
axes[1].legend(loc="best")
axes[1].grid(True, alpha=0.4)

axes[2].plot(t, u_unsat_classic, linewidth=1.8, label="Classical back-calculation")
axes[2].plot(t, u_unsat_general, linewidth=1.8, label="General back-calculation")
axes[2].set_ylabel("Unsaturated force (N)")
axes[2].set_xlabel("Time (s)")
axes[2].legend(loc="best")
axes[2].grid(True, alpha=0.4)

fig.tight_layout()
fig.savefig("pid_anti_windup_comparison.png", dpi=150)
plt.show()
