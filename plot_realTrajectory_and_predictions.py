
import os
import numpy as np
from matplotlib import pyplot as plt
import matplotlib
matplotlib.use('TkAgg')
np.random.seed(seed=3)


path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'real_trajectories_final')
trajectories = sorted([traj for traj in os.listdir(path) if traj.endswith('.txt')])

pxl2cm_X = 120/304      # 120cm of horizontal f.o.v. divided by 304pxl
pxl2cm_Y = 90/240       # 90cm of vertical f.o.v. divided by 240pxl
Xscaler   = [[0.0, 0.01],  [0, 304],    [0, 240]]            # check this!!!!
Yscaler   = [[-2.0, 0.0], [-304, 304], [-240, 240]]          # check this!!!!

plt.figure(figsize=(18, 14))
plt.gca()
for t in trajectories:
    plt.clf()

    # plot prediction
    trajectory = np.loadtxt(os.path.join(path, t), delimiter=',')
    time = np.cumsum(trajectory[:,0])

    # time
    plt.subplot(3, 1, 1)
    plt.plot(time, time, marker='.', color='magenta', alpha=0.5, markersize=4, label='input')
    plt.plot(time, time - trajectory[:, 3], marker='.', color='limegreen', alpha=0.5, markersize=4, label='prediction')
    plt.hlines(time[-1], xmin=0.0, xmax=time[-1], color='black')
    plt.xlim([-0.05, 1.5])
    plt.ylim([-0.05, 1.5])
    plt.xlabel('time [s]')
    plt.ylabel('time [s]')
    plt.grid()
    plt.legend(loc='best')

    # X
    plt.subplot(3, 1, 2)
    plt.plot(time, trajectory[:, 1] * pxl2cm_X, marker='.', color='magenta', alpha=0.5, markersize=4, label='input')
    plt.plot(time, trajectory[:, 4] * pxl2cm_X, marker='.', color='limegreen', alpha=0.5, markersize=4, label='prediction')
    plt.hlines(trajectory[-1, 1] * pxl2cm_X, xmin=0.0, xmax=time[-1], color='black')
    plt.xlim([-0.05, 1.5])
    plt.ylim([-10, 314 * pxl2cm_X])
    plt.xlabel('time [s]')
    plt.ylabel('X [cm]')
    plt.grid()
    plt.legend(loc='best')

    # Y
    plt.subplot(3, 1, 3)
    plt.plot(time, trajectory[:, 2] * pxl2cm_Y, marker='.', color='magenta', alpha=0.5, markersize=4, label='input')
    plt.plot(time, trajectory[:, 5] * pxl2cm_Y, marker='.', color='limegreen', alpha=0.5, markersize=4, label='prediction')
    plt.hlines(trajectory[-1, 2] * pxl2cm_Y, xmin=0.0, xmax=time[-1], color='black')
    plt.xlim([-0.05, 1.5])
    plt.ylim([-20, 250 * pxl2cm_Y])
    plt.xlabel('time [s]')
    plt.ylabel('Y [cm]')
    plt.grid()
    plt.gca().invert_yaxis()
    plt.legend(loc='best')

    # plt.savefig(f'real_traj_prediction_00002.png')
