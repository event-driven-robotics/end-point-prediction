
import os, numpy as np
from matplotlib import pyplot as plt
import matplotlib
matplotlib.use('TkAgg')


# PLOT SIMULATED TRAJECTORIES ONE BY ONE AGAINST A SINGLE, FIXED REAL ONE

path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'real_trajectories_final')
trajectories = sorted([traj for traj in os.listdir(path) if traj.endswith('.txt')])

sim_traj = np.loadtxt(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sim_trajectories_final', 'sim_traj_00002.txt'), delimiter=',')
sim_time = np.cumsum(sim_traj[:, 0])

plt.figure(figsize=(18, 14))
plt.gca()
for t in trajectories:
    plt.clf()

    real_traj = trajectory = np.loadtxt(os.path.join(path, t), delimiter=',')[:,:3]
    real_time = np.cumsum(real_traj[:,0])

    plt.suptitle(t)
    plt.subplot(321)
    plt.plot(sim_time, '.')
    plt.plot(real_time, '.')
    plt.xlabel('points')
    plt.ylabel('time')
    plt.ylim([0, 1.5])
    plt.grid()
    plt.legend(['simulated', 'real'])
    plt.subplot(323)
    plt.plot(304-sim_traj[:,1], '.')
    plt.plot(real_traj[:,1], '.')
    plt.xlabel('points')
    plt.ylabel('X')
    # plt.xlim([0, 1.5])
    plt.ylim([0, 304])
    plt.grid()
    plt.subplot(325)
    plt.plot(sim_traj[:,2], '.')
    plt.plot(real_traj[:,2], '.')
    plt.xlabel('points')
    plt.ylabel('Y')
    # plt.xlim([0, 1.5])
    plt.ylim([0, 240])
    plt.gca().invert_yaxis()
    plt.grid()
    plt.subplot(322)
    plt.plot(sim_traj[:,0], '.')
    plt.plot(real_traj[:,0], '.')
    plt.xlabel('points')
    plt.ylabel('delta T')
    # plt.xlim([0, 1.5])
    # plt.ylim([0, 240])
    plt.grid()
    plt.subplot(324)
    plt.plot(np.sqrt((sim_traj[1:,1]-sim_traj[:-1,1])**2+(sim_traj[1:,2]-sim_traj[:-1,2])**2), '.')
    plt.plot(np.sqrt((real_traj[1:,1]-real_traj[:-1,1])**2+(real_traj[1:,2]-real_traj[:-1,2])**2), '.')
    plt.xlabel('points')
    plt.ylabel('delta S')
    # plt.xlim([0, 1.5])
    # plt.ylim([0, 240])
    plt.grid()
    plt.subplot(326)
    plt.plot(sim_traj[:,1], sim_traj[:,2], '.')
    plt.plot(real_traj[:,1], real_traj[:,2], '.')
    plt.xlabel('X')
    plt.ylabel('Y')
    # plt.xlim([0, 1.5])
    # plt.ylim([0, 240])
    plt.gca().invert_yaxis()
    plt.grid()
