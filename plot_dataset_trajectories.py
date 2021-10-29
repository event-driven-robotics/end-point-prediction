
import os, numpy as np
from tqdm import tqdm
from matplotlib import pyplot as plt


datasets = ['asynch_trajectories_new']

plt.figure(figsize=(12, 9))


# PLOT TRAJECTORIES ONE BY ONE
for dataset in datasets:
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), dataset)
    num_trajs = len([traj for traj in os.listdir(path) if traj.endswith(".txt")])
    for i in range(51, num_trajs+1):
        plt.clf()
        traj = np.loadtxt(os.path.join(path, 'asynch_traj_'+str(i).zfill(5)+'.txt'), delimiter=',')
        plt.title("Trajectory #"+str(i))
        plt.subplot(311)
        plt.title("Trajectory # " + str(i))
        plt.plot(np.cumsum(traj[:,0]), traj[:,1], '.')
        plt.xlabel('time')
        plt.ylabel('X')
        plt.ylim([0, 304])
        plt.subplot(312)
        plt.plot(np.cumsum(traj[:,0]), traj[:,2], '.')
        plt.xlabel('time')
        plt.ylabel('Y')
        plt.ylim([0, 240])
        plt.gca().invert_yaxis()
        plt.subplot(313)
        plt.plot(traj[:,1], traj[:,2], '.')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.xlim([0, 304])
        plt.ylim([0, 240])
        plt.gca().invert_yaxis()
        plt.show()


# # PLOT TRAJECTORIES ALL TOGETHER (x(t), y(t) and y(x))
# for dataset in datasets:
#     path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'trajectories_'+dataset)
#     num_trajs = len([traj for traj in os.listdir(path) if traj.endswith(".txt")])
#     for i in tqdm(range(1, num_trajs+1)):
#         traj = np.loadtxt(os.path.join(path, 'traj_'+str(i).zfill(5)+'.txt'), delimiter=',')
#         plt.subplot(311)
#         plt.plot(traj[:, 0], traj[:, 1], '.')
#         plt.subplot(312)
#         plt.plot(traj[:, 0], traj[:, 2])
#         plt.subplot(313)
#         plt.plot(traj[:, 1], traj[:, 2])
# plt.title("Trajectories visual field span")
# plt.subplot(311)
# plt.xlabel('time')
# plt.ylabel('X')
# plt.ylim([0, 304])
# plt.subplot(312)
# plt.xlabel('time')
# plt.ylabel('Y')
# plt.ylim([0, 240])
# plt.gca().invert_yaxis()
# plt.subplot(313)
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.xlim([0, 304])
# plt.ylim([0, 240])
# plt.gca().invert_yaxis()
# plt.show()


# # PLOT TRAJECTORIES ALL TOGETHER (t(x) and y(x))
# path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'trajectories')
# num_trajs = len([traj for traj in os.listdir(path) if traj.endswith(".txt")])
# for i in tqdm(range(1, num_trajs+1)):
#     traj = np.loadtxt(os.path.join(path, 'traj_'+str(i).zfill(5)+'.txt'), delimiter=',')
#     plt.subplot(211)
#     plt.plot(traj[:, 1], traj[:, 0], '.')
#     plt.subplot(212)
#     plt.plot(traj[:, 1], traj[:, 2])
# plt.title("Trajectories visual field span")
# plt.subplot(211)
# plt.subplot(211).yaxis.tick_right()
# plt.xlabel('X')
# plt.ylabel('time')
# plt.ylim([0, 2])
# plt.gca().invert_xaxis()
# plt.subplot(212)
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.ylim([0, 240])
# plt.gca().invert_yaxis()
# plt.show()