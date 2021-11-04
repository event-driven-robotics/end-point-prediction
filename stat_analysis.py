
import os, numpy as np
from tqdm import tqdm
from matplotlib import pyplot as plt

import matplotlib
params = {'xtick.labelsize': 24,
          'ytick.labelsize': 24,
          'font.size': 13,
          'figure.autolayout': True,  # similar to tight_layout
          'figure.figsize': [7.2, 4.5],  # ratio 1.6 is pleasant
          'axes.titlesize': 13,
          'axes.labelsize': 28,
          'lines.linewidth': 2,
          'lines.markersize': 4,
          'legend.fontsize': 11}
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
plt.style.use(params)


################################################## FUNCTIONS ##################################################

def extract_perc_values(values, scaled_time, perc_points, range):
    means, stds = [], []
    for i, pt in enumerate(perc_points):
        if i == 0:
            elems = values[((scaled_time >= pt) & (scaled_time <= pt+range))]
        elif i == len(perc_points)-1:
            elems = values[(scaled_time >= pt-range) & (scaled_time <= pt)]
        else:
            elems = values[(scaled_time >= pt-range) & (scaled_time <= pt+range)]
        means.append(np.mean(elems))
        stds.append(np.std(elems))
    return means, stds

def moving_avg(x, n):
    cumsum = np.cumsum(x, axis=0, dtype=float)
    cumsum[n:] = cumsum[n:] - cumsum[:-n]
    return cumsum[n-1:] / float(n)


################################################## PARAMETERS ##################################################

n_points = 5
n_mov_avg = 5

D1_folder = "real_trajectories_final"
D2_folder = "sim_trajectories_final"

label1 = "real"
label2 = "simul"

perc_points = np.arange(0.0, 1.1, 0.1)


################################################ DATASET 1 PROCESSING ################################################

D1_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), D1_folder)
D1_files = sorted([traj for traj in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), D1_path)) if traj.endswith('.txt')])
D1_trajectories, D1_spatial_delta = [], []
D1_init, D1_init_vel, D1_angle, D1_mean_x_vel_list, D1_std_x_vel_list = [], [], [], [], []
D1_avg_acceleration, D1_accelerations, D1_avg_x_acceleration, D1_avg_y_acceleration = [], [], [], []
D1_mean_rate_list, D1_std_rate_list, D1_mean_rate_x_list, D1_std_rate_x_list = [], [], [], []
D1_restitution_coeffs = []
D1_bouncing_coordinates = []
for t in tqdm(D1_files, "Loading dataset_1 trajectories..."):

    # load data
    D1_trajectory = np.loadtxt(os.path.join(os.path.dirname(os.path.abspath(__file__)), D1_folder, t), delimiter=",")[:, :3]
    D1_trajectory[:, 0] = np.cumsum(D1_trajectory[:, 0])
    if D1_folder[0:7] != "asynch_" : D1_trajectory[:, 1] = 304 - D1_trajectory[:, 1]    # flip x-coordinate
    D1_trajectory[:, 2] = 240 - D1_trajectory[:, 2]    # flip y-coordinate
    D1_trajectories.append(D1_trajectory)

    # spatial delta
    deltas = D1_trajectory[1:, 1:3] - D1_trajectory[:-1, 1:3]
    D1_spatial_delta.append(np.mean(np.sqrt(deltas[:,0]**2 + deltas[:,1]**2)))

    # initial coordinates
    D1_init.append(np.array([D1_trajectory[0, 1], D1_trajectory[0, 2]]))

    # initial xy velocity and angle
    vel_init = sum(D1_trajectory[1:n_points+1, 1:3] - D1_trajectory[:n_points, 1:3]) / D1_trajectory[n_points, 0]
    # x_acc_init = np.abs(x_vel_init) / D1_trajectory[n_points, 0]
    # y_acc_init = np.abs(y_vel_init) / D1_trajectory[n_points, 0]
    D1_init_vel.append(vel_init)
    D1_angle.append(np.arctan2(vel_init[1], vel_init[0]))

    # # average acceleration
    # x_space_init2 = sum(D1_trajectory[n_points+1:2*n_points+1, 1] - D1_trajectory[n_points:2*n_points, 1])
    # y_space_init2 = sum(D1_trajectory[n_points+1:2*n_points+1, 2] - D1_trajectory[n_points:2*n_points, 2])
    # x_vel_init2 = np.abs(x_space_init2) / (D1_trajectory[2*n_points, 0] - D1_trajectory[n_points, 0])
    # y_vel_init2 = np.abs(y_space_init2) / (D1_trajectory[2*n_points, 0] - D1_trajectory[n_points, 0])
    # x_accel_init = (x_vel_init2-x_vel_init)/(D1_trajectory[2*n_points, 0] - D1_trajectory[n_points, 0])
    # y_accel_init = (y_vel_init2-y_vel_init)/(D1_trajectory[2*n_points, 0] - D1_trajectory[n_points, 0])
    # accel_init = np.sqrt(x_accel_init**2 + y_accel_init**2)
    #
    # x_space_final = sum(D1_trajectory[-n_points:, 1] - D1_trajectory[-n_points-1:-1, 1])
    # y_space_final = sum(D1_trajectory[-n_points:, 2] - D1_trajectory[-n_points-1:-1, 2])
    # x_vel_final = x_space_final / (D1_trajectory[-1, 0] - D1_trajectory[-n_points-1, 0])
    # y_vel_final = y_space_final / (D1_trajectory[-1, 0] - D1_trajectory[-n_points-1, 0])
    # x_space_final2 = sum(D1_trajectory[-2*n_points:-n_points, 1] - D1_trajectory[-2*n_points-1:-n_points-1, 1])
    # y_space_final2 = sum(D1_trajectory[-2*n_points:-n_points, 2] - D1_trajectory[-2*n_points-1:-n_points-1, 2])
    # x_vel_final2 = np.abs(x_space_final2) / (D1_trajectory[-n_points-1, 0] - D1_trajectory[-2*n_points-1, 0])
    # y_vel_final2 = np.abs(y_space_final2) / (D1_trajectory[-n_points-1, 0] - D1_trajectory[-2*n_points-1, 0])
    # x_accel_final = (x_vel_final-x_vel_final2)/(D1_trajectory[-1, 0] - D1_trajectory[-n_points-1, 0])
    # y_accel_final = (y_vel_final-y_vel_final2)/(D1_trajectory[-1, 0] - D1_trajectory[-n_points-1, 0])
    # accel_final = np.sqrt(x_accel_final**2 + y_accel_final**2)
    # D1_accelerations.append(np.array([x_accel_final/x_accel_init, y_accel_final/y_accel_init]))
    #
    # # velocities_x = (D1_trajectory[1:n_points+1, 1] - D1_trajectory[:n_points, 1]) / (D1_trajectory[1:n_points+1, 0] - D1_trajectory[:n_points, 0])
    # # velocities_y = (D1_trajectory[1:n_points + 1, 2] - D1_trajectory[:n_points, 2]) / (D1_trajectory[1:n_points+1, 0] - D1_trajectory[:n_points, 0])
    # # x_accel_init = (velocities_x[-1] - velocities_x[0]) / (D1_trajectory[n_points, 0] - D1_trajectory[1, 0])
    # # y_accel_init = (velocities_y[-1] - velocities_y[0]) / (D1_trajectory[n_points, 0] - D1_trajectory[1, 0])
    #
    # # x_space_final = sum(D1_trajectory[-n_points:, 1] - D1_trajectory[-n_points-1:-1, 1])
    # # y_space_final = sum(D1_trajectory[-n_points:, 2] - D1_trajectory[-n_points-1:-1, 2])
    # # x_vel_final = x_space_final / (D1_trajectory[-1, 0] - D1_trajectory[-n_points-1, 0])
    # # y_vel_final = y_space_final / (D1_trajectory[-1, 0] - D1_trajectory[-n_points-1, 0])
    # # x_acc_final = x_vel_final / (D1_trajectory[-1, 0] - D1_trajectory[-n_points-1, 0])
    # # y_acc_final = y_vel_final / (D1_trajectory[-1, 0] - D1_trajectory[-n_points-1, 0])
    #
    # # D1_avg_acceleration.append(np.array([x_acc_final/x_acc_init, y_acc_final/y_acc_init]))

    # timestep rate
    delta_t = D1_trajectory[1:, 0] - D1_trajectory[:-1, 0]
    normalized_time = D1_trajectory[1:, 0] / D1_trajectory[-1, 0]
    mean_rate, std_rate = extract_perc_values(delta_t, normalized_time, perc_points, range=0.05)
    D1_mean_rate_list.append(mean_rate)
    # D1_std_rate_list.append(std_rate)

    # timestep rate normalized over x
    normalized_x = D1_trajectory[1:, 1] / 304
    mean_rate_x, std_rate_x = extract_perc_values(delta_t, normalized_x, perc_points, range=0.05)
    D1_mean_rate_x_list.append(mean_rate_x)
    # D1_std_rate_x_list.append(std_rate_x)

    # average horizontal velocity
    D1_x_vel = (D1_trajectory[1:, 1] - D1_trajectory[:-1, 1]) / (D1_trajectory[1:, 0] - D1_trajectory[:-1, 0])
    D1_x_vel[np.abs(D1_x_vel) == np.inf] = np.nan
    mean_rate_x, std_rate_x = extract_perc_values(D1_x_vel, normalized_time, perc_points, range=0.05)
    D1_mean_x_vel_list.append(mean_rate_x)
    # D1_std_x_vel_list.append(std_rate_x)

    # acceleration at bounce
    # find the first time the derivative gets positive after being negative
    found = False
    D1_moving_avg_traj = moving_avg(D1_trajectory, n_mov_avg)
    for i in range(1, len(D1_moving_avg_traj)-n_points-1):
        if (D1_moving_avg_traj[i, 2] - D1_moving_avg_traj[i-1, 2] < 0) and (D1_moving_avg_traj[i+1, 2] - D1_moving_avg_traj[i, 2] > 0):
            bouncing_index = i
            found = True
            break
    if found:
        pos_before_bouncing = sum(D1_moving_avg_traj[bouncing_index-n_points+1:bouncing_index+1, 1:3] - D1_moving_avg_traj[bouncing_index-n_points:bouncing_index, 1:3])
        delta_t_pre = D1_moving_avg_traj[bouncing_index, 0] - D1_moving_avg_traj[bouncing_index - n_points, 0]
        vel_pre = np.abs(pos_before_bouncing/delta_t_pre)
        pos_after_bouncing = sum(D1_moving_avg_traj[bouncing_index+1:min(bouncing_index + n_points + 1, len(D1_moving_avg_traj)), 1:3] - D1_moving_avg_traj[bouncing_index:min(bouncing_index+n_points, len(D1_moving_avg_traj)-1), 1:3])
        delta_t_post = D1_moving_avg_traj[min(bouncing_index+n_points, len(D1_moving_avg_traj)-1), 0] - D1_moving_avg_traj[bouncing_index, 0]
        vel_post = np.abs(pos_after_bouncing/delta_t_post)
        D1_restitution_coeffs.append(vel_post/vel_pre)
        D1_avg_x_acceleration.append((vel_post[0] - vel_pre[0]) / delta_t_post)
        D1_avg_y_acceleration.append((vel_post[1] - vel_pre[1]) / delta_t_post)
        # bouncing coordinate (table height)
        D1_bouncing_coordinates.append(D1_moving_avg_traj[i, 2])
    else:
        D1_restitution_coeffs.append(np.array([np.nan, np.nan]))
        D1_bouncing_coordinates.append(np.nan)

D1_mean_spatial_delta   = np.nanmean(D1_spatial_delta)
D1_init                 = np.array(D1_init)
D1_init_vel             = np.array(D1_init_vel)
D1_mean_rate            = np.nanmean(np.array(D1_mean_rate_list), axis=0)
D1_std_rate             = np.nanstd(np.array(D1_mean_rate_list), axis=0)
D1_mean_rate_x          = np.nanmean(np.array(D1_mean_rate_x_list), axis=0)
D1_std_rate_x           = np.nanstd(np.array(D1_mean_rate_x_list), axis=0)
D1_mean_x_vel           = np.nanmean(np.array(D1_mean_x_vel_list), axis=0)
D1_std_x_vel            = np.nanstd(np.array(D1_mean_x_vel_list), axis=0)
D1_restitution_coeffs   = np.array(D1_restitution_coeffs)
D1_bouncing_coordinates = np.array(D1_bouncing_coordinates)


################################################ DATASET 2 PROCESSING ################################################

D2_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), D2_folder)
D2_files = sorted([traj for traj in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), D2_path)) if traj.endswith('.txt')])
D2_trajectories, D2_spatial_delta = [], []
D2_init, D2_init_vel, D2_angle, D2_mean_x_vel_list, D2_std_x_vel_list = [], [], [], [], []
D2_avg_acceleration, D2_accelerations, D2_avg_x_acceleration, D2_avg_y_acceleration = [], [], [], []
D2_mean_rate_list, D2_std_rate_list, D2_mean_rate_x_list, D2_std_rate_x_list = [], [], [], []
D2_restitution_coeffs = []
D2_bouncing_coordinates = []
for t in tqdm(D2_files, "Loading dataset_2 trajectories..."):

    # load data
    D2_trajectory = np.loadtxt(os.path.join(os.path.dirname(os.path.abspath(__file__)), D2_folder, t), delimiter=",")[:, :3]
    D2_trajectory[:, 0] = np.cumsum(D2_trajectory[:, 0])
    if D2_folder[0:7] != "asynch_" : D2_trajectory[:, 1] = 304 - D2_trajectory[:, 1]    # flip x-coordinate
    D2_trajectory[:, 2] = 240 - D2_trajectory[:, 2]    # flip y-coordinate
    D2_trajectories.append(D2_trajectory)

    # spatial delta
    deltas = D2_trajectory[1:, 1:3] - D2_trajectory[:-1, 1:3]
    D2_spatial_delta.append(np.mean(np.sqrt(deltas[:,0]**2 + deltas[:,1]**2)))

    # initial coordinates
    D2_init.append(np.array([D2_trajectory[0, 1], D2_trajectory[0, 2]]))

    # initial xy velocity and angle
    vel_init = sum(D2_trajectory[1:n_points+1, 1:3] - D2_trajectory[:n_points, 1:3]) / D2_trajectory[n_points, 0]
    # x_acc_init = np.abs(x_vel_init) / D2_trajectory[n_points, 0]
    # y_acc_init = np.abs(y_vel_init) / D2_trajectory[n_points, 0]
    D2_init_vel.append(vel_init)
    D2_angle.append(np.arctan2(vel_init[1], vel_init[0]))

    # # average acceleration
    # x_space_init2 = sum(D2_trajectory[n_points+1:2*n_points+1, 1] - D2_trajectory[n_points:2*n_points, 1])
    # y_space_init2 = sum(D2_trajectory[n_points+1:2*n_points+1, 2] - D2_trajectory[n_points:2*n_points, 2])
    # x_vel_init2 = np.abs(x_space_init2) / (D2_trajectory[2*n_points, 0] - D2_trajectory[n_points, 0])
    # y_vel_init2 = np.abs(y_space_init2) / (D2_trajectory[2*n_points, 0] - D2_trajectory[n_points, 0])
    # x_accel_init = (x_vel_init2-x_vel_init)/(D2_trajectory[2*n_points, 0] - D2_trajectory[n_points, 0])
    # y_accel_init = (y_vel_init2-y_vel_init)/(D2_trajectory[2*n_points, 0] - D2_trajectory[n_points, 0])
    # accel_init = np.sqrt(x_accel_init**2 + y_accel_init**2)
    #
    # x_space_final = sum(D2_trajectory[-n_points:, 1] - D2_trajectory[-n_points-1:-1, 1])
    # y_space_final = sum(D2_trajectory[-n_points:, 2] - D2_trajectory[-n_points-1:-1, 2])
    # x_vel_final = x_space_final / (D2_trajectory[-1, 0] - D2_trajectory[-n_points-1, 0])
    # y_vel_final = y_space_final / (D2_trajectory[-1, 0] - D2_trajectory[-n_points-1, 0])
    # x_space_final2 = sum(D2_trajectory[-2*n_points:-n_points, 1] - D2_trajectory[-2*n_points-1:-n_points-1, 1])
    # y_space_final2 = sum(D2_trajectory[-2*n_points:-n_points, 2] - D2_trajectory[-2*n_points-1:-n_points-1, 2])
    # x_vel_final2 = np.abs(x_space_final2) / (D2_trajectory[-n_points-1, 0] - D2_trajectory[-2*n_points-1, 0])
    # y_vel_final2 = np.abs(y_space_final2) / (D2_trajectory[-n_points-1, 0] - D2_trajectory[-2*n_points-1, 0])
    # x_accel_final = (x_vel_final-x_vel_final2)/(D2_trajectory[-1, 0] - D2_trajectory[-n_points-1, 0])
    # y_accel_final = (y_vel_final-y_vel_final2)/(D2_trajectory[-1, 0] - D2_trajectory[-n_points-1, 0])
    # accel_final = np.sqrt(x_accel_final**2 + y_accel_final**2)
    # D2_accelerations.append(np.array([x_accel_final/x_accel_init, y_accel_final/y_accel_init]))
    #
    # # velocities_x = (D2_trajectory[1:n_points+1, 1] - D2_trajectory[:n_points, 1]) / (D2_trajectory[1:n_points+1, 0] - D2_trajectory[:n_points, 0])
    # # velocities_y = (D2_trajectory[1:n_points + 1, 2] - D2_trajectory[:n_points, 2]) / (D2_trajectory[1:n_points+1, 0] - D2_trajectory[:n_points, 0])
    # # x_accel_init = (velocities_x[-1] - velocities_x[0]) / (D2_trajectory[n_points, 0] - D2_trajectory[1, 0])
    # # y_accel_init = (velocities_y[-1] - velocities_y[0]) / (D2_trajectory[n_points, 0] - D2_trajectory[1, 0])
    #
    # # x_space_final = sum(D2_trajectory[-n_points:, 1] - D2_trajectory[-n_points-1:-1, 1])
    # # y_space_final = sum(D2_trajectory[-n_points:, 2] - D2_trajectory[-n_points-1:-1, 2])
    # # x_vel_final = x_space_final / (D2_trajectory[-1, 0] - D2_trajectory[-n_points-1, 0])
    # # y_vel_final = y_space_final / (D2_trajectory[-1, 0] - D2_trajectory[-n_points-1, 0])
    # # x_acc_final = x_vel_final / (D2_trajectory[-1, 0] - D2_trajectory[-n_points-1, 0])
    # # y_acc_final = y_vel_final / (D2_trajectory[-1, 0] - D2_trajectory[-n_points-1, 0])
    #
    # # D2_avg_acceleration.append(np.array([x_acc_final/x_acc_init, y_acc_final/y_acc_init]))

    # timestep rate
    delta_t = D2_trajectory[1:, 0] - D2_trajectory[:-1, 0]
    normalized_time = D2_trajectory[1:, 0] / D2_trajectory[-1, 0]
    mean_rate, std_rate = extract_perc_values(delta_t, normalized_time, perc_points, range=0.05)
    D2_mean_rate_list.append(mean_rate)
    # D2_std_rate_list.append(std_rate)

    # timestep rate normalized over x
    normalized_x = D2_trajectory[1:, 1] / 304
    mean_rate_x, std_rate_x = extract_perc_values(delta_t, normalized_x, perc_points, range=0.05)
    D2_mean_rate_x_list.append(mean_rate_x)
    # D2_std_rate_x_list.append(std_rate_x)

    # average horizontal velocity
    D2_x_vel = (D2_trajectory[1:, 1] - D2_trajectory[:-1, 1]) / (D2_trajectory[1:, 0] - D2_trajectory[:-1, 0])
    D2_x_vel[np.abs(D2_x_vel) == np.inf] = np.nan
    mean_rate_x, std_rate_x = extract_perc_values(D2_x_vel, normalized_time, perc_points, range=0.05)
    D2_mean_x_vel_list.append(mean_rate_x)
    # D2_std_x_vel_list.append(std_rate_x)

    # acceleration at bounce
    # find the first time the derivative gets positive after being negative
    found = False
    D2_moving_avg_traj = moving_avg(D2_trajectory, n_mov_avg)
    for i in range(1, len(D2_moving_avg_traj)-n_points-1):
        if (D2_moving_avg_traj[i, 2] - D2_moving_avg_traj[i-1, 2] < 0) and (D2_moving_avg_traj[i+1, 2] - D2_moving_avg_traj[i, 2] > 0):
            bouncing_index = i
            found = True
            break
    if found:
        pos_before_bouncing = sum(D2_moving_avg_traj[bouncing_index-n_points+1:bouncing_index+1, 1:3] - D2_moving_avg_traj[bouncing_index-n_points:bouncing_index, 1:3])
        delta_t_pre = D2_moving_avg_traj[bouncing_index, 0] - D2_moving_avg_traj[bouncing_index - n_points, 0]
        vel_pre = np.abs(pos_before_bouncing/delta_t_pre)
        pos_after_bouncing = sum(D2_moving_avg_traj[bouncing_index+1:min(bouncing_index + n_points + 1, len(D2_moving_avg_traj)), 1:3] - D2_moving_avg_traj[bouncing_index:min(bouncing_index+n_points, len(D2_moving_avg_traj)-1), 1:3])
        delta_t_post = D2_moving_avg_traj[min(bouncing_index+n_points, len(D2_moving_avg_traj)-1), 0] - D2_moving_avg_traj[bouncing_index, 0]
        vel_post = np.abs(pos_after_bouncing/delta_t_post)
        D2_restitution_coeffs.append(vel_post/vel_pre)
        D2_avg_x_acceleration.append((vel_post[0] - vel_pre[0]) / delta_t_post)
        D2_avg_y_acceleration.append((vel_post[1] - vel_pre[1]) / delta_t_post)
        # bouncing coordinate (table height)
        D2_bouncing_coordinates.append(D2_moving_avg_traj[i, 2])
    else:
        D2_restitution_coeffs.append(np.array([np.nan, np.nan]))
        D2_bouncing_coordinates.append(np.nan)

D2_mean_spatial_delta   = np.nanmean(D2_spatial_delta)
D2_init                 = np.array(D2_init)
D2_init_vel             = np.array(D2_init_vel)
D2_mean_rate            = np.nanmean(np.array(D2_mean_rate_list), axis=0)
D2_std_rate             = np.nanstd(np.array(D2_mean_rate_list), axis=0)
D2_mean_rate_x          = np.nanmean(np.array(D2_mean_rate_x_list), axis=0)
D2_std_rate_x           = np.nanstd(np.array(D2_mean_rate_x_list), axis=0)
D2_mean_x_vel           = np.nanmean(np.array(D2_mean_x_vel_list), axis=0)
D2_std_x_vel            = np.nanstd(np.array(D2_mean_x_vel_list), axis=0)
D2_restitution_coeffs   = np.array(D2_restitution_coeffs)
D2_bouncing_coordinates = np.array(D2_bouncing_coordinates)


######################################################## PLOTS ########################################################

# Spatial Delta
fig1 = plt.figure(1)
ticks = [label1, label2]
spatial_data = [D1_spatial_delta, D2_spatial_delta]
plt.boxplot(spatial_data, vert=True, patch_artist=True, labels=ticks)
plt.ylabel(r' Spatial delta [pxl]')
plt.grid()
plt.show()

# Initial Position
fig2 = plt.figure(2)
plt.scatter(D2_init[:,0], D2_init[:,1], color='#ED0DD9', alpha=0.4, s=100, label=label2)
plt.scatter(D1_init[:,0], D1_init[:,1], color='darkcyan', alpha=0.4, s=100, label=label1)
plt.title(r'Initial Position')
plt.xlabel(r'init X [pxl]')
plt.ylabel(r'init Y [pxl]')
plt.grid()
plt.legend()
plt.show()

# Initial Velocity
fig3 = plt.figure(3)
plt.scatter(D2_init_vel[:,0], D2_init_vel[:,1], color='#ED0DD9', alpha=0.4, s=100, label=label2)
plt.scatter(D1_init_vel[:,0], D1_init_vel[:,1], color='darkcyan', alpha=0.4, s=100, label=label1)
plt.title(r'Initial Velocity (first '+ str(n_points) + ' points)')
plt.xlabel(r'init X vel [pxl/s]')
plt.ylabel(r'init Y vel [pxl/s]')
plt.grid()
plt.legend()
plt.show()

# # Mean acceleration
# fig4 = plt.figure(4)
# ticks = [label1, label2]
# accel_data = [D1_avg_acceleration, D2_avg_acceleration]
# plt.boxplot(accel_data, vert=True, patch_artist=True, labels=ticks)
# plt.ylabel(r' Average acceleration [pxl]')
# plt.grid()
# plt.show()

# Delta_t
fig5 = plt.figure(5)
plt.gca()
plt.plot((perc_points*100).astype(int), D1_mean_rate*1000, color='darkcyan', label=label1)
plt.fill_between((perc_points*100).astype(int), y1=(D1_mean_rate-D1_std_rate)*1000, y2=(D1_mean_rate+D1_std_rate)*1000, alpha=0.3, color='darkcyan')
plt.plot((perc_points*100).astype(int), D2_mean_rate*1000, color='#ED0DD9', label=label2)
plt.fill_between((perc_points*100).astype(int), y1=(D2_mean_rate-D2_std_rate)*1000, y2=(D2_mean_rate+D2_std_rate)*1000, alpha=0.3, color='#ED0DD9')
plt.xlabel('Trajectory %')
plt.ylabel('Average rate [ms]')
plt.grid()
plt.legend()
plt.show()

# Delta_t horizontal
fig6 = plt.figure(6)
plt.gca()
plt.plot((perc_points*100).astype(int), D1_mean_rate_x*1000, color='darkcyan', label=label1)
plt.fill_between((perc_points*100).astype(int), y1=(D1_mean_rate_x-D1_std_rate_x)*1000, y2=(D1_mean_rate_x+D1_std_rate_x)*1000, alpha=0.3, color='darkcyan')
plt.plot((perc_points*100).astype(int), D2_mean_rate_x*1000, color='#ED0DD9', label=label2)
plt.fill_between((perc_points*100).astype(int), y1=(D2_mean_rate_x-D2_std_rate_x)*1000, y2=(D2_mean_rate_x+D2_std_rate_x)*1000, alpha=0.3, color='#ED0DD9')
plt.xlabel('Trajectory %')
plt.ylabel('Average horizontal rate [ms]')
plt.grid()
plt.legend()
plt.show()

# average horizontal velocity
fig10 = plt.figure(10)
plt.gca()
plt.plot((perc_points*100).astype(int), D1_mean_x_vel, color='darkcyan', label=label1)
plt.fill_between((perc_points*100).astype(int), y1=(D1_mean_x_vel-D1_std_x_vel), y2=(D1_mean_x_vel+D1_std_x_vel), alpha=0.3, color='darkcyan')
plt.plot((perc_points*100).astype(int), D2_mean_x_vel, color='#ED0DD9', label=label2)
plt.fill_between((perc_points*100).astype(int), y1=(D2_mean_x_vel-D2_std_x_vel), y2=(D2_mean_x_vel+D2_std_x_vel), alpha=0.3, color='#ED0DD9')
plt.xlabel('Trajectory %')
plt.ylabel('Average horizontal velocity [pxl/s]')
plt.grid()
plt.legend()
plt.show()

# Coefficient Of Restitution
fig7 = plt.figure(7)
plt.scatter(D2_restitution_coeffs[:,0], D2_restitution_coeffs[:,1], color='#ED0DD9', alpha=0.4, s=100, label=label2)
plt.scatter(D1_restitution_coeffs[:,0], D1_restitution_coeffs[:,1], color='darkcyan', alpha=0.4, s=100, label=label1)
plt.xlabel(r' COR along X')
plt.ylabel(r'COR along Y')
plt.grid()
plt.legend()
plt.show()

# Bouncing height
fig8 = plt.figure(8)
ticks = [label1, label2]
bouncing_height = [D1_bouncing_coordinates, D2_bouncing_coordinates[~np.isnan(D2_bouncing_coordinates)]]
plt.boxplot(bouncing_height, vert=True, patch_artist=True, labels=ticks)
plt.ylabel(r' Bouncing height [pxl]')
plt.grid()
plt.show()


# All trajectories
fig9 = plt.figure(9)
for s in D2_trajectories: plt.plot(s[:, 1], s[:, 2], color='#ED0DD9', alpha=0.4)
for r in D1_trajectories: plt.plot(r[:, 1], r[:, 2], color='darkcyan', alpha=0.4)
plt.xlabel('x coord. [pxl]')
plt.ylabel('y coord. [pxl]')
plt.grid()
plt.legend()
plt.show()

pass

# # Real Delta_t
# fig4, ax4 = plt.subplots()
# plt.gca()
# plt.boxplot(real_mean_rate_list[~np.isnan(real_mean_rate_list).any(axis=1), :]*1000, vert=True, patch_artist=True, labels=(perc_points*100).astype(int))
# plt.title(r'Delta T (real dataset)')
# plt.xlabel('Percentage [%]')
# plt.ylabel('[ms]')
# plt.grid()
# plt.show()
#
# # Simulated Delta_t
# fig5, ax5 = plt.subplots()
# plt.gca()
# plt.boxplot(sim_mean_rate_list[~np.isnan(sim_mean_rate_list).any(axis=1), :]*1000, vert=True, patch_artist=True, labels=(perc_points*100).astype(int))
# plt.title(r'Delta T (simulated)')
# plt.xlabel('Percentage [%]')
# plt.ylabel('[ms]')
# plt.grid()
# plt.show()
#
# # Real Delta_t X
# fig6, ax6 = plt.subplots()
# plt.gca()
# plt.boxplot(real_mean_rate_x_list[~np.isnan(real_mean_rate_x_list).any(axis=1), :]*1000, vert=True, patch_artist=True, labels=(perc_points*100).astype(int))
# plt.title(r'Horizontal Delta T (real dataset)')
# plt.xlabel('Percentage [%]')
# plt.ylabel('[ms]')
# plt.grid()
# plt.show()
#
# # Simulated Delta_t X
# fig7, ax7 = plt.subplots()
# plt.gca()
# plt.boxplot(sim_mean_rate_x_list[~np.isnan(sim_mean_rate_x_list).any(axis=1), :]*1000, vert=True, patch_artist=True, labels=(perc_points*100).astype(int))
# plt.title(r'Horizontal Delta T (simulated)')
# plt.xlabel('Percentage [%]')
# plt.ylabel('[ms]')
# plt.grid()
# plt.show()
