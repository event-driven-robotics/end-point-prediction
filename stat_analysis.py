import os, numpy as np
from tqdm import tqdm
from matplotlib import pyplot as plt

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

real_folder = "real_trajectories"
real_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), real_folder)
real_trajs = sorted([traj for traj in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), real_path)) if traj.endswith('.txt')])
real_spatial_delta = []
real_x_init, real_y_init = [], []
real_x_vel, real_y_vel, real_angle = [], [], []
real_mean_rate_list, real_std_rate_list = [], []
real_mean_rate_x_list, real_std_rate_x_list = [], []
real_x_restitution_coeffs, real_y_restitution_coeffs = [], []
real_bouncing_coordinates = []
for t in tqdm(real_trajs, "Loading real trajectories..."):
    real_trajectory = np.loadtxt(os.path.join(os.path.dirname(os.path.abspath(__file__)), real_folder, t), delimiter=",")
    real_trajectory[:, 0] = np.cumsum(real_trajectory[:, 0])
    real_trajectory[:, 1] = 304 - real_trajectory[:, 1]    # flip y-coordinate
    real_trajectory[:, 2] = 240 - real_trajectory[:, 2]    # flip y-coordinate
    # spatial delta
    real_spatial_x = real_trajectory[1:, 1] - real_trajectory[:-1, 1]
    real_spatial_y = real_trajectory[1:, 2] - real_trajectory[:-1, 2]
    real_spatial_delta.append(np.mean(np.sqrt(real_spatial_x**2 + real_spatial_y**2)))
    # initial coordinates
    real_x_init.append(real_trajectory[0, 1])
    real_y_init.append(real_trajectory[0, 2])
    # initial xy velocity and angle
    n_points = 5
    x_space = sum(real_trajectory[1:n_points+1, 1] - real_trajectory[:n_points, 1])
    y_space = sum(real_trajectory[1:n_points+1, 2] - real_trajectory[:n_points, 2])
    real_x_vel.append(np.abs(x_space) / real_trajectory[n_points, 0])
    real_y_vel.append(np.abs(y_space) / real_trajectory[n_points, 0])
    real_angle.append(np.arctan2(real_y_vel, real_x_vel))
    # timestep rate
    delta_t = real_trajectory[1:, 0] - real_trajectory[:-1, 0] # time difference between two subsequent tracked positions
    perc_points = np.arange(0.0, 1.1, 0.1)
    normalized_time = real_trajectory[1:, 0]/real_trajectory[-1, 0] # normalized timestamps
    mean_rate, std_rate = extract_perc_values(delta_t, normalized_time, perc_points, range=0.05)
    real_mean_rate_list.append(mean_rate)
    real_std_rate_list.append(std_rate)
    # timestep rate normalized over x
    normalized_x = real_trajectory[1:, 1]/304
    mean_rate_x, std_rate_x = extract_perc_values(delta_t, normalized_x, perc_points, range=0.05)
    real_mean_rate_x_list.append(mean_rate_x)
    real_std_rate_x_list.append(std_rate_x)
    # acceleration at bounce
    # find the first time the derivative gets positive after being negative
    found = False
    for i in range(2, len(real_trajectory)-n_points-1):
        if (real_trajectory[i, 2] - real_trajectory[i-2, 2] < 0) and (real_trajectory[i+1, 2] - real_trajectory[i, 2] > 0):
            bouncing_index = i
            found = True
            break
    if found:
        bouncing_index2 = np.argmin(real_trajectory[:, 2])
        x_before_bouncing = sum(real_trajectory[bouncing_index-n_points+1:bouncing_index+1, 1] - real_trajectory[bouncing_index-n_points:bouncing_index, 1])
        y_before_bouncing = sum(real_trajectory[bouncing_index-n_points+1:bouncing_index+1, 2] - real_trajectory[bouncing_index-n_points:bouncing_index, 2])
        delta_t_pre = real_trajectory[bouncing_index, 0] - real_trajectory[bouncing_index - n_points, 0]
        x_vel_pre = np.abs(x_before_bouncing/delta_t_pre)
        y_vel_pre = np.abs(y_before_bouncing/delta_t_pre)
        x_after_bouncing = sum(real_trajectory[bouncing_index+1:bouncing_index+n_points+1, 1] - real_trajectory[bouncing_index:bouncing_index+n_points, 1])
        y_after_bouncing = sum(real_trajectory[bouncing_index+1:bouncing_index+n_points+1, 2] - real_trajectory[bouncing_index:bouncing_index+n_points, 2])
        delta_t_post = real_trajectory[bouncing_index+n_points, 0] - real_trajectory[bouncing_index, 0]
        x_vel_post = np.abs(x_after_bouncing/delta_t_post)
        y_vel_post = np.abs(y_after_bouncing/delta_t_post)
        real_x_restitution_coeffs.append(x_vel_post/x_vel_pre)
        real_y_restitution_coeffs.append(y_vel_post/y_vel_pre)
        # bouncing coordinate (table height)
        real_bouncing_coordinates.append(real_trajectory[i, 2])
    else:
        real_x_restitution_coeffs.append(np.nan)
        real_y_restitution_coeffs.append(np.nan)
        real_bouncing_coordinates.append(np.nan)

real_mean_spatial_delta = np.mean(real_spatial_delta)
# real_mean_rate_dataset = np.mean(np.array(real_mean_rate_list), axis=0)
# real_std_rate_dataset = np.mean(np.array(real_std_rate_list), axis=0)
real_mean_rate_list = np.array(real_mean_rate_list)
real_mean_rate_x_list = np.array(real_mean_rate_x_list)
real_bouncing_coordinates = np.array(real_bouncing_coordinates)
print("Bouncing height for real trajs:   mean = ", np.nanmean(real_bouncing_coordinates), "     std = " , np.nanstd(real_bouncing_coordinates))

gt_folder = "asynch_trajectories"
gt_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), gt_folder)
gt_trajs = sorted([traj for traj in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), gt_path)) if traj.endswith('.txt')])
gt_spatial_delta = []
gt_x_init, gt_y_init = [], []
gt_x_vel, gt_y_vel, gt_angle = [], [], []
gt_mean_rate_list, gt_std_rate_list = [], []
gt_mean_rate_x_list, gt_std_rate_x_list = [], []
gt_x_restitution_coeffs, gt_y_restitution_coeffs = [], []
gt_bouncing_coordinates = []
for t in tqdm(gt_trajs, "Loading gt trajectories..."):
    gt_trajectory = np.loadtxt(os.path.join(os.path.dirname(os.path.abspath(__file__)), gt_folder, t), delimiter=",")
    gt_trajectory[:, 1] = 304 - gt_trajectory[:, 1]    # flip y-coordinate
    gt_trajectory[:, 2] = 240 - gt_trajectory[:, 2]    # flip y-coordinate
    # spatial delta
    gt_spatial_x = gt_trajectory[1:, 1] - gt_trajectory[:-1, 1]
    gt_spatial_y = gt_trajectory[1:, 2] - gt_trajectory[:-1, 2]
    gt_spatial_delta.append(np.mean(np.sqrt(gt_spatial_x**2 + gt_spatial_y**2)))
    # initial coordinates
    gt_x_init.append(gt_trajectory[0, 1])
    gt_y_init.append(gt_trajectory[0, 2])
    # initial xy velocity and angle
    n_points = 5
    x_space = sum(gt_trajectory[1:n_points+1, 1] - gt_trajectory[:n_points, 1])
    y_space = sum(gt_trajectory[1:n_points+1, 2] - gt_trajectory[:n_points, 2])
    gt_x_vel.append(np.abs(x_space) / gt_trajectory[n_points, 0])
    gt_y_vel.append(np.abs(y_space) / gt_trajectory[n_points, 0])
    gt_angle.append(np.arctan2(gt_y_vel, gt_x_vel))
    # timestep rate
    delta_t = gt_trajectory[1:, 0] - gt_trajectory[:-1, 0]
    perc_points = np.arange(0.0, 1.1, 0.1)
    normalized_time = gt_trajectory[1:, 0]/gt_trajectory[-1, 0]
    mean_rate, std_rate = extract_perc_values(delta_t, normalized_time, perc_points, range=0.05)
    gt_mean_rate_list.append(mean_rate)
    gt_std_rate_list.append(std_rate)
    # timestep rate normalized over x
    normalized_x = gt_trajectory[1:, 1]/304
    mean_rate_x , std_rate_x = extract_perc_values(delta_t, normalized_x, perc_points, range=0.05)
    gt_mean_rate_x_list.append(mean_rate_x)
    gt_std_rate_x_list.append(std_rate_x)
    # acceleration at bounce
    # find the first time the derivative gets positive after being negative
    found = False
    for i in range(2, len(gt_trajectory)-n_points-1):
        if (gt_trajectory[i, 2] - gt_trajectory[i-2, 2] < 0) and (gt_trajectory[i+1, 2] - gt_trajectory[i, 2] > 0):
            bouncing_index = i
            found = True
            break
    if found:
        bouncing_index2 = np.argmin(gt_trajectory[:, 2])
        x_before_bouncing = sum(gt_trajectory[bouncing_index-n_points+1:bouncing_index+1, 1] - gt_trajectory[bouncing_index-n_points:bouncing_index, 1])
        y_before_bouncing = sum(gt_trajectory[bouncing_index-n_points+1:bouncing_index+1, 2] - gt_trajectory[bouncing_index-n_points:bouncing_index, 2])
        delta_t_pre = gt_trajectory[bouncing_index, 0] - gt_trajectory[bouncing_index - n_points, 0]
        x_vel_pre = np.abs(x_before_bouncing/delta_t_pre)
        y_vel_pre = np.abs(y_before_bouncing/delta_t_pre)
        x_after_bouncing = sum(gt_trajectory[bouncing_index+1:bouncing_index+n_points+1, 1] - gt_trajectory[bouncing_index:bouncing_index+n_points, 1])
        y_after_bouncing = sum(gt_trajectory[bouncing_index+1:bouncing_index+n_points+1, 2] - gt_trajectory[bouncing_index:bouncing_index+n_points, 2])
        delta_t_post = gt_trajectory[bouncing_index+n_points, 0] - gt_trajectory[bouncing_index, 0]
        x_vel_post = np.abs(x_after_bouncing/delta_t_post)
        y_vel_post = np.abs(y_after_bouncing/delta_t_post)
        gt_x_restitution_coeffs.append(x_vel_post/x_vel_pre)
        gt_y_restitution_coeffs.append(y_vel_post/y_vel_pre)
        # bouncing coordinate (table height)
        gt_bouncing_coordinates.append(gt_trajectory[i, 2])
    else:
        gt_x_restitution_coeffs.append(np.nan)
        gt_y_restitution_coeffs.append(np.nan)
        gt_bouncing_coordinates.append(np.nan)

gt_mean_spatial_delta = np.mean(gt_spatial_delta)
# gt_mean_rate_dataset = np.mean(np.array(gt_mean_rate_list), axis=0)
# gt_std_rate_dataset = np.mean(np.array(gt_std_rate_list), axis=0)
gt_mean_rate_list = np.array(gt_mean_rate_list)
gt_mean_rate_x_list = np.array(gt_mean_rate_x_list)
gt_bouncing_coordinates = np.array(gt_bouncing_coordinates)
print("Bouncing height for simulated trajs:   mean = ", np.nanmean(gt_bouncing_coordinates), "     std = " , np.nanstd(gt_bouncing_coordinates))

# Initial position
fig1 = plt.figure(1)
plt.scatter(real_x_init, real_y_init, color='darkcyan', alpha=0.4, s=100, label='real trajs.')
plt.scatter(gt_x_init, gt_y_init, color='#ED0DD9', alpha=0.4, s=100, label='ground truth')
plt.title(r'Initial Position')
plt.xlabel(r'init x position [pixels]')
plt.ylabel(r'init y position [pixels]')
plt.grid()
plt.legend()
plt.show()

# Initial velocity
fig2 = plt.figure(2)
plt.scatter(real_x_vel, real_y_vel, color='darkcyan', alpha=0.4, s=100, label='real trajs.')
plt.scatter(gt_x_vel, gt_y_vel, color='#ED0DD9', alpha=0.4, s=100, label='ground truth')
plt.title(r'Initial Velocity (first 5 points)')
plt.xlabel(r'init x velocity [pixels/s]')
plt.ylabel(r'init y velocity [pixels/s]')
plt.grid()
plt.legend()
plt.show()

# Spatial delta
fig3, ax3 = plt.subplots()
ticks = ['real', 'ground truth']
all_data = [real_spatial_delta, gt_spatial_delta]
plt.boxplot(all_data, vert=True, patch_artist=True, labels=ticks)
plt.title(r'Spatial delta')
plt.ylabel(r'[pixels]')
plt.grid()
plt.show()

# delta t
fig4, ax4 = plt.subplots()
plt.gca()
plt.boxplot(real_mean_rate_list[~np.isnan(real_mean_rate_list).any(axis=1), :]*1000, vert=True, patch_artist=True, labels=(perc_points*100).astype(int))
plt.title(r'Delta T (real dataset)')
plt.xlabel('Percentage [%]')
plt.ylabel('[ms]')
plt.grid()
plt.show()

# delta t x
fig5, ax5 = plt.subplots()
plt.gca()
plt.boxplot(real_mean_rate_x_list[~np.isnan(real_mean_rate_x_list).any(axis=1), :]*1000, vert=True, patch_artist=True, labels=(perc_points*100).astype(int))
plt.title(r'Horizontal Delta T (real dataset)')
plt.xlabel('Percentage [%]')
plt.ylabel('[ms]')
plt.grid()
plt.show()

# delta t
fig6, ax6 = plt.subplots()
plt.gca()
plt.boxplot(gt_mean_rate_list[~np.isnan(gt_mean_rate_list).any(axis=1), :]*1000, vert=True, patch_artist=True, labels=(perc_points*100).astype(int))
plt.title(r'Delta T (ground truth)')
plt.xlabel('Percentage [%]')
plt.ylabel('[ms]')
plt.grid()
plt.show()

# delta t x
fig7, ax7 = plt.subplots()
plt.gca()
plt.boxplot(gt_mean_rate_x_list[~np.isnan(gt_mean_rate_x_list).any(axis=1), :]*1000, vert=True, patch_artist=True, labels=(perc_points*100).astype(int))
plt.title(r'Horizontal Delta T (ground truth)')
plt.xlabel('Percentage [%]')
plt.ylabel('[ms]')
plt.grid()
plt.show()

# restitution coeff
fig8 = plt.figure()
plt.scatter(gt_x_restitution_coeffs, gt_y_restitution_coeffs, color='darkcyan', alpha=0.4, s=100, label='ground truth')
plt.scatter(real_x_restitution_coeffs, real_y_restitution_coeffs, color='magenta', alpha=0.4, s=100, label='real trajs.')
plt.title(r'Restitution Coefficients')
plt.xlabel(r'x')
plt.ylabel(r'y')
plt.grid()
plt.legend()
plt.show()