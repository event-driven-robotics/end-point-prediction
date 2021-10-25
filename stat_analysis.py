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



n_points = 5
n_mov_avg = 5

################################################## REAL TRAJECTORIES ##################################################
real_folder = "real_trajectories"
real_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), real_folder)
real_trajs = sorted([traj for traj in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), real_path)) if traj.endswith('.txt')])
real_spatial_delta = []
real_x_init, real_y_init = [], []
real_x_vel, real_y_vel, real_angle = [], [], []
real_avg_acceleration = []
real_avg_x_acceleration, real_avg_y_acceleration = [], []
real_mean_rate_list, real_std_rate_list = [], []
real_mean_rate_x_list, real_std_rate_x_list = [], []
real_x_restitution_coeffs, real_y_restitution_coeffs = [], []
real_bouncing_coordinates = []
for t in tqdm(real_trajs, "Loading real trajectories..."):
    real_trajectory = np.loadtxt(os.path.join(os.path.dirname(os.path.abspath(__file__)), real_folder, t), delimiter=",")
    real_trajectory[:, 0] = np.cumsum(real_trajectory[:, 0])
    real_trajectory[:, 1] = 304 - real_trajectory[:, 1]    # flip x-coordinate
    real_trajectory[:, 2] = 240 - real_trajectory[:, 2]    # flip y-coordinate
    # spatial delta
    real_spatial_x = real_trajectory[1:, 1] - real_trajectory[:-1, 1]
    real_spatial_y = real_trajectory[1:, 2] - real_trajectory[:-1, 2]
    real_spatial_delta.append(np.mean(np.sqrt(real_spatial_x**2 + real_spatial_y**2)))
    # initial coordinates
    real_x_init.append(real_trajectory[0, 1])
    real_y_init.append(real_trajectory[0, 2])
    # initial xy velocity and angle
    x_space_init = sum(real_trajectory[1:n_points+1, 1] - real_trajectory[:n_points, 1])
    y_space_init = sum(real_trajectory[1:n_points+1, 2] - real_trajectory[:n_points, 2])
    x_vel_init = np.abs(x_space_init) / real_trajectory[n_points, 0]
    y_vel_init = np.abs(y_space_init) / real_trajectory[n_points, 0]
    real_x_vel.append(x_vel_init)
    real_y_vel.append(y_vel_init)
    real_angle.append(np.arctan2(y_vel_init, x_vel_init))
    # average acceleration
    x_space_final = sum(real_trajectory[-n_points:, 1] - real_trajectory[-n_points-1:-1, 1])
    y_space_final = sum(real_trajectory[-n_points:, 2] - real_trajectory[-n_points-1:-1, 2])
    x_vel_final = np.abs(x_space_final) / (real_trajectory[-1, 0] - real_trajectory[-n_points-1, 0])
    y_vel_final = np.abs(y_space_final) / (real_trajectory[-1, 0] - real_trajectory[-n_points-1, 0])
    real_avg_acceleration.append((np.sqrt(x_vel_final**2 + y_vel_final**2) - np.sqrt(x_vel_init**2 + y_vel_init**2)) / real_trajectory[-1, 0])
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
    real_moving_avg_traj = moving_avg(real_trajectory, n_mov_avg)
    for i in range(2, len(real_moving_avg_traj)-n_points-1):
        if (real_moving_avg_traj[i, 2] - real_moving_avg_traj[i-2, 2] < 0) and (real_moving_avg_traj[i+1, 2] - real_moving_avg_traj[i, 2] > 0):
            bouncing_index = i + n_mov_avg
            found = True
            break
    if found:
        x_before_bouncing = sum(real_moving_avg_traj[bouncing_index-n_points+1:bouncing_index+1, 1] - real_moving_avg_traj[bouncing_index-n_points:bouncing_index, 1])
        y_before_bouncing = sum(real_moving_avg_traj[bouncing_index-n_points+1:bouncing_index+1, 2] - real_moving_avg_traj[bouncing_index-n_points:bouncing_index, 2])
        delta_t_pre = real_moving_avg_traj[bouncing_index, 0] - real_moving_avg_traj[bouncing_index - n_points, 0]
        x_vel_pre = np.abs(x_before_bouncing/delta_t_pre)
        y_vel_pre = np.abs(y_before_bouncing/delta_t_pre)
        x_after_bouncing = sum(real_moving_avg_traj[bouncing_index+1:min(bouncing_index + n_points + 1, len(real_moving_avg_traj)), 1] - real_moving_avg_traj[bouncing_index:min(bouncing_index+n_points, len(real_moving_avg_traj)-1), 1])
        y_after_bouncing = sum(real_moving_avg_traj[bouncing_index+1:min(bouncing_index + n_points + 1, len(real_moving_avg_traj)), 2] - real_moving_avg_traj[bouncing_index:min(bouncing_index+n_points, len(real_moving_avg_traj)-1), 2])
        delta_t_post = real_moving_avg_traj[min(bouncing_index+n_points, len(real_moving_avg_traj)-1), 0] - real_moving_avg_traj[bouncing_index, 0]
        x_vel_post = np.abs(x_after_bouncing/delta_t_post)
        y_vel_post = np.abs(y_after_bouncing/delta_t_post)
        real_x_restitution_coeffs.append(x_vel_post/x_vel_pre)
        real_y_restitution_coeffs.append(y_vel_post/y_vel_pre)
        real_avg_x_acceleration.append((x_vel_post - x_vel_pre) / delta_t_post)
        real_avg_y_acceleration.append((y_vel_post - y_vel_pre) / delta_t_post)
        # bouncing coordinate (table height)
        real_bouncing_coordinates.append(real_moving_avg_traj[i, 2])
    else:
        real_x_restitution_coeffs.append(np.nan)
        real_y_restitution_coeffs.append(np.nan)
        real_bouncing_coordinates.append(np.nan)

real_mean_spatial_delta = np.mean(real_spatial_delta)
real_mean_rate_list = np.array(real_mean_rate_list)
real_mean_rate = np.nanmean(real_mean_rate_list, axis=0)
real_std_rate = np.nanstd(real_mean_rate_list, axis=0)
real_mean_rate_x = np.nanmean(real_mean_rate_x_list, axis=0)
real_std_rate_x = np.nanstd(real_mean_rate_x_list, axis=0)
real_mean_rate_x_list = np.array(real_mean_rate_x_list)
real_bouncing_coordinates = np.array(real_bouncing_coordinates)
print("Bouncing height for real trajs:   mean = ", np.nanmean(real_bouncing_coordinates), "     std = " , np.nanstd(real_bouncing_coordinates))


################################################ SIMULATED TRAJECTORIES ################################################
sim_folder = "asynch_trajectories"
sim_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), sim_folder)
sim_trajs = sorted([traj for traj in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), sim_path)) if traj.endswith('.txt')])
sim_spatial_delta = []
sim_x_init, sim_y_init = [], []
sim_x_vel, sim_y_vel, sim_angle = [], [], []
sim_avg_acceleration = []
sim_avg_x_acceleration, sim_avg_y_acceleration = [], []
sim_mean_rate_list, sim_std_rate_list = [], []
sim_mean_rate_x_list, sim_std_rate_x_list = [], []
sim_x_restitution_coeffs, sim_y_restitution_coeffs = [], []
sim_bouncing_coordinates = []
for t in tqdm(sim_trajs, "Loading sim trajectories..."):
    sim_trajectory = np.loadtxt(os.path.join(os.path.dirname(os.path.abspath(__file__)), sim_folder, t), delimiter=",")
    if sim_trajectory.size == 0:
        print(t)
    else:
        sim_trajectory[:, 0] = np.cumsum(sim_trajectory[:, 0])
        # sim_trajectory[:, 1] = 304 - sim_trajectory[:, 1]    # flip x-coordinate
        sim_trajectory[:, 2] = 240 - sim_trajectory[:, 2]    # flip y-coordinate
        # spatial delta
        sim_spatial_x = sim_trajectory[1:, 1] - sim_trajectory[:-1, 1]
        sim_spatial_y = sim_trajectory[1:, 2] - sim_trajectory[:-1, 2]
        sim_spatial_delta.append(np.mean(np.sqrt(sim_spatial_x**2 + sim_spatial_y**2)))
        # initial coordinates
        sim_x_init.append(sim_trajectory[0, 1])
        sim_y_init.append(sim_trajectory[0, 2])
        # initial xy velocity and angle
        x_space_init = sum(sim_trajectory[1:n_points+1, 1] - sim_trajectory[:n_points, 1])
        y_space_init = sum(sim_trajectory[1:n_points+1, 2] - sim_trajectory[:n_points, 2])
        x_vel_init = np.abs(x_space_init) / sim_trajectory[n_points, 0]
        y_vel_init = np.abs(y_space_init) / sim_trajectory[n_points, 0]
        sim_x_vel.append(x_vel_init)
        sim_y_vel.append(y_vel_init)
        sim_angle.append(np.arctan2(y_vel_init, x_vel_init))
        # average acceleration
        x_space_final = sum(sim_trajectory[-n_points:, 1] - sim_trajectory[-n_points-1:-1, 1])
        y_space_final = sum(sim_trajectory[-n_points:, 2] - sim_trajectory[-n_points-1:-1, 2])
        x_vel_final = np.abs(x_space_final) / (sim_trajectory[-1, 0] - sim_trajectory[-n_points-1, 0])
        y_vel_final = np.abs(y_space_final) / (sim_trajectory[-1, 0] - sim_trajectory[-n_points-1, 0])
        sim_avg_acceleration.append((np.sqrt(x_vel_final**2 + y_vel_final**2) - np.sqrt(x_vel_init**2 + y_vel_init**2)) / sim_trajectory[-1, 0])
        # timestep rate
        delta_t = sim_trajectory[1:, 0] - sim_trajectory[:-1, 0]
        perc_points = np.arange(0.0, 1.1, 0.1)
        normalized_time = sim_trajectory[1:, 0]/sim_trajectory[-1, 0]
        mean_rate, std_rate = extract_perc_values(delta_t, normalized_time, perc_points, range=0.05)
        sim_mean_rate_list.append(mean_rate)
        sim_std_rate_list.append(std_rate)
        # timestep rate normalized over x
        normalized_x = sim_trajectory[1:, 1]/304
        mean_rate_x , std_rate_x = extract_perc_values(delta_t, normalized_x, perc_points, range=0.05)
        sim_mean_rate_x_list.append(mean_rate_x)
        sim_std_rate_x_list.append(std_rate_x)
        # acceleration at bounce
        # find the first time the derivative gets positive after being negative
        found = False
        sim_moving_avg_traj = moving_avg(sim_trajectory, n_mov_avg)
        for i in range(2, len(sim_trajectory)-n_points-1):
            if (sim_moving_avg_traj[i, 2] - sim_moving_avg_traj[i-2, 2] < 0) and (sim_moving_avg_traj[i+1, 2] - sim_moving_avg_traj[i, 2] > 0):
                bouncing_index = i
                found = True
                break
        if found:
            x_before_bouncing = sum(sim_moving_avg_traj[bouncing_index-n_points+1:bouncing_index+1, 1] - sim_moving_avg_traj[bouncing_index-n_points:bouncing_index, 1])
            y_before_bouncing = sum(sim_moving_avg_traj[bouncing_index-n_points+1:bouncing_index+1, 2] - sim_moving_avg_traj[bouncing_index-n_points:bouncing_index, 2])
            delta_t_pre = sim_moving_avg_traj[bouncing_index, 0] - sim_moving_avg_traj[bouncing_index - n_points, 0]
            x_vel_pre = np.abs(x_before_bouncing/delta_t_pre)
            y_vel_pre = np.abs(y_before_bouncing/delta_t_pre)
            x_after_bouncing = sum(sim_moving_avg_traj[bouncing_index+1:min(bouncing_index+n_points+1, len(sim_moving_avg_traj)), 1] - sim_moving_avg_traj[bouncing_index:min(bouncing_index+n_points, len(sim_moving_avg_traj)-1), 1])
            y_after_bouncing = sum(sim_moving_avg_traj[bouncing_index+1:min(bouncing_index+n_points+1, len(sim_moving_avg_traj)), 2] - sim_moving_avg_traj[bouncing_index:min(bouncing_index+n_points, len(sim_moving_avg_traj)-1), 2])
            delta_t_post = sim_moving_avg_traj[min(bouncing_index+n_points, len(sim_moving_avg_traj)-1), 0] - sim_moving_avg_traj[bouncing_index, 0]
            x_vel_post = np.abs(x_after_bouncing/delta_t_post)
            y_vel_post = np.abs(y_after_bouncing/delta_t_post)
            sim_x_restitution_coeffs.append(x_vel_post/x_vel_pre)
            sim_y_restitution_coeffs.append(y_vel_post/y_vel_pre)
            sim_avg_x_acceleration.append((x_vel_post - x_vel_pre) / delta_t_post)
            sim_avg_y_acceleration.append((y_vel_post - y_vel_pre) / delta_t_post)
            # bouncing coordinate (table height)
            sim_bouncing_coordinates.append(sim_moving_avg_traj[i, 2])
        else:
            sim_x_restitution_coeffs.append(np.nan)
            sim_y_restitution_coeffs.append(np.nan)
            sim_bouncing_coordinates.append(np.nan)

sim_mean_spatial_delta = np.mean(sim_spatial_delta)
sim_mean_rate_list = np.array(sim_mean_rate_list)
sim_mean_rate = np.nanmean(sim_mean_rate_list, axis=0)
sim_std_rate = np.nanstd(sim_mean_rate_list, axis=0)
sim_mean_rate_x = np.nanmean(sim_mean_rate_x_list, axis=0)
sim_std_rate_x = np.nanstd(sim_mean_rate_x_list, axis=0)
sim_mean_rate_x_list = np.array(sim_mean_rate_x_list)
sim_bouncing_coordinates = np.array(sim_bouncing_coordinates)
print("Bouncing height for simulated trajs:   mean = ", np.nanmean(sim_bouncing_coordinates), "     std = " , np.nanstd(sim_bouncing_coordinates))


######################################################## PLOTS ########################################################
# Initial Position
fig1 = plt.figure(1)
plt.scatter(sim_x_init, sim_y_init, color='#ED0DD9', alpha=0.4, s=100, label='simulated')
plt.scatter(real_x_init, real_y_init, color='darkcyan', alpha=0.4, s=100, label='real')
plt.title(r'Initial Position')
plt.xlabel(r'init X [pxl]')
plt.ylabel(r'init Y [pxl]')
plt.grid()
plt.legend()
plt.show()

# Initial Velocity
fig2 = plt.figure(2)
plt.scatter(sim_x_vel, sim_y_vel, color='#ED0DD9', alpha=0.4, s=100, label='simulated')
plt.scatter(real_x_vel, real_y_vel, color='darkcyan', alpha=0.4, s=100, label='real')
plt.title(r'Initial Velocity (first '+ str(n_points) + ' points)')
plt.xlabel(r'init X vel [pxl/s]')
plt.ylabel(r'init Y vel [pxl/s]')
plt.grid()
plt.legend()
plt.show()

# Spatial Delta
fig3, ax3 = plt.subplots()
ticks = ['real', 'simulated']
spatial_data = [real_spatial_delta, sim_spatial_delta]
plt.boxplot(spatial_data, vert=True, patch_artist=True, labels=ticks)
plt.ylabel(r' Spatial delta [pxl]')
plt.grid()
plt.show()

# Mean acceleration
fig9, ax9 = plt.subplots()
ticks = ['real', 'simulated']
accel_data = [real_avg_acceleration, sim_avg_acceleration]
plt.boxplot(accel_data, vert=True, patch_artist=True, labels=ticks)
plt.ylabel(r' Average acceleration [pxl]')
plt.grid()
plt.show()

# Delta_t
plt.figure(figsize=(18, 14))
plt.gca()
plt.plot((perc_points*100).astype(int), real_mean_rate*1000, color='darkcyan', label='real')
plt.fill_between((perc_points*100).astype(int), y1=(real_mean_rate-real_std_rate)*1000, y2=(real_mean_rate+real_std_rate)*1000, alpha=0.3, color='darkcyan')
plt.plot((perc_points*100).astype(int), sim_mean_rate*1000, color='#ED0DD9', label='simulated')
plt.fill_between((perc_points*100).astype(int), y1=(sim_mean_rate-sim_std_rate)*1000, y2=(sim_mean_rate+sim_std_rate)*1000, alpha=0.3, color='#ED0DD9')
plt.xlabel('Trajectory %')
plt.ylabel('Average rate [ms]')
plt.grid()
plt.legend()
plt.show()

# Delta_t horizontal
plt.figure(figsize=(18, 14))
plt.gca()
plt.plot((perc_points*100).astype(int), real_mean_rate_x*1000, color='darkcyan', label='real')
plt.fill_between((perc_points*100).astype(int), y1=(real_mean_rate_x-real_std_rate_x)*1000, y2=(real_mean_rate_x+real_std_rate_x)*1000, alpha=0.3, color='darkcyan')
plt.plot((perc_points*100).astype(int), sim_mean_rate_x*1000, color='#ED0DD9', label='simulated')
plt.fill_between((perc_points*100).astype(int), y1=(sim_mean_rate_x-sim_std_rate_x)*1000, y2=(sim_mean_rate_x+sim_std_rate_x)*1000, alpha=0.3, color='#ED0DD9')
plt.xlabel('Trajectory %')
plt.ylabel('Average horizontal rate [ms]')
plt.grid()
plt.legend()
plt.show()

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

# Coefficient Of Restitution
fig8 = plt.figure()
plt.scatter(sim_x_restitution_coeffs, sim_y_restitution_coeffs, color='#ED0DD9', alpha=0.4, s=100, label='simulated')
plt.scatter(real_x_restitution_coeffs, real_y_restitution_coeffs, color='darkcyan', alpha=0.4, s=100, label='real')
plt.xlabel(r' COR along X')
plt.ylabel(r'COR along Y')
plt.grid()
plt.legend()
plt.show()

# Mean acceleration
fig10, ax10 = plt.subplots()
ticks = ['real', 'simulated']
bouncing_height = [real_bouncing_coordinates, sim_bouncing_coordinates[~np.isnan(sim_bouncing_coordinates)]]
plt.boxplot(bouncing_height, vert=True, patch_artist=True, labels=ticks)
plt.ylabel(r' Bouncing height [pxl]')
plt.grid()
plt.show()

pass