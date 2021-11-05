
import os
import numpy as np
from tqdm import tqdm

# if not existing, create the folder where to save the trajectories
if not os.path.exists(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'asynch_tracker_trajectories')):
    os.makedirs('asynch_tracker_trajectories')

source = "/home/mmonforte/data/Sphere/"
folders = tqdm(sorted(os.listdir(source)))
for f in folders:
    folders.set_description("Processing folder %s" % f)
    src_file = np.loadtxt(source+f+"/tracking_prediction_log.txt", delimiter=",")[:,:3]
    # src_file[:, 0] = np.cumsum(src_file[:, 0])-src_file[0, 0]
    src_file[:, 1] = 304 - src_file[:, 1]
    np.savetxt("asynch_tracker_trajectories/asynch_traj_"+str(f.zfill(5))+".txt", src_file, fmt=['%f', '%d', '%d'], delimiter=",")
