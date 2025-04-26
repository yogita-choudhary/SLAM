import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# === Load Ground Truth Trajectory ===
# gt_df = pd.read_csv('oxts_trajectory_clean.txt', sep=' ', header=None,
#                     names=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
gt_df = pd.read_csv('output_poses.txt', sep=' ', header=None,
                     names=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
gt_xyz = gt_df[['x', 'y', 'z']].to_numpy()

# === Load Estimated Trajectory ===
# est_df = pd.read_csv('pose_output.txt', sep=' ', header=None,
#                      names=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
# est_df = pd.read_csv('pose_output_lvi_sam.txt', sep=' ', header=None,
#                      names=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
# est_df = pd.read_csv('pose_output_lvio_sam_m2p2.txt', sep=' ', header=None,
#                      names=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
est_df = pd.read_csv('pose_output_lvio_sam_m2p2.txt', sep=' ', header=None,
                     names=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
# est_xyz_filtered = est_df[['x', 'y', 'z']].to_numpy()
# === Filter out repeated entries in estimated trajectory ===
est_xyz_filtered = est_df[['x', 'y', 'z']].drop_duplicates().reset_index(drop=True).to_numpy()
# 
# === Trim both to same length ===
print("len(gt_xyz):", len(gt_xyz), "len(est_xyz_filtered):", len(est_xyz_filtered))
min_len = len(est_xyz_filtered)
gt_trimmed = gt_xyz[::14]
est_trimmed = est_xyz_filtered[:min_len][:-2]

# === Plot Top-Down View ===
plt.figure(figsize=(8, 6))
plt.plot(gt_trimmed[:, 0], gt_trimmed[:, 1], label='Ground Truth', linewidth=2)
plt.plot(est_trimmed[:, 0], est_trimmed[:, 1], label='Estimated (filtered)', linestyle='--', linewidth=2)
plt.title('LVIO SAM Top-Down Trajectory View for M2P2 Dataset (X vs Y)')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.tight_layout()
plt.savefig('top_down_trajectory_lvio_sam_m2p2.png')

# === Compute Error Metrics ===
position_errors = est_trimmed - gt_trimmed
squared_errors = position_errors ** 2
rmse = np.sqrt(np.mean(squared_errors, axis=0))
rmse_total = np.sqrt(np.mean(np.sum(squared_errors, axis=1)))

# Absolute Pose Error (APE)
ape = np.linalg.norm(position_errors, axis=1)
ape_mean = np.mean(ape)
ape_std = np.std(ape)
ape_max = np.max(ape)

# Relative Pose Error (RPE)
rpe = np.linalg.norm(np.diff(position_errors, axis=0), axis=1)
rpe_mean = np.mean(rpe)
rpe_std = np.std(rpe)
rpe_max = np.max(rpe)

# === Print Metrics ===
print("=== Trajectory Error Metrics ===")
print(f"RMSE (x, y, z): {rmse}")
print(f"Total RMSE: {rmse_total:.2f} m")
print(f"APE: mean = {ape_mean:.2f} m, std = {ape_std:.2f} m, max = {ape_max:.2f} m")
print(f"RPE: mean = {rpe_mean:.2f} m, std = {rpe_std:.2f} m, max = {rpe_max:.2f} m")