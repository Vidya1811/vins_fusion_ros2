import numpy as np
import matplotlib.pyplot as plt

def load_tum_file(path):
    data = []
    with open(path) as f:
        for line in f:
            if line.strip() and not line.startswith("#"):
                parts = line.strip().split()
                if len(parts) >= 8:
                    timestamp, tx, ty, tz = float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])
                    data.append((tx, ty, tz))
    return np.array(data)

# Load files
gt = load_tum_file("groundtruth_tum.txt")
est = load_tum_file("vins_estimate_tum.txt")

# Flip the VINS estimate (adjust axes if needed)
est_flipped = est.copy()
est_flipped[:, 0] *= 1  # Flip X
est_flipped[:, 1] *= 1  # Flip Y
# est_flipped[:, 2] *= -1  # Uncomment if Z needs flipping

# Plot in two separate subplots
plt.figure(figsize=(12, 5))

# Ground Truth
plt.subplot(1, 2, 1)
plt.plot(gt[:, 0], gt[:, 1], color='blue')
plt.title("Ground Truth Trajectory")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.axis('equal')

# VINS Estimate
plt.subplot(1, 2, 2)
plt.plot(est_flipped[:, 0], est_flipped[:, 1], color='red')
plt.title("VINS Estimate Trajectory")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.axis('equal')

plt.tight_layout()
plt.show()

