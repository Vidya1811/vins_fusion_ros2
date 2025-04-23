# convert_vicon_to_tum.py
import re

with open("vicon_raw.txt") as f, open("groundtruth_tum.txt", "w") as out:
    lines = f.readlines()
    sec = nsec = 0
    tx = ty = tz = qx = qy = qz = qw = None

    for line in lines:
        if "sec:" in line:
            sec = int(re.search(r"sec:\s*(\d+)", line).group(1))
        elif "nanosec:" in line:
            nsec = int(re.search(r"nanosec:\s*(\d+)", line).group(1))
        elif "translation" in line:
            match = re.findall(r"[-+]?\d*\.\d+|\d+", line)
            tx, ty, tz = map(float, match[:3])
        elif "rotation" in line:
            match = re.findall(r"[-+]?\d*\.\d+|\d+", line)
            qx, qy, qz, qw = map(float, match[:4])

        if None not in [tx, ty, tz, qx, qy, qz, qw]:
            timestamp = sec + nsec * 1e-9
            out.write(f"{timestamp:.9f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
            tx = ty = tz = qx = qy = qz = qw = None

