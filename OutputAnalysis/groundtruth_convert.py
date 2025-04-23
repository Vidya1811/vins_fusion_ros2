# groundtruth_convert.py (final working version)
import re

def extract_block(lines, start_idx):
    block = ""
    for i in range(start_idx, len(lines)):
        if lines[i].strip() == "---":
            break
        block += lines[i]
    return block

with open("vicon_raw.txt") as f, open("groundtruth_tum.txt", "w") as out:
    lines = f.readlines()
    i = 0
    while i < len(lines):
        if "stamp:" in lines[i]:
            block = extract_block(lines, i)
            try:
                sec = int(re.search(r"sec:\s*(\d+)", block).group(1))
                nsec = int(re.search(r"nanosec:\s*(\d+)", block).group(1))
                timestamp = sec + nsec * 1e-9

                # Try Vicon format first
                t_match = re.search(r"translation:\s*\n\s*x:\s*([-\d.]+)\n\s*y:\s*([-\d.]+)\n\s*z:\s*([-\d.]+)", block)
                r_match = re.search(r"rotation:\s*\n\s*x:\s*([-\d.]+)\n\s*y:\s*([-\d.]+)\n\s*z:\s*([-\d.]+)\n\s*w:\s*([-\d.]+)", block)

                # Fallback to PoseStamped format
                if not t_match:
                    t_match = re.search(r"position:\s*\n\s*x:\s*([-\d.]+)\n\s*y:\s*([-\d.]+)\n\s*z:\s*([-\d.]+)", block)
                if not r_match:
                    r_match = re.search(r"orientation:\s*\n\s*x:\s*([-\d.]+)\n\s*y:\s*([-\d.]+)\n\s*z:\s*([-\d.]+)\n\s*w:\s*([-\d.]+)", block)

                if t_match and r_match:
                    tx, ty, tz = map(float, t_match.groups())
                    qx, qy, qz, qw = map(float, r_match.groups())
                    out.write(f"{timestamp:.9f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
            except Exception as e:
                print(f"Skipping block at line {i}: {e}")
        i += 1

