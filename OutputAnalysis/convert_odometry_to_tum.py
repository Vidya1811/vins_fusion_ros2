# vins_estimate_convert.py
import re

with open("vins_raw.txt") as f, open("vins_estimate_tum.txt", "w") as out:
    lines = f.readlines()
    block = ""
    for line in lines:
        if line.strip() == "---":
            try:
                sec = int(re.search(r"sec:\s*(\d+)", block).group(1))
                nsec = int(re.search(r"nanosec:\s*(\d+)", block).group(1))
                timestamp = sec + nsec * 1e-9

                tx = float(re.search(r"position:\s*\n\s*x:\s*([-\d.]+)", block).group(1))
                ty = float(re.search(r"y:\s*([-\d.]+)", block).group(1))
                tz = float(re.search(r"z:\s*([-\d.]+)", block).group(1))

                qx = float(re.search(r"orientation:\s*\n\s*x:\s*([-\d.]+)", block).group(1))
                qy = float(re.search(r"y:\s*([-\d.]+)", block).group(1))
                qz = float(re.search(r"z:\s*([-\d.]+)", block).group(1))
                qw = float(re.search(r"w:\s*([-\d.]+)", block).group(1))

                out.write(f"{timestamp:.9f} {tx:.6f} {ty:.6f} {tz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")
            except:
                pass
            block = ""
        else:
            block += line

