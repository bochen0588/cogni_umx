#!/usr/bin/env python3
import subprocess
import platform
import os
import shutil
from pathlib import Path

# Auto-select path to OpenSCAD
if platform.system() == "Windows":
    OPENSCAD_PATH = r"C:\Program Files\OpenSCAD\openscad.exe"
else:
    OPENSCAD_PATH = "openscad"  # Assumes it's in PATH

SCAD_FILE = "cogni_umx.scad"


path = Path("parts")
if path.exists():
    shutil.rmtree(path);
path.mkdir(exist_ok=False)

parts =[
    "joint_wing_top_front",
    "joint_wing_top_rear",
    "joint_wing_top_front_left",
    "joint_wing_top_front_left_tip",
    "joint_wing_top_front_right",
    "joint_wing_top_front_right_tip",
    "joint_wing_top_rear_left",
    "joint_wing_top_rear_left_tip",
    "joint_wing_top_rear_right",
    "joint_wing_top_rear_right_tip",
    "joint_wing_bottom_front",
    "joint_wing_bottom_rear",
    "joint_gear_elevator",
]

# Export loop
for part in parts:
    print('part', part)
    output_file = path / f"{part}.stl"
    define_arg = f"-D part=\"{part}\""
    cmd = [OPENSCAD_PATH, "-o", output_file, define_arg, SCAD_FILE]
    print(f"Exporting {output_file}...")
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error exporting part {part}: {e}")
        break
    except FileNotFoundError:
        print("OpenSCAD not found. Check your OPENSCAD_PATH.")
        break

print("Done.")


