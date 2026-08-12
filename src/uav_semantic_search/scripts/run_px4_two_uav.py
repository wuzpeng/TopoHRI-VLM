#!/usr/bin/env python3
"""ROS-safe Python launcher for the PX4/Gazebo shell startup script.

Do not install a .sh file with catkin_install_python(); Catkin creates a
Python wrapper in devel/lib and shell syntax then produces SyntaxError.
"""
import os
import sys
from pathlib import Path


def main():
    try:
        import rospkg
        package_dir = Path(rospkg.RosPack().get_path("uav_semantic_search"))
        shell_script = package_dir / "scripts" / "run_px4_two_uav.sh"
    except Exception:
        shell_script = Path(__file__).resolve().with_suffix(".sh")

    if not shell_script.is_file():
        raise SystemExit("PX4 startup shell script not found: %s" % shell_script)

    # Replace this process with Bash so Ctrl+C and termination signals propagate
    # directly to the existing shell implementation.
    os.execvp("bash", ["bash", str(shell_script)] + sys.argv[1:])


if __name__ == "__main__":
    main()
