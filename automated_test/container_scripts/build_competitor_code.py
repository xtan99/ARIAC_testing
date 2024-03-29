#!/usr/bin/env python3

import os
import sys
import subprocess
import yaml


def main():
    
    # Install rosdep packages
    os.chdir('/workspace') 
    rosdep_cmd = "rosdep install --from-paths src --ignore-src -y"
    rosdep_update_cmd = "rosdep update --include-eol-distros"
    rosdep_fix_cmd = " sudo apt-get update"
    subprocess.run(rosdep_fix_cmd, shell=True)
    subprocess.run(rosdep_update_cmd, shell=True)
    subprocess.run(rosdep_cmd, shell=True)

    # Build the workspace
    build_cmd = "colcon build --packages-skip ariac_controllers ariac_description ariac_gui ariac_moveit_config ariac_msgs ariac_plugins ariac_sensors"
    subprocess.run(build_cmd, shell=True)


if __name__=="__main__":
    main()
