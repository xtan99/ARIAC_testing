#!/usr/bin/env python3

import os
import sys
from subprocess import Popen, call
from signal import SIGINT, SIGTERM
import yaml
import time
import glob
import subprocess
import shutil


def main():
    package_name = "ariac_test"

    # Clear ros log folder
    if (os.path.exists('/root/.ros/log')):
        shutil.rmtree('/root/.ros/log/')
    
    # Clears the tmp directory
    for file_path in ["/tmp/test_log.txt"]:
        if os.path.exists(file_path):
            os.remove(file_path)

    test_name = sys.argv[1]
    launch_file = f"{test_name}.launch.py"
    
    if test_name == "test3":
        trial_name = "assembly_test"

    else:    
        trial_name = "testing"
    
    process = Popen(["ros2", "launch", package_name, launch_file, f"trial_name:={trial_name}", '--noninteractive'])

    time.sleep(10)

    log_file = open("/tmp/test_log.txt", 'a')
    log_file.write("Starting Test")

    while True:

        try:
            outs = subprocess.run(["ros2", "topic", "echo", "/test_status", "--once"], capture_output = True, text = True)
            result = outs.stdout
            if (result[6:12] == "Failed" or result[6:13] == "Passed!"):
                log_file.write(f"\n{result[6:-1]}")
                break

        except subprocess.CalledProcessError:
            log_file.write(f"Error while reading topic test_status")
            break


        try:
            output = subprocess.check_output(
                "gz topic -l", shell=True).decode("utf-8")

            if output == '' or output.count('An instance of Gazebo is not running') > 0:
                print('Gazebo Crashed')
                result = "Gazebo crashed"
                log_file.write(f"\n{result}")
                break
                
        except subprocess.CalledProcessError:
            pass

    print(f"==== Test {test_name} completed")

    log_file.write("\nCompleted Task")
    log_file.close()

    # process.send_signal(SIGTERM)
    process.kill()

    # Might raise a TimeoutExpired if it takes too long
    return_code = process.wait(timeout=10)
    print(f"return_code: {return_code}")

if __name__ == "__main__":
    main()
