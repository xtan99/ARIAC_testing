#!/bin/bash

cd /container_scripts

chmod +x run_test.py

source /opt/ros/iron/setup.bash
source /workspace/install/setup.bash

python3 run_test.py $1