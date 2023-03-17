
#  AUTHOR: Rama Hruday Bandaru
#  FILENAME: startsitl.sh
#  SPECIFICATION: linux command for running the arducoptor SITL which connects to gazebo iris drone model
#  FOR: CS 5392 Reinforcement Learning Section 01 

#!/bin/bash
cd ~/ardupilot/ArduCopter/ && sim_vehicle.py -v ArduCopter -f gazebo-iris --console