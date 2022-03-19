#!/bin/bash

set -e

DRONE_NO="01"

echo "STEP 1"
python3 01-save_device_calibration.py depthai_calib_drone${DRONE_NO}.json

echo "STEP 2"
python3 02-flash_device_calibration.py depthai_calib_drone${DRONE_NO}.json camchain-drone${DRONE_NO}.yaml

echo "STEP 3"
python3 03-generate-mesh-file.py 

rm depthai_calib_drone${DRONE_NO}.json
