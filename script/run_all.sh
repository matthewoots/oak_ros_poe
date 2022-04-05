#!/bin/bash

set -e

NAME="drone02"
CALIB_TYPE="pinhole"

# IMPORTANT: for OAK-D-PRO please use 400; for OAK-D-Lite please use 480
RESOLUTION=480

echo "STEP 1"
python3 01-save_device_calibration.py depthai_calib_${NAME}.json

echo "STEP 2"
python3 02-flash_device_calibration.py depthai_calib_${NAME}.json camchain-${CALIB_TYPE}-${NAME}.yaml

echo "STEP 3"
python3 03-generate-mesh-file.py --resolution ${RESOLUTION}

rm depthai_calib_${NAME}.json
