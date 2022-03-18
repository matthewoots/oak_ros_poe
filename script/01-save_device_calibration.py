#!/usr/bin/env python3

from pathlib import Path
import depthai as dai

import argparse

calibBackUpFile = str((Path(__file__).parent / Path('depthai_calib_backup.json')).resolve().absolute())

parser = argparse.ArgumentParser()
parser.add_argument('calibBackUpFile', nargs='?', help="Path to V6 calibration file in json", default=calibBackUpFile)
args = parser.parse_args()

# Connect device
with dai.Device() as device:

    deviceCalib = device.readCalibration()
    deviceCalib.eepromToJsonFile(args.calibBackUpFile)
    print("Calibration Data on the device is backed up at:")
    print(args.calibBackUpFile)