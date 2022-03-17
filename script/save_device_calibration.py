#!/usr/bin/env python3

from pathlib import Path
import depthai as dai

calibBackUpFile = str((Path(__file__).parent / Path('depthai_calib_backup.json')).resolve().absolute())

# Connect device
with dai.Device() as device:

    deviceCalib = device.readCalibration()
    deviceCalib.eepromToJsonFile(calibBackUpFile)
    print("Calibration Data on the device is backed up at:")
    print(calibBackUpFile)