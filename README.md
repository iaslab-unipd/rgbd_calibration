## Dependencies

This package depends on [calibration_toolkit](https://github.com/iaslab-unipd/calibration_toolkit) version 0.3.2 (i.e. `git checkout v0.3.2`).

## Initial Setup

Edit **conf/checkerboards.yaml** to match your checkerboard(s) parameters.

## Data Collection

Use the [sensor_data_collection](https://github.com/iaslab-unipd/sensor_data_collection) package for collecting data from your sensors.

## Calibration

File **launch/kinect_47A_tro.launch** is a sample file to run the offline calibration of a Kinect 1.
Modify it to match your setup and perform the calibration.

