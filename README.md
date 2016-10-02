## Dependencies

This package depends on [calibration_toolkit](https://github.com/iaslab-unipd/calibration_toolkit) version 0.3 (i.e. `git checkout v0.3`).

## Initial Setup

Edit **conf/checkerboards.yaml** to match your checkerboard(s) parameters.

## Data Collection

Edit **launch/kinect_data_collection.launch** to match your preferences.

Setup your sensor node (e.g. use openni instead of openni2):
```
<include file="$(find openni2_launch)/launch/openni2.launch">
  ...
</include>
```
And the published topics:
```
<node pkg="rgbd_calibration" type="rgbd_data_collection" name="rgbd_data_collection" output="screen" required="true">
  ...
  <remap from="~image"                  to="/$(arg kinect_name)/rgb/image_rect_color" />
  <remap from="~camera_info"            to="/$(arg kinect_name)/rgb/camera_info" />
  <remap from="~point_cloud"            to="/$(arg kinect_name)/depth/points" />
</node>
```
You can also change some other parameters, for example:
* Set `search_checkerboard` to `false` to collect all images even those where the checkerboard is not visible.
* Change the `rate` value to get data more quickly.

At the end, launch the node:
```
roslaunch rgbd_calibration kinect_data_collection.launch save_folder:=<your_folder>
```

## Calibration

Edit **launch/kinect_offline_calibration.launch** to match your preferences.

