---
layout: default
title: RGB-D Calibration
---

# [](#description)Description

Typical consumer-grade **RGB-D cameras** are provided with a coarse intrinsic and extrinsic calibration that generally does not meet the accuracy requirements needed by many robotics applications. Our calibration approach is based on a **novel, two components, measurement error model** that unifies the error sources of different RGB-D pairs based on different technologies, such as structured-light 3D cameras and time-of-flight cameras. The proposed correction model is implemented using two different **parametric undistortion maps** that provide the calibrated readings by means of linear combinations of control functions. A non-linear optimization algorithm refines the camera-depth sensor rigid displacement along with the aforementioned parametric maps in a single optimization step guaranteeing highly reliable results.

# [](#software)Software

The code is available on [GitHub]({{ site.github.repository_url }}) under a [BSD license](https://opensource.org/licenses/BSD-3-Clause) and is specifically developed for [ROS Indigo](http://wiki.ros.org/indigo).

Main dependencies:

- [Eigen](http://eigen.tuxfamily.org/)
- [PCL](http://pointclouds.org/)
- [OpenCV](http://opencv.org/)
- [Ceres Solver](http://ceres-solver.org/)

# [](#results)Results

## Depth Correction

Results of the calibration applied to a set of clouds of a wall at different distances.

{% include image.html name="calib_results.png" %}

## RGB-Depth Registration

Resulting RGB-Depth registration using default calibration parameters compared to the RGB-Depth registration after the calibration procedure.

{% include image.html name="original_rgbd.png" %}
{% include image.html name="calibrated_rgbd.png" %}

## Visual Odometry

Qualitative results of visual odometry experiments on original and calibrated data. The estimated path (first column) and the clouds reconstructed using the original distorted data and the corrected ones (second and third column respectively) are compared.

{% include image.html name="vo_results.png" %}

**First row:** results obtained from [DVO][dvo-url], a state-of-the-art software. **Second row:** results obtained from OFVO, a simpler approach developed just for testing purposes.


# [](#publications)Publications

1. **Robust Intrinsic and Extrinsic Calibration of RGB-D Cameras**, Filippo Basso, Emanuele Menegatti and Alberto Pretto, _2017_. [[ArXiv preprint][arxiv-url]] [[BibTeX][arvix-bib]]
1. **A non-parametric Calibration Algorithm for Depth Sensors Exploiting RGB Cameras**, Filippo Basso, _Ph.D. Thesis, 2015_. [[Padua@research][thesis-url]] [[BibTeX][thesis-bib]]
1. **Unsupervised Intrinsic and Extrinsic Calibration of a Camera-Depth Sensor Couple**, Filippo Basso, Alberto Pretto and Emanuele Menegatti, _Robotics and Automation (ICRA), 2014 IEEE International Conference on_. [Obsolete] [[IEEEXplore][icra-url]] [[BibTeX][icra-bib]]


[dvo-url]: https://github.com/tum-vision/dvo

[arxiv-url]: {{ site.paper_url }}
[arvix-bib]: {{ site.url }}/download/arxiv2017.bib
[thesis-url]: http://paduaresearch.cab.unipd.it/8908/
[thesis-bib]: {{ site.url }}/download/thesis.bib
[icra-url]: http://ieeexplore.ieee.org/document/6907780/
[icra-bib]: {{ site.url }}/download/icra2014.bib

*[OFVO]: (Optical Flow Visual Odometry) is a very simple custom-built visual odometry system based on dense optical flow.
