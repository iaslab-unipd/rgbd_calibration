---
layout: default
title: RGB-D Calibration
---

# [](#software)Software

The code is available on [GitHub]({{ site.github.repository_url }}) under a [BSD license](https://opensource.org/licenses/BSD-3-Clause).

Main dependencies:

- [ROS Indigo](http://wiki.ros.org/indigo)
- [Eigen](http://eigen.tuxfamily.org/)
- [PCL](http://pointclouds.org/)
- [OpenCV](http://opencv.org/)
- [Ceres Solver](http://ceres-solver.org/)

# [](#results)Results

Qualitative results of _visual odometry_ experiments on original and calibrated data:

- first row: [DVO][dvo-url];
- second row: OFVO.

{% include image.html name="vo_results.png" %}


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
