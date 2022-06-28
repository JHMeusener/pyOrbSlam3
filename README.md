This repo provides Python Bindings to OrbSlam (OrbSlam3).

OrbSlam3 is a simultaneous mapping and localization (SLAM) algorithm based on oriented fast rotated brief (ORB) descriptors.

it is based on 
https://github.com/UZ-SLAMLab/ORB_SLAM3 (module OrbSlam3),
https://github.com/pybind/pybind11 (module pybind11),
https://github.com/edmBernard/pybind11_opencv_numpy (module pybind11_opencv_numpy and setup.py),
Code from these have their own licences

Installation Tested on Ubuntu 22.04 and Ubuntu 18.04

Prerequisites:
OpenCV >= 4.4
Python3 > 3.6
python-numpy
compiler capable of c++14
prerequisites of the submodules

changes to make to Orbslam3:
copy the folder "/modules/changesInOrbSlam3" into OrbSlam3

Compile the OrbSlam3 module according to the OrbSlam3 instructions

install sophos from the orbslam3 thirdparty modules (in the build folder: sudo make install)

in the main directory install PyOrbSlam3 by entering "pip install ."

Usage in Python:
see example

Each row of the extracted Mappoints consists of 3-XYZ-Coordinates, 3-Normal-Vector, 1-NrOfObservations, 1-FoundRatio and *-Orb-Descriptor of the Mappoint

Currently only the monocular camera should work.
