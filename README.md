This repo provides python bindings to OrbSlam3

it is based on 
https://github.com/UZ-SLAMLab/ORB_SLAM3 (module orbslam3),
https://github.com/pybind/pybind11 (module pybind11),
https://github.com/edmBernard/pybind11_opencv_numpy  (setup.py, ndarray_converter.cpp/.h),
Code from these have their own licences


changes to make to Orbslam3:
copy the folder "/modules/changesInOrbSlam3" into OrbSlam3

Compile the Orbslam3 module according to the OrbSlam3 instructions

install sophos from the orbslam3 thirdparty modules (in the build folder: sudo make install)

in the main directory install PyOrbSlam3 by entering "pip install ."

Usage in Python:
see example

Each row of the extracted Mappoints consists of 3-XYZ-Coordinates, 3-Normal-Vector, 1-NrOfObservations, 1-FoundRatio and *-Orb-Descriptor of the Mappoint
