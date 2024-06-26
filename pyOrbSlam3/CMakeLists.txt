cmake_minimum_required(VERSION 3.4...3.18)
project(pyOrbSlam)

LIST(APPEND CMAKE_MODULE_PATH 
    ${PROJECT_SOURCE_DIR}/cmake
)

#set (PYTHON_LIBRARIES  #necessary if cmake finds the wrong python environment
# /home/jhm/miniconda3/lib/libpython3.9.so)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -march=native  -O0 -g3 -ggdb")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -march=native  -O0 -g3 -ggdb")

add_subdirectory(modules/pybind11)

find_package(OpenCV REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/modules/ORB_SLAM3/cmake_modules)

#add_subdirectory(modules/ORB_SLAM3)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/modules/ORB_SLAM3/
${PROJECT_SOURCE_DIR}/modules/ORB_SLAM3/include
${PROJECT_SOURCE_DIR}/modules/ORB_SLAM3/include/CameraModels
${PROJECT_SOURCE_DIR}/modules/pybind11_opencv_numpy
${PROJECT_SOURCE_DIR}/modules/ORB_SLAM3/Thirdparty/Sophus/sophus
${PROJECT_SOURCE_DIR}/modules/ORB_SLAM3/Thirdparty/Sophus
${Pangolin_INCLUDE_DIRS}
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/modules/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/modules/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/modules/ORB_SLAM3/lib/libORB_SLAM3.so
-lboost_system
)
find_package(Python COMPONENTS Interpreter Development REQUIRED)
message("Python_FOUND:${Python_FOUND}")
message("Python_VERSION:${Python_VERSION}")
message("Python_Development_FOUND:${Python_Development_FOUND}")
message("Python_LIBRARIES:${Python_LIBRARIES}")

pybind11_add_module(pyOrbSlam 
    src/pyOrbSlam.cpp
    ${PROJECT_SOURCE_DIR}/modules/pybind11_opencv_numpy/ndarray_converter.cpp
)

add_executable(testOrbSlamVideo
        ${PROJECT_SOURCE_DIR}/src/orbSlamTest.cc)
target_link_libraries(testOrbSlamVideo
PRIVATE
    ${OpenCV_LIBS}
    ${LIBS}
)


target_link_libraries(pyOrbSlam
PRIVATE
    ${OpenCV_LIBS}
    ${LIBS}
    ${PYTHON_LIBRARIES} Python3::NumPy
)

target_compile_definitions(pyOrbSlam 
PRIVATE 
    VERSION_INFO=${EXAMPLE_VERSION_INFO}
)
#string(REPLACE "libpython${PY_LIB}" /libORB_SLAM3.so PY_LIB ${Python_LIBRARIES})
message("copying libORB_SLAM3.so to python libraries")

string(REGEX REPLACE "[^\/]+$" "libORB_SLAM3.so" PY_LIB ${Python_LIBRARIES})

set(COM 
   "cp ${PROJECT_SOURCE_DIR}/modules/ORB_SLAM3/lib/libORB_SLAM3.so ${PY_LIB}")

message(${COM})

execute_process (
    COMMAND bash -c ${COM})


