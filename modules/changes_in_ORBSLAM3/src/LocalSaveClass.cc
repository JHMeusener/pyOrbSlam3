#include "LocalSaveClass.h"
#include <mutex> // For std::mutex

namespace ORB_SLAM3 {

// Static function to access the singleton instance
LocalSaveClass& LocalSaveClass::getInstance() {
    static LocalSaveClass instance; // Static instance ensures only one instance is created
    return instance;
}
// Private constructor to prevent external instantiation
    LocalSaveClass::LocalSaveClass(){};

    // Private destructor to prevent deletion from outside
    LocalSaveClass::~LocalSaveClass(){};

// Implementation of member functions
void LocalSaveClass::add2dKeyPoint(double x, double y) {
    std::lock_guard<std::mutex> lock(mtx);
    x2d.push_back(x);
    y2d.push_back(y);
    return;
}

void LocalSaveClass::add3dKeyPoint(double x, double y, double z) {
    std::lock_guard<std::mutex> lock(mtx);
    x3d.push_back(x);
    y3d.push_back(y);
    z3d.push_back(z);
    return;
}

void LocalSaveClass::add3dKeyPointWorld(double x, double y, double z) {
    std::lock_guard<std::mutex> lock(mtx);
    x3dWorld.push_back(x);
    y3dWorld.push_back(y);
    z3dWorld.push_back(z);
    return;
}

void LocalSaveClass::reset() {
    std::lock_guard<std::mutex> lock(mtx);
    x2d.clear();
    y2d.clear();
    x3d.clear();
    y3d.clear();
    z3d.clear();
    x3dWorld.clear();
    y3dWorld.clear();
    z3dWorld.clear();
    return;
}

size_t LocalSaveClass::numKeyPoints() const {
    std::lock_guard<std::mutex> lock(mtx);
    return x2d.size(); // Assuming all vectors have the same size
}

std::tuple<double, double, double, double, double, double, double, double> LocalSaveClass::getKeyPoint(size_t index) const {
    std::lock_guard<std::mutex> lock(mtx);
    if (index < x2d.size() && index < x3d.size() && index < y3d.size() && index < z3d.size()) {
        return std::make_tuple(x2d[index], y2d[index], x3d[index], y3d[index], z3d[index], x3dWorld[index], y3dWorld[index], z3dWorld[index]);
    } else {
        return std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
}


cv::Mat LocalSaveClass::getFramePoints(){
    if (x2d.size() > 0){
        cv::Mat conMat = cv::Mat(x2d.size(), 8, CV_32F);
        for (int row = 0; row<x2d.size(); row++){
            conMat.at<float>(row, 0) = float(x2d[row]);
            conMat.at<float>(row, 1) = float(y2d[row]);
            conMat.at<float>(row, 2) = float(x3d[row]);
            conMat.at<float>(row, 3) = float(y3d[row]);
            conMat.at<float>(row, 4) = float(z3d[row]);
            conMat.at<float>(row, 5) = float(x3dWorld[row]);
            conMat.at<float>(row, 6) = float(y3dWorld[row]);
            conMat.at<float>(row, 7) = float(z3dWorld[row]);
        }
        return conMat;
    }
    else {
        cv::Mat conMat = cv::Mat(1, 8, CV_32F);
        return conMat;
    }
};

} // namespace ORB_SLAM3
