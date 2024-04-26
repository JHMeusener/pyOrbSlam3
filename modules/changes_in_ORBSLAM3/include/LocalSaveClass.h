/**
* This is a class to store keypoints from orb and the corresponding local 3d map points of the frame
*/
#include <vector>
#include <tuple> // For std::tuple
#include <mutex> // For std::mutex
#include <opencv2/core.hpp>

namespace ORB_SLAM3 {

class LocalSaveClass {
public:
    // Private constructor to prevent external instantiation
    LocalSaveClass();

    // Private destructor to prevent deletion from outside
    ~LocalSaveClass();

    // Static method to access the singleton instance
    static LocalSaveClass& getInstance();

    void add2dKeyPoint(double x, double y);
    void add3dKeyPoint(double x, double y, double z);
    void add3dKeyPointWorld(double x, double y, double z);
    void reset();
    size_t numKeyPoints() const;
    std::tuple<double, double, double, double, double, double, double, double> getKeyPoint(size_t index) const;
    cv::Mat getFramePoints();
private:
    
    // Private copy constructor and assignment operator to prevent copying
    LocalSaveClass(const LocalSaveClass&) = delete;
    LocalSaveClass& operator=(const LocalSaveClass&) = delete;

    // Private member variables
    std::vector<double> x2d;
    std::vector<double> y2d;
    std::vector<double> x3d;
    std::vector<double> y3d;
    std::vector<double> z3d;

    std::vector<double> x3dWorld;
    std::vector<double> y3dWorld;
    std::vector<double> z3dWorld;

    mutable std::mutex mtx;
};

} // namespace ORB_SLAM3
