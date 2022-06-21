 
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <System.h>
#include <Converter.h>
#include <MapPoint.h>

#include <fstream>
#include <iostream>
#include <string>

#include <exception>
#include <typeinfo>
#include <stdexcept>

#include "ndarray_converter.h"

#include<chrono>
#include<thread>

#include <sys/types.h>
#include <unistd.h>

namespace py = pybind11;

class PyOrbSlam
{
public:
  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System* slam; 
  ORB_SLAM3::Converter* conv;
  //PyOrbSlam(){}; 
  PyOrbSlam(std::string path_to_vocabulary, std::string path_to_settings, std::string sensorType, bool useViewer)
	{
    ORB_SLAM3::System::eSensor sensor;
    if (sensorType.compare("Mono")==0){
      sensor = ORB_SLAM3::System::MONOCULAR;
    }
    if (sensorType.compare("Stereo")==0){
      sensor = ORB_SLAM3::System::STEREO;
    }
    if (sensorType.compare("RGBD")==0){
      sensor = ORB_SLAM3::System::RGBD;
    }
    if (sensorType.compare("MonoIMU")==0){
      sensor = ORB_SLAM3::System::IMU_MONOCULAR;
    }
    if (sensorType.compare("StereoIMU")==0){
      sensor = ORB_SLAM3::System::IMU_STEREO;
    }
    if (sensorType.compare("RGBDIMU")==0){
      sensor = ORB_SLAM3::System::IMU_RGBD;
    }
        try{
		  slam = new ORB_SLAM3::System(path_to_vocabulary,path_to_settings,sensor, useViewer);
      conv = new ORB_SLAM3::Converter();
      }
      catch (const std::exception& e)
      {
        ofstream myfile;
        myfile.open ("errorLog.txt");
        myfile << e.what();
        myfile.close();
        //std::cerr << e.what() << std::endl;
        throw runtime_error(e.what());
      }
  };

  ~PyOrbSlam(){
    slam->Shutdown();
    this_thread::sleep_for(chrono::milliseconds(5000));
    delete &slam;
  };

  void Shutdown(){
     try{
        slam->Shutdown();
     }
    catch (const std::exception& e)
      {
        cout << e.what() << endl;
        ofstream myfile;
        myfile.open ("errorLog.txt");
        myfile << e.what();
        myfile.close();
        //std::cerr << e.what() << std::endl;
        throw runtime_error(e.what());
      }
  };

  void saveTrajectory(string filePath){
    // Save camera trajectory
    slam->SaveKeyFrameTrajectoryTUM(filePath);
  };

  void ActivateLocalizationMode(){slam->ActivateLocalizationMode();};
    
  void DeactivateLocalizationMode(){slam->DeactivateLocalizationMode();};

  void Reset(){slam->Reset();};

  std::string getPID(){pid_t pid = getpid();
                  return to_string(pid);}

  void ResetActiveMap(){slam->ResetActiveMap();};

  int GetTrackingState(){return slam->GetTrackingState();};

  bool IsLost(){return slam->isLost();};

  cv::Mat GetTrackedMapPoints(){
    try{
      /*
      Map* pActiveMap = mpAtlas->GetCurrentMap();
    if(!pActiveMap)
        return;

    const vector<MapPoint*> &vpMPs = pActiveMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        Eigen::Matrix<float,3,1> pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos(0),pos(1),pos(2));
    }
    glEnd();

      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      
      cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap && pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    if(!pBiggerMap)
    {
        std::cout << "There is not a map!!" << std::endl;
        return;
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(!pKF || pKF->isBad())
            continue;
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = pKF->GetImuPose();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

        }
        else
        {
            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f t = Twc.translation();
            f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
        }
    }
    f.close();*/
      std::vector<ORB_SLAM3::MapPoint*> mapPoints =  slam->GetTrackedMapPoints();
      cv::Mat positions = conv->toCvMat(mapPoints[0]->GetWorldPos());
      cv::Mat normals = conv->toCvMat(mapPoints[0]->GetNormal());
      
      for (int i = 0; i<mapPoints.size()-1;i++){
        cv::vconcat(positions, conv->toCvMat(mapPoints[i]->GetWorldPos()), positions);
        cv::vconcat(normals,conv->toCvMat(mapPoints[i]->GetNormal()), normals);
      }
      cv::hconcat(positions,normals,positions);
      return positions;
    }
    catch (const std::exception& e)
      {
        cout << e.what() << endl;
        ofstream myfile;
        myfile.open ("errorLog.txt");
        myfile << e.what();
        myfile.close();
        //std::cerr << e.what() << std::endl;
        throw runtime_error(e.what());
      }
    
    // for each
    //Eigen::Vector3f normal =  GetNormal();
    //int observed =  Observations();
    //float foundRation = GetFoundRatio();
    //cv::Mat descr =  GetDescriptor();
    //Eigen::Vector3f position =  GetWorldPos();
    
    };
  /*std::vector<cv::KeyPoint> GetTrackedKeyPoints(){
    
    std::vector<cv::KeyPoint> keypoints =  slam->GetTrackedKeyPointsUn()
    
    };*/
  

  cv::Mat process(cv::Mat &in_image, const double &seconds){
    cv::Mat camPose;
    Sophus::SE3f camPoseReturn;
    g2o::SE3Quat g2oQuat;
    try{
      camPoseReturn = slam->TrackMonocular(in_image,seconds);
      g2oQuat = conv->toSE3Quat(camPoseReturn);
      camPose = conv->toCvMat(g2oQuat);
    }
    catch (const std::exception& e)
      {
        ofstream myfile;
        myfile.open ("errorLog.txt");
        myfile << e.what();
        myfile.close();
        throw  runtime_error(e.what());
        //std::cerr << e.what() << std::endl;
      }
    return camPose;
  }
};

PYBIND11_MODULE(pyOrbSlam, m)
{
	NDArrayConverter::init_numpy();

	py::class_<PyOrbSlam>(m, "OrbSlam")
    //.def(py::init())
		.def(py::init<string,string, string,bool>(), py::arg("path_to_vocabulary"), py::arg("path_to_settings"), py::arg("sensorType")="Mono", py::arg("useViewer")=false)
		.def("saveTrajectory", &PyOrbSlam::saveTrajectory, py::arg("filePath"))
		.def("process", &PyOrbSlam::process, py::arg("in_image"), py::arg("seconds"))
    .def("ActivateLocalizationMode", &PyOrbSlam::ActivateLocalizationMode)
    .def("DeactivateLocalizationMode", &PyOrbSlam::DeactivateLocalizationMode)
    .def("Reset", &PyOrbSlam::Reset)
    .def("ResetActiveMap", &PyOrbSlam::ResetActiveMap)
    .def("GetTrackingState", &PyOrbSlam::GetTrackingState)
    .def("IsLost", &PyOrbSlam::IsLost)
    .def("GetTrackedMapPoints", &PyOrbSlam::GetTrackedMapPoints)
    .def("getPID",&PyOrbSlam::getPID)
    .def("shutdown",&PyOrbSlam::Shutdown);
};
