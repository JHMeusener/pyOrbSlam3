 
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

class Debug
{
  public:
  Debug(){};
  ~Debug(){};
  std::string getPID(){pid_t pid = getpid();
                  return to_string(pid);}
};

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
    delete &conv;
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

  void ResetActiveMap(){slam->ResetActiveMap();};

  int GetTrackingState(){return slam->GetTrackingState();};

  bool IsLost(){return slam->isLost();};

  int getNrOfMaps(){
      vector<ORB_SLAM3::Map*> vpMaps = slam->mpAtlas->GetAllMaps();
      return vpMaps.size();
  }

  cv::Mat GetTrackedMapReferencePointsOfMap(int mapNr){
    try{
      ORB_SLAM3::Map* pActiveMap;
      if (mapNr == -1){
        pActiveMap = slam->mpAtlas->GetCurrentMap();
      }
      else {
        vector<ORB_SLAM3::Map*> vpMaps = slam->mpAtlas->GetAllMaps();
        pActiveMap = vpMaps[mapNr];
      }
      if(!pActiveMap)
        return  cv::Mat(1,3,CV_32FC1, 0.0f);
    const vector<ORB_SLAM3::MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();
    set<ORB_SLAM3::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());
    if(vpRefMPs.empty())
        return cv::Mat(1,3,CV_32FC1, 0.0f);
    cv::Mat positions = cv::Mat(vpRefMPs.size(),40,CV_32FC1, 0.0f);
    for(size_t i=0, iend=vpRefMPs.size(); i<iend;i++)
      {
          if(vpRefMPs[i]->isBad())
              continue;
          Eigen::Matrix<float,3,1> pos = vpRefMPs[i]->GetWorldPos();
          //glVertex3f(pos(0),pos(1),pos(2));
          positions.at<float>(i,0) = pos(0);
          positions.at<float>(i,1) = pos(1);
          positions.at<float>(i,2) = pos(2);
          Eigen::Matrix<float,3,1> norm = vpRefMPs[i]->GetNormal();
          //glVertex3f(pos(0),pos(1),pos(2));
          positions.at<float>(i,3) = norm(0);
          positions.at<float>(i,4) = norm(1);
          positions.at<float>(i,5) = norm(2);
          positions.at<float>(i,6) = float(vpRefMPs[i]->Observations());
          positions.at<float>(i,7) = vpRefMPs[i]->GetFoundRatio();
          cv::Mat descr =  vpRefMPs[i]->GetDescriptor();
          for (int z = 0; z<32; z++){
            positions.at<float>(i,8+z) = float(descr.at<unsigned char>(z));
          }
      }
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
  }


    cv::Mat GetTrackedMapPointsOfMap(int mapNr){
    try{
      ORB_SLAM3::Map* pActiveMap;
      if (mapNr == -1){
        pActiveMap = slam->mpAtlas->GetCurrentMap();
      }
      else {
        vector<ORB_SLAM3::Map*> vpMaps = slam->mpAtlas->GetAllMaps();
        pActiveMap = vpMaps[mapNr];
      }
      if(!pActiveMap)
        return  cv::Mat(1,3,CV_32FC1, 0.0f);

    const vector<ORB_SLAM3::MapPoint*> &vpMPs = pActiveMap->GetAllMapPoints();
    const vector<ORB_SLAM3::MapPoint*> &vpRefMPs = pActiveMap->GetReferenceMapPoints();

    set<ORB_SLAM3::MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return cv::Mat(1,3,CV_32FC1, 0.0f);
    cv::Mat positions = cv::Mat(vpMPs.size(),40,CV_32FC1, 0.0f);
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
      {
          if(vpMPs[i]->isBad() )//|| spRefMPs.count(vpMPs[i]))
              continue;
          Eigen::Matrix<float,3,1> pos = vpMPs[i]->GetWorldPos();
          //glVertex3f(pos(0),pos(1),pos(2));
          positions.at<float>(i,0) = pos(0);
          positions.at<float>(i,1) = pos(1);
          positions.at<float>(i,2) = pos(2);
          Eigen::Matrix<float,3,1> norm = vpMPs[i]->GetNormal();
          //glVertex3f(pos(0),pos(1),pos(2));
          positions.at<float>(i,3) = norm(0);
          positions.at<float>(i,4) = norm(1);
          positions.at<float>(i,5) = norm(2);
          positions.at<float>(i,6) = float(vpMPs[i]->Observations());
          positions.at<float>(i,7) = vpMPs[i]->GetFoundRatio();
          cv::Mat descr =  vpMPs[i]->GetDescriptor();
          for (int z = 0; z<32; z++){
            positions.at<float>(i,8+z) = float(descr.at<unsigned char>(z));
          }
      }
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
  }

  cv::Mat getKeyFramesOfMap(int mapNr, bool withIMU){
      try{
            ORB_SLAM3::Map* pActiveMap;
            if (mapNr == -1){
              pActiveMap = slam->mpAtlas->GetCurrentMap();
            }
            else {
              vector<ORB_SLAM3::Map*> vpMaps = slam->mpAtlas->GetAllMaps();
              pActiveMap = vpMaps[mapNr];
            }
            if(!pActiveMap)
              return  cv::Mat(1,3,CV_32FC1, 0.0f);
            vector<ORB_SLAM3::KeyFrame*> vpKFs = pActiveMap->GetAllKeyFrames();
            sort(vpKFs.begin(),vpKFs.end(),ORB_SLAM3::KeyFrame::lId);
            cv::Mat keyPositions = cv::Mat(vpKFs.size(),7,CV_32FC1, 0.0f);
            for(size_t i=0; i<vpKFs.size(); i++)
            {
                ORB_SLAM3::KeyFrame* pKF = vpKFs[i];

              // pKF->SetPose(pKF->GetPose()*Two);

                if(!pKF || pKF->isBad())
                    continue;
                if (withIMU)
                {
                    Sophus::SE3f Twb = pKF->GetImuPose();
                    Eigen::Quaternionf q = Twb.unit_quaternion();
                    Eigen::Vector3f twb = Twb.translation();
                    keyPositions.at<float>(i,0) = twb(0);
                    keyPositions.at<float>(i,1) = twb(1);
                    keyPositions.at<float>(i,2) = twb(2);
                    keyPositions.at<float>(i,3) = q.x();
                    keyPositions.at<float>(i,4) = q.y();
                    keyPositions.at<float>(i,5) = q.z();
                    keyPositions.at<float>(i,6) = q.w();
                }
                else
                {
                    Sophus::SE3f Twc = pKF->GetPoseInverse();
                    Eigen::Quaternionf q = Twc.unit_quaternion();
                    Eigen::Vector3f t = Twc.translation();
                    keyPositions.at<float>(i,0) = t(0);
                    keyPositions.at<float>(i,1) = t(1);
                    keyPositions.at<float>(i,2) = t(2);
                    keyPositions.at<float>(i,3) = q.x();
                    keyPositions.at<float>(i,4) = q.y();
                    keyPositions.at<float>(i,5) = q.z();
                    keyPositions.at<float>(i,6) = q.w();
                }
            }
            return keyPositions;
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
  }
            
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
  py::class_<Debug>(m, "Debug")
    .def(py::init())
    .def("getPID",&Debug::getPID);
    
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
    .def("GetTrackedMapPoints", &PyOrbSlam::GetTrackedMapPointsOfMap, py::arg("mapNr")=-1)
    .def("GetTrackedMapReferencePoints", &PyOrbSlam::GetTrackedMapReferencePointsOfMap, py::arg("mapNr")=-1)
    .def("getNrOfMaps", &PyOrbSlam::getNrOfMaps)
    .def("getKeyFramesOfMap", &PyOrbSlam::getKeyFramesOfMap, py::arg("mapNr")=-1, py::arg("withIMU") = false)
    .def("shutdown",&PyOrbSlam::Shutdown);
};
