#include "CameraRockRGBD.hpp"
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/utilite/UTimer.h>


#define SERIAL_DEFAULT "rock_camera_rgbd"

namespace rgbd_slam{

CameraRockRGBD::CameraRockRGBD( float imageRate,
                  const rtabmap::Transform &localTransform)
                  : Camera(imageRate, localTransform),
                    max_range_(5.0f),
                    serial_(SERIAL_DEFAULT) {
}

CameraRockRGBD::CameraRockRGBD(
                 float max_range,
                 std::string serial,
                 float imageRate,
                 const rtabmap::Transform &localTransform)
                 : Camera(imageRate, localTransform),
                   max_range_(max_range),
                   serial_(serial){
}

CameraRockRGBD::~CameraRockRGBD(){
}


bool CameraRockRGBD::init(const std::string &calibrationFolder,
                          const std::string &cameraName){
  return true;
}

bool CameraRockRGBD::isCalibrated() const {
  return true;
}

std::string CameraRockRGBD::getSerial() const{
  return "cameraRockRGBD_";
}

rtabmap::SensorData CameraRockRGBD::captureImage(rtabmap::CameraInfo *info){

  rtabmap::CameraModel model(
		632, //fx
		348, //fy
		float(rgb_image_.cols/2) - 0.5f,  //cx
		float(rgb_image_.rows/2) - 0.5f,  //cy
		this->getLocalTransform(),
		0,
		rgb_image_.size());

	rtabmap::SensorData data = rtabmap::SensorData(rgb_image_, depth_image_,
                                                 model, this->getNextSeqID(),
                                                 UTimer::now());
	// depth_ = cv::Mat();
	// rgb_ = cv::Mat();
	return data;
}


}
