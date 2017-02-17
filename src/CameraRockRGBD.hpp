#pragma once

#include <rtabmap/core/RtabmapExp.h> // DLL export/import defines
#include <rtabmap/core/Camera.h>

namespace rgbd_slam {

class RTABMAP_EXP CameraRockRGBD : public rtabmap::Camera {

public:

	CameraRockRGBD( float imageRate=0.0f,
                  const rtabmap::Transform &localTransform
                        = rtabmap::Transform::getIdentity());
  CameraRockRGBD( float max_range,
                  std::string serial,
                  float imageRate=0.0f,
                  const rtabmap::Transform &localTransform
                        = rtabmap::Transform::getIdentity());

	virtual ~CameraRockRGBD();

  // Virtual functions required from rtabmap::Camera class
	virtual bool init(const std::string & calibrationFolder = ".",
                    const std::string & cameraName = "");
	virtual bool isCalibrated() const;
	virtual std::string getSerial() const;
  static bool available(){return true;}

  void setRGBImage(cv::Mat &rgb_image){rgb_image_ = rgb_image;}
  void setDepthImage(cv::Mat &depth_image){depth_image_ = depth_image;}
  void setMaxRange(float max_range){max_range_ = max_range_;}

  rtabmap::SensorData takeData(){ return captureImage(); }

protected:
  // output RGBD camera data
	virtual rtabmap::SensorData captureImage(rtabmap::CameraInfo * info = 0);

private:
  cv::Mat rgb_image_; // CV_8UC3 data type
  cv::Mat depth_image_; // CV_32UC1 data type in meter
  float max_range_;
  std::string serial_;
};


}
