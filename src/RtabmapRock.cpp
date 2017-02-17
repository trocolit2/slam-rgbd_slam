#include "RtabmapRock.hpp"
#include <rtabmap/core/OdometryInfo.h>

namespace rgbd_slam{

#define CLOUD_DECIMATION 4
#define CLOUD_MAX_DEPTH 5.0f

RtabmapRock::RtabmapRock(CameraRockRGBD *camera)
  : rtabmap_ (new rtabmap::Rtabmap()),
    odometry_ (new rtabmap::OdometryF2M()) {

  camera_ = camera;
  camera_->init("","");
  rtabmap_->init();

  camera_iteration_ = 0;
  odometry_iteratrion_ = 0;
  odometry_update_ = 5;
  mapping_update_ = 10;
}

RtabmapRock::RtabmapRock(){
  RtabmapRock(new CameraRockRGBD());
}

RtabmapRock::~RtabmapRock(){

  delete camera_;
  delete odometry_;
  delete rtabmap_;
}

void RtabmapRock::rtabmapRun(CameraRockRGBD *camera){

  rtabmap::SensorData data = camera->takeImage();

  if(data.isValid()){
    if((camera_iteration_++ % odometry_update_) == 0){
      rtabmap::OdometryInfo info;
      rtabmap::Transform pose = odometry_->process(data, &info);

      if((odometry_iteratrion_++ % mapping_update_) == 0) {
        if(rtabmap_->process(data, pose)) {
          // mapBuilder.processStatistics(rtabmap.getStatistics());
          if(rtabmap_->getLoopClosureId() > 0)
            std::cout << "Loop closure detected!" << std::endl;
          }
        }

        std::cout << "RTABMAP POSE = ["
                  << pose.x() <<", "
                  << pose.y() <<", "
                  << pose.z() <<"]"
                  << std::endl;
    }

    // mapBuilder.processOdometry(data, pose, info);
  }

}

} // namespace rtabmap
