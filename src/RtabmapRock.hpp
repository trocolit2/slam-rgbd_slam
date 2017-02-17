#include "CameraRockRGBD.hpp"

#include <rtabmap/core/OdometryF2M.h>
#include <rtabmap/core/Rtabmap.h>

namespace rgbd_slam{

class RtabmapRock {

public:
  RtabmapRock();
  RtabmapRock(CameraRockRGBD *camera);

  virtual ~RtabmapRock();

  void rtabmapRun(CameraRockRGBD *camera);
  CameraRockRGBD getCameraRGBDRock() const;

  void setOdometryUpdate(int odometry_update){odometry_update_ = odometry_update;}
  void setMappingUpdate(int mapping_update){mapping_update_ = mapping_update;}

private:

  CameraRockRGBD *camera_;
  rtabmap::Rtabmap *rtabmap_;
  rtabmap::OdometryF2M *odometry_;

  int camera_iteration_;
  int odometry_update_;
  int odometry_iteratrion_;
  int mapping_update_;

};

}
