#include "CameraRockRGBD.hpp"

#include <rtabmap/core/OdometryF2M.h>
#include <rtabmap/core/Rtabmap.h>
#include <pcl/point_types.h>
#include <map>

namespace rgbd_slam{

class MapBuilder {

public:
  MapBuilder();
  MapBuilder(CameraRockRGBD *camera);

  virtual ~MapBuilder();

  void mapping(CameraRockRGBD *camera);

  CameraRockRGBD getCameraRGBDRock() const;

protected:

  CameraRockRGBD *camera_;
  rtabmap::Rtabmap *rtabmap_;
  rtabmap::OdometryF2M *odometry_;
  rtabmap::Transform lastOdomPose_;
	rtabmap::Transform odometryCorrection_;

  void processStatitics(const rtabmap::Statistics &stats);

  void processOdometry( const rtabmap::SensorData &data,
                        rtabmap::Transform pose,
                        const rtabmap::OdometryInfo &odom);

private:

  int camera_iteration;
  int odometry_update;
  int odometry_iteratrion;
  int mapping_update;

   // maybe split in util file functions
  // std::map<std::string, rtabmap::Transform> addedClouds_;
  //
  // bool addCloud(const std::string &id,
  //           		const pcl::PCLPointCloud2Ptr &binaryCloud,
  //           		const rtabmap::Transform &pose);
  //
  // bool removeCloud(const std::string &id);

};

}
