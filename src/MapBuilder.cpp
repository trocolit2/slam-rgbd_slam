#include "MapBuilder.hpp"
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/OdometryInfo.h>

namespace rgbd_slam{

#define CLOUD_DECIMATION 4
#define CLOUD_MAX_DEPTH 5.0f

MapBuilder::MapBuilder(CameraRockRGBD *camera)
  : rtabmap_ (new rtabmap::Rtabmap()),
    odometry_ (new rtabmap::OdometryF2M()),
    lastOdomPose_(rtabmap::Transform::getIdentity()),
    odometryCorrection_( rtabmap::Transform::getIdentity() ){

  camera_ = camera;
  camera_->init("","");
  rtabmap_->init();
}

MapBuilder::MapBuilder(){
  MapBuilder(new CameraRockRGBD());
}


MapBuilder::~MapBuilder(){

  delete camera_;
  delete odometry_;
  delete rtabmap_;

}

void MapBuilder::mapping(CameraRockRGBD *camera){

  rtabmap::SensorData data = camera->takeImage();

  if(data.isValid()){
    if((camera_iteration++ % odometry_update) == 0){
      rtabmap::OdometryInfo info;
      rtabmap::Transform pose = odometry_->process(data, &info);

      if((odometry_iteratrion++ % mapping_update) == 0) {
        if(rtabmap_->process(data, pose)) {
          // mapBuilder.processStatistics(rtabmap.getStatistics());
          if(rtabmap_->getLoopClosureId() > 0)
            std::cout << "Loop closure detected!" << std::endl;
          }
        }
    }

    // mapBuilder.processOdometry(data, pose, info);
  }

};

// void MapBuilder::processStatitics(const rtabmap::Statistics &stats){
//
  //============================
  // Add RGB-D clouds
  //============================
  // const std::map<int, rtabmap::Transform> &poses = stats.poses();
  // std::map<std::string, rtabmap::Transform> clouds = addedClouds_;
  // for(std::map<int, rtabmap::Transform>::const_iterator iter = poses.begin();
  //     iter!=poses.end(); ++iter){
  //   if(!iter->second.isNull()){
  //     std::string cloudName = uFormat("cloud%d", iter->first);
  //
  //     // 3d point cloud
  //     if(addedClouds_.find(id) != addedClouds_.end()){
  //       // Update only if the pose has changed
  //       rtabmap::Transform tCloud;
  //       cloudViewer_->getPose(cloudName, tCloud);
  //       if(tCloud.isNull() || iter->second != tCloud)
  //       {
  //         if(!cloudViewer_->updateCloudPose(cloudName, iter->second))
  //         {
  //           UERROR("Updating pose cloud %d failed!", iter->first);
  //         }
  //       }
  //       cloudViewer_->setCloudVisibility(cloudName, true);
  //     }
  //     else if(uContains(stats.getSignatures(), iter->first))
  //     {
  //       Signature s = stats.getSignatures().at(iter->first);
  //       s.sensorData().uncompressData(); // make sure data is uncompressed
  //       // Add the new cloud
  //       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = util3d::cloudRGBFromSensorData(
  //           s.sensorData(),
  //           4,     // decimation
  //           4.0f); // max depth
  //       if(cloud->size())
  //       {
  //         if(!cloudViewer_->addCloud(cloudName, cloud, iter->second))
  //         {
  //           UERROR("Adding cloud %d to viewer failed!", iter->first);
  //         }
  //       }
  //       else
  //       {
  //         UWARN("Empty cloud %d!", iter->first);
  //       }
  //     }
  //   }
  //   else
  //   {
  //     UWARN("Null pose for %d ?!?", iter->first);
  //   }
  // }
  //
  // //============================
  // // Add 3D graph (show all poses)
  // //============================
  // cloudViewer_->removeAllGraphs();
  // cloudViewer_->removeCloud("graph_nodes");
  // if(poses.size())
  // {
  //   // Set graph
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr graph(new pcl::PointCloud<pcl::PointXYZ>);
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr graphNodes(new pcl::PointCloud<pcl::PointXYZ>);
  //   for(std::map<int, Transform>::const_iterator iter=poses.begin(); iter!=poses.end(); ++iter)
  //   {
  //     graph->push_back(pcl::PointXYZ(iter->second.x(), iter->second.y(), iter->second.z()));
  //   }
  //   *graphNodes = *graph;
  //
  //
  //   // add graph
  //   cloudViewer_->addOrUpdateGraph("graph", graph, Qt::gray);
  //   cloudViewer_->addCloud("graph_nodes", graphNodes, Transform::getIdentity(), Qt::green);
  //   cloudViewer_->setCloudPointSize("graph_nodes", 5);
  // }
  //
  // odometryCorrection_ = stats.mapCorrection();
  //
  // cloudViewer_->update();
  //
  //
  // }
  //
// }

// void MapBuilder::processOdometry( const rtabmap::SensorData &data,
//                                   rtabmap::Transform pose,
//                                   const rtabmap::OdometryInfo &odom){
//
//   if(pose.isNull())
//     pose = lastOdomPose_; //Odometry lost
//   else{
//     lastOdomPose_ = pose; //Start odometry
//
//     // 3d cloud
//     if(data.depthOrRightRaw().cols == data.imageRaw().cols &&
//        data.depthOrRightRaw().rows == data.imageRaw().rows &&
//        !data.depthOrRightRaw().empty() &&
//        (data.stereoCameraModel().isValidForProjection() ||
//           data.cameraModels().size())){
//
//       pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
//       cloud = rtabmap::util3d::cloudRGBFromSensorData( data,
//                                                         CLOUD_DECIMATION,
//                                                         CLOUD_MAX_DEPTH);
//       // need add all cloud, look rtabmap cloudViewer addCloud Function
//       }
//     }
// }
//
//
// bool MapBuilder::addCloud(  const std::string &id,
//                             const pcl::PCLPointCloud2Ptr &binaryCloud,
//                             const rtabmap::Transform &pose){
//
//   if(addedClouds_.find(id) != addedClouds_.end()){
//   	removeCloud(id);
//   }
//
//   addedClouds_.insert( std::pair<std::string,rtabmap::Transform>(id,pose));
//   return true;
// }
//
// bool MapBuilder::removeCloud(const std::string &id){
//   auto map_search = addedClouds_.find(id);
//   addedClouds_.erase(map_search); // remove after visualizer
//   return true;
// }


} // namespace rtabmap
