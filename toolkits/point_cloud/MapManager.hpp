#pragma once

#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <cstdlib>
#include <stdlib.h>
#include <string.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZI PointType;
//typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::octree::OctreePointCloudSearch<PointType> OctreePointCloudSearch;
typedef OctreePointCloudSearch::Ptr OctreePtr;

class MapManager
{
public:
  MapManager(){}

  MapManager(float map_resolution)
    : octree(new OctreePointCloudSearch(map_resolution)),
      map(new PointCloud), leafsize(map_resolution) {
        octree->setInputCloud(map);
        octree->addPointsFromInputCloud();
      }

  ~MapManager(){}

  void init(double resolution){
    octree = boost::shared_ptr<OctreePointCloudSearch>(new OctreePointCloudSearch(resolution));
    map = boost::shared_ptr<PointCloud>(new PointCloud());
    leafsize = resolution;
    octree->setInputCloud(map);
    octree->addPointsFromInputCloud();
  }

  void update()
  {
    octree->setInputCloud(map);
    octree->addPointsFromInputCloud();
  }

  // 通过八叉树搜索最近邻
  bool getNearestPoint(PointType &pointSch, PointType &pointSel)
  {
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;

    // K = 1
    if (octree->isVoxelOccupiedAtPoint(pointSch)
        && octree->nearestKSearch(pointSch, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
      pointSel = octree->getInputCloud()->points[pointIdxNKNSearch[0]];
      return true;
    }
    return false;
  }

  // 直接搜索体素
  // bool voxelSearch(PointType &pointSch, PointType &pointSel)
  // {
  //   std::vector<int> pointInVoxel;
  //   if (octree->isVoxelOccupiedAtPoint(pointSch))
  //   {
  //     if(octree->voxelSearch(pointSch, pointInVoxel))
  //     {
  //       std::cout << "在体素内找到 " << pointInVoxel.size() << " 个点." << std::endl;
  //       pointSel = octree->getPointByIndex(pointInVoxel[0]); //取第一个点
  //       return true;
  //     }
  //   }
  //   return false;
  // }

  void AddFrameToMap(PointCloud::Ptr frame)
  {
    PointType tempPoint;
    for (unsigned int i = 0; i < frame->points.size(); i++)
    { // Add frame to Global Map
      tempPoint = frame->points[i];
      if (!octree->isVoxelOccupiedAtPoint(tempPoint))
        octree->addPointToCloud(tempPoint, map);
    }
  }

  // 只是对AddFrameToMap的wrapper
  void addFrame(PointCloud::Ptr frame)
  {
    AddFrameToMap(frame);
  }

  void setNewMap(PointCloud::Ptr frame)
  {
    octree->deleteTree();
    *map = *frame;
    octree->setInputCloud(map);
    octree->addPointsFromInputCloud();
  }

  void UpdateMap(PointCloud::Ptr cur_frame)
  {
    octree->deleteTree();
    *map = *cur_frame;
    PointCloud::Ptr map_filtered(new PointCloud);
    grid.setInputCloud(map);
    grid.setLeafSize(leafsize, leafsize, leafsize);
    grid.filter(*map_filtered);

    *map = *map_filtered;
    octree->setInputCloud(map);
    octree->addPointsFromInputCloud();
  }
  const PointCloud::Ptr& getMapPtr() { return map; }

private:
  OctreePtr octree;
  PointCloud::Ptr map;
  pcl::VoxelGrid<PointType> grid;
  double leafsize;
};
#endif