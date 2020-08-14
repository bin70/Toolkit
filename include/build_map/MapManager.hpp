#pragma once

#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H

#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>
#include <common.hpp>
#include <point_cloud/common.hpp>

typedef pcl::octree::OctreePointCloudSearch<PointType> OctreePointCloudSearch;
typedef OctreePointCloudSearch::Ptr OctreePtr;

class MapManager
{
public:
  MapManager(float map_resolution)
    : octree(new OctreePointCloudSearch(map_resolution)),
      map(new PointCloud),
      leafsize(map_resolution) {}

  ~MapManager(){}

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
    for (int i = 0; i < frame->points.size(); i++)
    { // Add frame to Global Map
      tempPoint = frame->points[i];
      if (!octree->isVoxelOccupiedAtPoint(tempPoint))
        octree->addPointToCloud(tempPoint, map);
    }
  }

  void addFrame(PointCloud::Ptr frame)
  {
    AddFrameToMap(frame);
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