#pragma once

#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H

#include <pcl/octree/octree_search.h>
#include <pcl/filters/voxel_grid.h>
#include <common.hpp>
#include <point_cloud/common.hpp>

typedef pcl::octree::OctreePointCloudSearch<PointType> OctreePointCloudSearch;
typedef OctreePointCloudSearch::Ptr OctreePtr;

class OctreeManager
{
public:
  OctreeManager(float map_resolution)
    : octree(new OctreePointCloudSearch(map_resolution)),
      map(new PointCloud),
      leafsize(map_resolution) {}

  ~OctreeManager(){}

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

  void addPointCloud(PointCloud::Ptr cloud)
  {
    for (int i = 0; i < cloud->points.size(); i++)
    { // Add cloud to Global Map
      if (!octree->isVoxelOccupiedAtPoint(cloud->points[i]))
        octree->addPointToCloud(cloud->points[i], map);
    }
  }

  void updateByPointCloud(PointCloud::Ptr cloud)
  {
    octree->deleteTree();
    *map = *cloud;
    PointCloud::Ptr map_filtered(new PointCloud);
    grid.setInputCloud(map);
    grid.setLeafSize(leafsize, leafsize, leafsize);
    grid.filter(*map_filtered);

    *map = *map_filtered;
    octree->setInputCloud(map);
    octree->addPointsFromInputCloud();
  }
  const PointCloud::Ptr& getPointCloudPtr() { return map; }

private:
  OctreePtr octree;
  PointCloud::Ptr map;
  pcl::VoxelGrid<PointType> grid;
  double leafsize;
};
#endif