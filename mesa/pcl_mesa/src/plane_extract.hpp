/*
 *  Jun 19, 2015, David Z
 *  Plane extraction, 
 *    input a point cloud, 
 *    output the parameter of this plane
 * */

#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

template<typename PointT>
CPlaneExtract<PointT>::CPlaneExtract()
{
  minimal_plane_points_ = 5000; // 30% * 144 * 176 ~ 7000
  distance_threshold_ = 0.03;   // search distance 3 cm
}

template<typename PointT>
CPlaneExtract<PointT>::~CPlaneExtract(){}

template<typename PointT>
bool CPlaneExtract<PointT>::extract( CloudPtr& in, Eigen::Vector4f& nv)
{
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices); 
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); 
  
  bool ret = extract(in, inliers, coefficients); 
  nv[0] = coefficients->values[0];
  nv[1] = coefficients->values[1];
  nv[2] = coefficients->values[2]; 
  nv[3] = coefficients->values[3]; 
  
  if(nv[1] < 0) // ny<0, 
    nv*=-1.;

  return ret;
}

template<typename PointT>
bool CPlaneExtract<PointT>::extract( CloudPtr& in, 
    pcl::PointIndices::Ptr& inliers, pcl::ModelCoefficients::Ptr& coefficients)
{ 
  
  cerr<<"plane_extract.hpp: start to extract the floor !"<<endl;

  pcl::SACSegmentation<PointT> seg;
  cerr<<"what1"<<endl;
  seg.setOptimizeCoefficients(true);
  cerr<<"what2"<<endl;
  seg.setModelType(pcl::SACMODEL_PLANE);    
  cerr<<"what3"<<endl;
  seg.setMethodType(pcl::SAC_RANSAC);
  cerr<<"plane_extract.hpp: set DistanceThrehold"<<endl;
  seg.setDistanceThreshold(distance_threshold_);
  cerr<<"plane_extract.hpp: setInputCloud"<<endl;
  seg.setInputCloud(in);

  cerr<<"plane_extract.hpp: what' going on!"<<endl;
  seg.segment(*inliers, *coefficients);

  cerr<<"plane_extract.hpp: finish extract the floor !"<<endl;

  if(inliers->indices.size() < minimal_plane_points_) 
    return false;
  return true;
}

template<typename PointT>
bool CPlaneExtract<PointT>::extractFloor(CloudPtr& in, CloudPtr& out)
{
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices); 
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients); 
  
  bool ret = extract(in, inliers, coefficients); 
  
  if(!ret)
  {
    return false;
  }
  
  for(int i=0; i<inliers->indices.size(); ++i)
  {
    PointT& pt = in->points[inliers->indices[i]]; 
    out->points.push_back(pt);
  }
  out->width = inliers->indices.size(); 
  out->height = 1; 
  return true;
}

template<typename PointT>
float CPlaneExtract<PointT>::getPitchByGround(CloudPtr& in)
{
  Eigen::Vector4f nv; 
  // filter first 
  CloudPtr filtered(new Cloud); 
  // filterPointCloud(in, filtered);
  nanFilter(in, filtered);

  if(!extract(filtered, nv))
  {
    return -1; 
  }
  return getPitchByGround(nv);
}

template<typename PointT>
float CPlaneExtract<PointT>::getPitchByGround(Eigen::Vector4f& nv) // align plane to the ground, and extract pitch angle 
{
  /* coordinate reference, ground normal would be {0, 1, 0}
    y   x
    |  /
    | /
    |/____ z  
  */
  float nz = fabs(nv[2]);
  if(nz < 1e-4)
  {
    return 0;
  }else
  {
    return atan2(nv[1], nz);
  }
  return -1;
}

template<typename PointT>
void CPlaneExtract<PointT>::nanFilter(CloudPtr& in, CloudPtr& out)
{
  for(int i=0; i<in->points.size(); i++)
  {
    PointT& pt = in->points[i]; 
    if(pt.x != pt.x || pt.y != pt.y || pt.z != pt.z)
    {
      continue; 
    }
    out->points.push_back(pt);
  }
  out->width = out->points.size(); 
  out->height = 1;
  return; 
}

template<typename PointT>
void CPlaneExtract<PointT>::filterPointCloud(CloudPtr& in, CloudPtr& out)
{
  CloudPtr tmp(new pcl::PointCloud<PointT>);
  // passthrough filter
  pcl::PassThrough<PointT > pass;
  pass.setInputCloud(in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, depth_limit_);
  pass.filter(*tmp);
  // g_passFilter<PointT>(in, tmp, 0.0, _depth_limit, "z");

  // voxelgrid filter
  pcl::VoxelGrid<PointT> vog;
  vog.setInputCloud(tmp); 
  vog.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  vog.filter(*out);
  return ;
}


