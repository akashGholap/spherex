#ifndef SPHEREX_PP_H
#define SPHEREX_PP_H
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include "dji_sdk/setNextHop.h"
#include <std_msgs/Bool.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <Eigen/Geometry>

//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
using namespace std;

struct Sector
{
  pcl::PointIndices::Ptr indices;
  bool indicate;
  int sector_number;
  Sector()
  :indices(new pcl::PointIndices), indicate(true), sector_number(0)
  {}
};
struct Candidate
{
int begin;
int end;
float phi;


};
struct Plane
{
  std::vector<Sector, Eigen::aligned_allocator<Sector>> sectors;
  Plane()
  :sectors()
  {}
};

struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};

class SphereX
{
  public:
  bool hop_status;
	bool icp_status;
  float d;
  float theta;
  float phi;
  float od;
  float otheta;
  float ophi;   //old
  float ox,oy,oz; //old ones
  float x, y, z; //current
  float nxp, nyp, nzp; //predicted
  float preVecPhi;

	SphereX() : hop_status(false),icp_status(false),d(0.0),theta(0.0),phi(0.0),od(0.0),otheta(0.0),ophi(0.0),ox(0.0),oy(0.0),oz(0.0),x(0.0),y(0.0),z(0.0),nxp(0.0),nyp(0.0),nzp(0.0),preVecPhi(0.0)
  {}

};


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};
void storefilename_callback(const std_msgs::String& pcd_file_name);

void loadData_from_txt (std::vector<PCD, Eigen::aligned_allocator<PCD> > &models);

void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample);

bool set_next_hop_callback(dji_sdk::setNextHop::Request &req, dji_sdk::setNextHop::Response &res);

void hop_status_callback(const std_msgs::Bool &status);

bool compute_next_hop();
void point_cloud_callback (const sensor_msgs::PointCloud2 &cloud_msg);

#endif
