#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>

// #include "addressbook.struct_pb.h"
#include "imu.struct_pb.h"

#include "xyzit_points.hpp"
#include <pcl/point_cloud.h>

#include <pcl/io/pcd_io.h>

// #include <pcl/compression/organized_pointcloud_compression.h>
// #include <pcl/io/impl/organized_pointcloud_compression.hpp>

#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/io/impl/octree_pointcloud_compression.hpp>

#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/impl/octree_pointcloud.hpp>




int main() {

  inner_struct::ZImu imu1{1,2,3,inner_struct::ZFrameType::IMU,"channel_name", {4,5,6}, {7,8,9}, 10};
  inner_class::ZImu imu2 = converter::StructToClass(imu1);
  inner_struct::ZImu imu3 = converter::ClassToStruct(imu2);


  // std::cout << imu1 << std::endl;
  // std::cout << imu2.ShortDebugString() << std::endl;
  // std::cout << imu3 << std::endl;


  // inner_struct::ZPointCloudXYZI pc1;
  // pc1.points.push_back(pcl::PointXYZI());
  // pc1.points.push_back(pcl::PointXYZI());
  // pcl::PointXYZI p1;
  // pcl::PointXYZI p1 {1};
  // std::cout << p1 << std::endl;

  pcl::PointXYZIT p2 {1,2};
  // std::cout << p2 << std::endl;


  pcl::PointCloud<pcl::PointXYZIT>::Ptr pc = std::make_shared<pcl::PointCloud<pcl::PointXYZIT>>();
  pc->push_back(p2);
  pc->push_back(p2);
  pc->push_back(p2);
  pc->push_back(p2);

  std::cout<< pc->width <<" " << pc->height << std::endl;


  pcl::io::OctreePointCloudCompression<pcl::PointXYZIT> compress;

  std::stringstream ss;

  compress.encodePointCloud(pc,ss);

  std::cout<<" str = " << ss.str() << std::endl;


  // pc2
  pcl::PointCloud<pcl::PointXYZIT>::Ptr pc2 = std::make_shared<pcl::PointCloud<pcl::PointXYZIT>>();
  compress.decodePointCloud(ss,pc2);

  std::cout << *pc2 << std::endl;


  std::cout << "Done!!!" << std::endl;

  return 0;
}
