#include "zpointcloud.hpp"

#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/octree/octree_pointcloud.h>

#include <pcl/io/impl/octree_pointcloud_compression.hpp>
#include <pcl/octree/impl/octree_pointcloud.hpp>

namespace zhito{


namespace converter {
// StructToClass declaration

// static pcl::io::OctreePointCloudCompression<pcl::PointXYZIT> s_compress(
//     {pcl::io::compression_Profiles_e::MED_RES_ONLINE_COMPRESSION_WITH_COLOR}, true);
static pcl::io::OctreePointCloudCompression<pcl::PointXYZIT> s_compress;

::zhito::inner_class::ZPointCloudXYZIT StructToClass(zhito::inner_struct::sp_cZPointCloudXYZIT const& in) {
  ::zhito::inner_class::ZPointCloudXYZIT result;
  *result.mutable_header() = StructToClass(*(std::shared_ptr<zhito::inner_struct::ZFrame const>)in);
  // 压缩点云
  std::stringstream compressed_pcl_pointcloud;
  s_compress.encodePointCloud(in, compressed_pcl_pointcloud);
  result.set_pcl_compressed_pc(compressed_pcl_pointcloud.str());
  // std::cout <<" compress size = " << compressed_pcl_pointcloud.str().size() <<"\n";
  return result;
}

// ClassToStruct declaration
zhito::inner_struct::spZPointCloudXYZIT ClassToStruct(::zhito::inner_class::ZPointCloudXYZIT const& in) {
  ::zhito::inner_struct::spZPointCloudXYZIT result = std::make_shared<zhito::inner_struct::ZPointCloudXYZIT>();
  *(std::shared_ptr<::zhito::inner_struct::ZFrame>)result = ClassToStruct(in.header());
  // 解压点云
  std::stringstream compressed_pcl_pointcloud(in.pcl_compressed_pc());
  // std::cout <<" compressed_pcl_pointcloud size = " << compressed_pcl_pointcloud.str().size() << "\n";
  // decodePointCloud 的参数定义成了智能指针的引用, 所以不能存在转换. 只有是 const T& 和 std::shared_ptr<T>, 可以使用多态,即直接使用子类对象或者子类智能指针
  s_compress.decodePointCloud(compressed_pcl_pointcloud, (pcl::PointCloud<pcl::PointXYZIT>::Ptr&)result);
  // std::cout <<" compressed_pcl_pointcloud size = " << compressed_pcl_pointcloud.str().size() << " done \n";

  return result;
}

}  // namespace converter

} // namespace zhito
