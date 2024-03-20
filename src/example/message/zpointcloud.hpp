#pragma once

#include <pcl/point_cloud.h>

#include "zframe.struct_pb.h"
#include "pointcloud.pb.h"
#include "xyzit_points.hpp"

// save to file
#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>

namespace zhito{


namespace inner_struct {

template <typename _PointType>
struct ZPointCloud : public ZFrame, public pcl::PointCloud<_PointType> {

};

using ZPointCloudXYZIT = ::zhito::inner_struct::ZPointCloud<pcl::PointXYZIT>;
using spZPointCloudXYZIT = std::shared_ptr<::zhito::inner_struct::ZPointCloudXYZIT>;
using sp_cZPointCloudXYZIT = std::shared_ptr<::zhito::inner_struct::ZPointCloudXYZIT const>;


template <typename _PointType>
inline std::ostream& operator<<(std::ostream& os,::zhito::inner_struct::ZPointCloud<_PointType> const& in)
{
    os << (ZFrame&)in << std::endl;
    os << (pcl::PointCloud<_PointType>&)in << std::endl;
    return os;
}

}  // namespace inner_struct


namespace converter {
// StructToClass declaration

::zhito::inner_class::ZPointCloudXYZIT StructToClass(inner_struct::sp_cZPointCloudXYZIT const& in);
// enum

// ClassToStruct declaration
::zhito::inner_struct::spZPointCloudXYZIT ClassToStruct(::zhito::inner_class::ZPointCloudXYZIT const& in);

} // namespace converter

} // namespace zhito