#pragma once

#include <pcl/point_cloud.h>

#include "base.struct_pb.h"
#include "pointcloud.pb.h"
#include "xyzit_points.hpp"

// save to file
#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>

namespace inner_struct {

// template <typename _PointType>
// struct ZPointCloud : public ZFrame, public pcl::PointCloud<_PointType> {

// };

template <typename _PointType>
struct ZPointCloud :  public pcl::PointCloud<_PointType>, public ZFrame {

};

using ZPointCloudXYZIT = ::inner_struct::ZPointCloud<pcl::PointXYZIT>;
using spZPointCloudXYZIT = std::shared_ptr<::inner_struct::ZPointCloudXYZIT>;
using sp_cZPointCloudXYZIT = std::shared_ptr<::inner_struct::ZPointCloudXYZIT const>;


template <typename _PointType>
inline std::ostream& operator<<(std::ostream& os,::inner_struct::ZPointCloud<_PointType> const& in)
{
    os << (ZFrame&)in << std::endl;
    os << (pcl::PointCloud<_PointType>&)in << std::endl;
    return os;
}

}  // namespace inner_struct


namespace converter {
// StructToClass declaration

::inner_class::ZPointCloudXYZIT StructToClass(inner_struct::sp_cZPointCloudXYZIT const& in);
// enum

// ClassToStruct declaration
::inner_struct::spZPointCloudXYZIT ClassToStruct(::inner_class::ZPointCloudXYZIT const& in);

} // namespace converter