#pragma once

#include <pcl/point_cloud.h>

#include "base.struct_pb.h"
#include "image.pb.h"

#include <opencv2/core.hpp>

namespace inner_struct {

struct ZImage : public ZFrame, public cv::Mat {

};

using spZImage = std::shared_ptr<::inner_struct::ZImage>;
using sp_cZImage = std::shared_ptr<::inner_struct::ZImage const>;


std::ostream& operator<<(std::ostream& os,::inner_struct::ZImage const& in);

}  // namespace inner_struct


namespace converter {
// StructToClass declaration

::inner_class::ZImage StructToClass(inner_struct::ZImage const& in);
// enum

// ClassToStruct declaration
::inner_struct::ZImage ClassToStruct(::inner_class::ZImage const& in);

} // namespace converter