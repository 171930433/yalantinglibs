#pragma once

#include <pcl/point_cloud.h>

#include "base.struct_pb.h"
#include "image.pb.h"

#include <opencv2/core.hpp>

namespace zhito{

namespace inner_struct {

struct ZImage : public ZFrame, public cv::Mat {

};

using spZImage = std::shared_ptr<::zhito::inner_struct::ZImage>;
using sp_cZImage = std::shared_ptr<::zhito::inner_struct::ZImage const>;


std::ostream& operator<<(std::ostream& os,::zhito::inner_struct::ZImage const& in);

}  // namespace inner_struct


namespace converter {
// StructToClass declaration

::zhito::inner_class::ZImage StructToClass(inner_struct::ZImage const& in);
// enum

// ClassToStruct declaration
::zhito::inner_struct::ZImage ClassToStruct(::zhito::inner_class::ZImage const& in);

} // namespace converter

}// namespace zhito