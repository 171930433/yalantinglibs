#include "zimage.hpp"

#include <opencv2/imgcodecs.hpp>
#include <string_view>

namespace zhito{


namespace inner_struct {
std::ostream& operator<<(std::ostream& os, ::zhito::inner_struct::ZImage const& in) {
  os << (ZFrame const&)in << std::endl;
  os << (cv::Mat const&)in << std::endl;
  return os;
}

}  // namespace inner_struct

namespace converter {

::zhito::inner_class::ZImage StructToClass(inner_struct::ZImage const& in) {
  ::zhito::inner_class::ZImage result;
  *result.mutable_header() = StructToClass((inner_struct::ZFrame const&)in);
  //
  std::vector<int> const compressing_factor{cv::ImwriteFlags::IMWRITE_JPEG_QUALITY, 95};
  std::vector<uchar> buf;
  std::string asd;
  cv::imencode(".jpg", in, buf, compressing_factor);
  *result.mutable_cv_compressed_image() = std::string_view((char*)buf.data(), buf.size());  //! 会发生两次拷贝,需要考虑

  return result;
}
// enum

// ClassToStruct declaration
::zhito::inner_struct::ZImage ClassToStruct(::zhito::inner_class::ZImage const& in) {
  ::zhito::inner_struct::ZImage result;
  (inner_struct::ZFrame&)result = ClassToStruct(in.header());
  std::vector<uchar> buf((uchar const*)(in.cv_compressed_image().data()),
                         (uchar const*)(in.cv_compressed_image().data() + in.cv_compressed_image().size()));
  (cv::Mat&)result = cv::imdecode(buf, 1);  //! 会发生两次拷贝,需要考虑

  return result;
}

}  // namespace converter

}// namespace zhito
