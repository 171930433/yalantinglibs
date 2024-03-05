#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>

// #include "addressbook.struct_pb.h"
#include "imu.struct_pb.h"
// protobuf
#include <google/protobuf/util/json_util.h>


namespace Eigen
{
template <bool Is_writing_escape, typename Stream>
inline void to_json_impl(Stream& s, const Eigen::Vector3d& t) {
  iguana::to_json<Is_writing_escape>(*(double(*)[3]) & t, s);
}

template <typename It>
IGUANA_INLINE void from_json_impl(Eigen::Vector3d& value, It&& it, It&& end) {
  iguana::from_json(*(double(*)[3]) & value, it, end);
}
}

int main() {
  //   // MyInner::AddressBook address_book1;
  //   // tutorial::AddressBook address_book2;

  //   // address_book1.people.push_back(
  //   //     MyInner::Person{"xiaoming",
  //   //                     1,
  //   //                     "123@qq.com",
  //   //                     {{"123-456",
  //   //                     ::MyInner::PhoneNumber::PhoneType::WORK}}});

  //   // // tojson
  //   // std::string str1;
  //   // struct_json::to_json(address_book1, str1);
  //   // std::cout << " address_book1 is" << str1 << "\n";

  //   // // address_book1 --> address_book2
  //   // google::protobuf::util::JsonStringToMessage(str1, &address_book2);
  //   // std::cout << "address_book2 is" << address_book2.DebugString() <<
  //   "\n";

  //   // // address_book2 --> address_book3
  //   // MyInner::AddressBook address_book3;
  //   // std::string str2;
  //   // google::protobuf::util::JsonOptions options;
  //   // options.always_print_enums_as_ints = true;
  //   // google::protobuf::util::MessageToJsonString(address_book2, &str2,
  //   options);
  //   // std::cout << "address_book2  json is" << str2 << "\n";

  //   // struct_json::from_json(address_book3, str2);
  //   // std::string str3;
  //   // struct_json::to_json(address_book3, str3);
  //   // std::cout << "address_book3 is " << str3 << "\n";


  inner_struct::ZImu imu1{1,2,3,inner_struct::ZFrameType::IMU,"channel_name", {4,5,6,}, {7,8,9}, 10};
  inner_class::ZImu imu2 = inner_struct::InnerStructToInnerClass(imu1);

  std::cout << imu2.DebugString() << std::endl;

  std::string str;
  iguana::to_json(imu1, str);

  std::cout << str << std::endl;


  inner_struct::ZImu imu3;
  iguana::from_json(imu3, str);

  std::string str3;
  iguana::to_json(imu3, str3);
  std::cout << str << std::endl;




  std::cout << "Done!!!" << std::endl;

  return 0;
}
