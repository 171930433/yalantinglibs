#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>

// #include "addressbook.struct_pb.h"
// #include "ylt/struct_json/json_reader.h"
// #include "ylt/struct_json/json_writer.h"

// protobuf
#include <google/protobuf/util/json_util.h>

// #include "addressbook.pb.h"
// namespace tutorial = MyInner;

namespace iguana {

// template <>
// struct enum_value<MyInner::PhoneNumber::PhoneType> {
//   constexpr static std::array<int, 3> value = {0, 1, 2};
// };

}  // namespace iguana


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




    std::cout << "Done!!!" << std::endl;

  return 0;
}
