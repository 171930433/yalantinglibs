#include <fstream>
#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>

#include "addressbook.struct_pb.h"
// #include "ylt/struct_json/json_reader.h"
// #include "ylt/struct_json/json_writer.h"

// protobuf
#include <google/protobuf/util/json_util.h>

#include "addressbook.pb.h"
// namespace tutorial = MyInner;


namespace iguana {

// template <>
// struct enum_value<MyInner::PhoneNumber::PhoneType> {
//   constexpr static std::array<int, 3> value = {0, 1, 2};
// };

}  // namespace iguana

namespace MyInner {


}

// int main() {
// //   // MyInner::AddressBook address_book1;
// //   // tutorial::AddressBook address_book2;

// //   // address_book1.people.push_back(
// //   //     MyInner::Person{"xiaoming",
// //   //                     1,
// //   //                     "123@qq.com",
// //   //                     {{"123-456",
// //   //                     ::MyInner::PhoneNumber::PhoneType::WORK}}});

// //   // // tojson
// //   // std::string str1;
// //   // struct_json::to_json(address_book1, str1);
// //   // std::cout << " address_book1 is" << str1 << "\n";

// //   // // address_book1 --> address_book2
// //   // google::protobuf::util::JsonStringToMessage(str1, &address_book2);
// //   // std::cout << "address_book2 is" << address_book2.DebugString() << "\n";

// //   // // address_book2 --> address_book3
// //   // MyInner::AddressBook address_book3;
// //   // std::string str2;
// //   // google::protobuf::util::JsonOptions options;
// //   // options.always_print_enums_as_ints = true;
// //   // google::protobuf::util::MessageToJsonString(address_book2, &str2, options);
// //   // std::cout << "address_book2  json is" << str2 << "\n";

// //   // struct_json::from_json(address_book3, str2);
// //   // std::string str3;
// //   // struct_json::to_json(address_book3, str3);
// //   // std::cout << "address_book3 is " << str3 << "\n";

// innerstruct::AddressBook ab1;
// ab1.people.push_back(
//     {"xiaoming",
//      1,
//      "123@qq.com",
//      {{"123-456", ::innerstruct::PhoneNumber::PhoneType::WORK}}});
// //   ab1.people[0].pt_enu = Eigen::Vector3d(1,2,3);

// nlohmann::json j;

// to_json(j, ab1);

// std::cout << j.dump(2) << std::endl;

// //   std::cout << "Done!!!" << std::endl;

// return 0;
// }


#include <iostream>
#include <eigen3/Eigen/Dense>


namespace iguana
{
namespace detail{

// template <typename It>
  // std::stringstream ss(it,end);
  // std::cout <<" !!!!!!!!--- " << ss.str() <<"\n";


}
}

namespace Eigen
{
  namespace detail
  {
 inline void parse_item(Eigen::Vector3d &value, std::string::iterator it, std::string::iterator end) {

  }
} 
}


#include "iguana/json_reader.hpp"
#include "iguana/json_writer.hpp"

// 定义一个结构体表示一个人的信息
struct Person {
    std::string name;
    int age;
    Eigen::Vector3d position;
    // int arr[3] = {4,5,6};
};



// 声明Person结构体到JSON的序列化方式
REFLECTION(Person, name, age, position);
// REFLECTION(Person, name, age);
namespace Eigen
{
template <typename Stream>
IGUANA_INLINE void render_json_value(Stream &ss, Eigen::Vector3d const& val) {
  std::stringstream ss2;
  ss2 << "[" << val[0] <<"," << val[1] <<"," <<val[2] <<"]";
  ss.append(ss2.str());
}



}





// /home/gsk/pro/yalantinglibs/src/struct_pb/protoc-plugin/example/iguana/json_reader.hpp:554:37: error:
// no matching function for call to 
// ‘
// parse_item(Eigen::Matrix<double, 3, 1>&,
//  __gnu_cxx::__normal_iterator<const char*, std::__cxx11::basic_string<char> >&,
// __gnu_cxx::__normal_iterator<const char*, std::__cxx11::basic_string<char> >&)’

int main() {
    // 解析JSON字符串到Person结构体
    std::string json_data = R"( {"name":"jhon","age":30,"position":[1,2,3]} )";

    Person person;
    iguana::from_json(person, json_data);

    // 输出解析后的Person结构体
    std::cout << "Name: " << person.name << std::endl;
    std::cout << "Age: " << person.age << std::endl;
    std::cout << "Position: " << person.position.transpose() << std::endl;

    // 将Person结构体序列化为JSON字符串
    Person person2{"jhon",30,{1,2,3}};

    std::string str;
    iguana::to_json(person2, str);
    std::cout << "Serialized JSON: " << str << std::endl;

    return 0;
}

