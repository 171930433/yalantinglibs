#include <fstream>
#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>

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

namespace MyInner {

// REFLECTION(AddressBook, people);
// REFLECTION(Person, name, id, email, phones);
// REFLECTION(PhoneNumber, number, type);

}  // namespace MyInner
// namespace tutorial

// void prompt_for_address(tutorial::Person& person) {
//   std::cout << "==================================" << std::endl;
//   std::cout << "            Add People            " << std::endl;
//   std::cout << "==================================" << std::endl;
//   std::cout << "Enter person ID number: ";
//   std::cin >> person.id;
//   std::cin.ignore(256, '\n');
//   std::cout << "Enter name: ";
//   std::getline(std::cin, person.name);
//   std::cout << "Enter email address (blank for none): ";
//   std::getline(std::cin, person.email);
//   while (true) {
//     std::cout << "Enter a phone number (or leave blank to finish): ";
//     tutorial::PhoneNumber phone_number;
//     std::getline(std::cin, phone_number.number);
//     if (phone_number.number.empty()) {
//       break;
//     }
//     std::cout << "Is this a mobile, home, or work phone? ";
//     std::string type;
//     std::getline(std::cin, type);
//     if (type == "mobile") {
//       phone_number.type = tutorial::PhoneNumber::PhoneType::MOBILE;
//     }
//     else if (type == "home") {
//       phone_number.type = tutorial::PhoneNumber::PhoneType::HOME;
//     }
//     else if (type == "work") {
//       phone_number.type = tutorial::PhoneNumber::PhoneType::WORK;
//     }
//     else {
//       std::cout << "Unknown phone type: Using default." << std::endl;
//     }
//     person.phones.push_back(phone_number);
//   }
// }
// void list_people(const tutorial::AddressBook& address_book) {
//   std::cout << "==================================" << std::endl;
//   std::cout << "          List People             " << std::endl;
//   std::cout << "==================================" << std::endl;
//   for (const auto& person : address_book.people) {
//     std::cout << "     Person ID: " << person.id << std::endl;
//     std::cout << "          Name: " << person.name << std::endl;
//     if (!person.email.empty()) {
//       std::cout << "E-mail address: " << person.email << std::endl;
//     }
//     for (const auto& phone : person.phones) {
//       switch (phone.type) {
//         case tutorial::PhoneNumber::PhoneType::MOBILE:
//           std::cout << "Mobile phone #: ";
//           break;
//         case tutorial::PhoneNumber::PhoneType::HOME:
//           std::cout << "  Home phone #: ";
//           break;
//         case tutorial::PhoneNumber::PhoneType::WORK:
//           std::cout << "  Work phone #: ";
//           break;
//       }
//       std::cout << phone.number << std::endl;
//     }
//   }
// }
// int main(int argc, char* argv[]) {
//   if (argc != 2) {
//     std::cerr << "Usage: " << argv[0] << " ADDRESS_BOOK_FILE" << std::endl;
//     return -1;
//   }
//   tutorial::AddressBook address_book;
//   std::fstream input(argv[1], std::ios::in | std::ios::binary);
//   if (!input) {
//     std::cout << argv[1] << ": File not found. Creating a new file."
//               << std::endl;
//   }
//   else {
//     input.seekg(0, input.end);
//     int length = input.tellg();
//     input.seekg(0, input.beg);
//     std::string buffer;
//     buffer.resize(length);
//     input.read(buffer.data(), buffer.size());
//     input.close();
//     bool ok = struct_pb::deserialize_to(address_book, buffer);
//     if (!ok) {
//       std::cerr << "Failed to parse address book." << std::endl;
//       return -1;
//     }
//   }
//   list_people(address_book);

//   std::string str;
//   struct_json::to_json(address_book, str);
//   std::cout << str << "\n";

//   // address_book.people.emplace_back();
//   // prompt_for_address(address_book.people.back());
//   // std::fstream output(argv[1],
//   //                     std::ios::out | std::ios::trunc | std::ios::binary);
//   // std::string buffer = struct_pb::serialize<std::string>(address_book);
//   // output.write(buffer.data(), buffer.size());
//   // output.close();
//   // list_people(address_book);
//   std::cout << "Done!!!" << std::endl;
//   return 0;
// }

int main() {
  // MyInner::AddressBook address_book1;
  // tutorial::AddressBook address_book2;

  // address_book1.people.push_back(
  //     MyInner::Person{"xiaoming",
  //                     1,
  //                     "123@qq.com",
  //                     {{"123-456",
  //                     ::MyInner::PhoneNumber::PhoneType::WORK}}});

  // // tojson
  // std::string str1;
  // struct_json::to_json(address_book1, str1);
  // std::cout << " address_book1 is" << str1 << "\n";

  // // address_book1 --> address_book2
  // google::protobuf::util::JsonStringToMessage(str1, &address_book2);
  // std::cout << "address_book2 is" << address_book2.DebugString() << "\n";

  // // address_book2 --> address_book3
  // MyInner::AddressBook address_book3;
  // std::string str2;
  // google::protobuf::util::JsonOptions options;
  // options.always_print_enums_as_ints = true;
  // google::protobuf::util::MessageToJsonString(address_book2, &str2, options);
  // std::cout << "address_book2  json is" << str2 << "\n";

  // struct_json::from_json(address_book3, str2);
  // std::string str3;
  // struct_json::to_json(address_book3, str3);
  // std::cout << "address_book3 is " << str3 << "\n";

  // MyInner::AddressBook ab1;
  // ab1.people.push_back(
  //     {"xiaoming",
  //      1,
  //      "123@qq.com",
  //      {{"123-456", ::MyInner::PhoneNumber::PhoneType::WORK}}});
  // ab1.people[0].pt_enu = std::make_optional<Eigen::Vector3d>(1,2,3);

  // std::cout << struct_pb::internal::to_string(ab1) << std::endl;

 
  std::cout << "Done!!!" << std::endl;

  return 0;
}