#pragma once
#include "EnumGenerator.h"
#include "GeneratorBase.h"
#include "MessageGenerator.h"
#include "Options.hpp"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/io/printer.h"
namespace struct_pb {
namespace compiler {
class FileGenerator : public GeneratorBase {
 public:
  FileGenerator(const google::protobuf::FileDescriptor *file, Options options);
  ;
  void generate_header(google::protobuf::io2::Printer *p);
  void generate_source(google::protobuf::io2::Printer *p);

 private:
  void generate_shared_header_code(google::protobuf::io2::Printer *p);
  void generate_fwd_decls(google::protobuf::io2::Printer *p);
  void generate_enum_definitions(google::protobuf::io2::Printer *p);
  void generate_message_definitions(google::protobuf::io2::Printer *p);
  void generate_message_struct_pb_func_definitions(
      google::protobuf::io2::Printer *p);
  void generate_message_struct_pb_func_source(google::protobuf::io2::Printer *p);
  void generate_dependency_includes(google::protobuf::io2::Printer *p);
  void generate_ns_open(google::protobuf::io2::Printer *p);
  void generate_ns_close(google::protobuf::io2::Printer *p);

  // added
  void generate_message_reflection_definitions(google::protobuf::io2::Printer *p);

  void generate_message_tostring_func_source(google::protobuf::io2::Printer *p);

  void generate_enum_mapper_decls(google::protobuf::io2::Printer *p);

    // added
  void generate_message_struct2class_definitions(google::protobuf::io2::Printer *p);
  void generate_message_class2struct_definitions(google::protobuf::io2::Printer *p);
  void generate_message_to_string_definitions(google::protobuf::io2::Printer *p);
  void generate_message_struct2class_source(google::protobuf::io2::Printer *p);
  void generate_message_class2struct_source(google::protobuf::io2::Printer *p);
  void generate_message_tostring_source(google::protobuf::io2::Printer *p);
  void generate_eigen_helper(google::protobuf::io2::Printer *p);
  void generate_pcl_helper(google::protobuf::io2::Printer *p);
  void generate_enum_helper(google::protobuf::io2::Printer *p);

 private:
  const google::protobuf::FileDescriptor *file_;
  std::vector<std::unique_ptr<MessageGenerator>> message_generators_;
  std::vector<std::unique_ptr<EnumGenerator>> enum_generators_;
};
}  // namespace compiler
}  // namespace struct_pb