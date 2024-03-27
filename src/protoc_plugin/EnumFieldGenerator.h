#pragma once
#include "FieldGenerator.h"

namespace struct_pb {
namespace compiler {

class EnumFieldGenerator : public FieldGenerator {
 public:
  EnumFieldGenerator(const FieldDescriptor *descriptor, const Options &options);
  std::string cpp_type_name() const override;
  void generate_calculate_size(google::protobuf::io2::Printer *p,
                               const std::string &value,
                               bool can_ignore_default_value) const override;
  void generate_calculate_size_only(google::protobuf::io2::Printer *p,
                                    const std::string &value) const;
  void generate_serialization(google::protobuf::io2::Printer *p,
                              const std::string &value,
                              bool can_ignore_default_value) const override;
  void generate_serialization_only(google::protobuf::io2::Printer *p,
                                   const std::string &value) const;
  void generate_deserialization(google::protobuf::io2::Printer *p,
                                const std::string &value) const override;
  void generate_deserialization_only(
      google::protobuf::io2::Printer *p, const std::string &output,
      const std::string &max_size = "size") const;

  // added
  void generate_struct_to_class(google::protobuf::io2::Printer *p) const override;
  void generate_class_to_struct(google::protobuf::io2::Printer *p) const override;

 private:
  std::string default_value() const;
};
class RepeatedEnumFieldGenerator : public FieldGenerator {
 public:
  RepeatedEnumFieldGenerator(const FieldDescriptor *descriptor,
                             const Options &options);

  std::string cpp_type_name() const override;
  void generate_calculate_size(google::protobuf::io2::Printer *p,
                               const std::string &value,
                               bool can_ignore_default_value) const override;
  void generate_calculate_size_only(google::protobuf::io2::Printer *p,
                                    const std::string &value,
                                    const std::string &output) const;
  void generate_serialization(google::protobuf::io2::Printer *p,
                              const std::string &value,
                              bool can_ignore_default_value) const override;
  void generate_serialization_only(google::protobuf::io2::Printer *p,
                                   const std::string &value) const;
  void generate_deserialization(google::protobuf::io2::Printer *p,
                                const std::string &value) const override;

 private:
  bool is_packed() const;
  std::string packed_tag() const;
  std::string unpacked_tag() const;
};
}  // namespace compiler
}  // namespace struct_pb
