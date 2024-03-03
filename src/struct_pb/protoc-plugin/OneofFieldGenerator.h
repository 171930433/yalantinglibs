#pragma once
#include "FieldGenerator.h"
namespace struct_pb {
namespace compiler {

class OneofFieldGenerator : public FieldGenerator {
 public:
  OneofFieldGenerator(const FieldDescriptor *descriptor,
                      const Options &options);
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
      google::protobuf::io2::Printer *p, const std::string &value,
      const std::string &max_size = "size") const;
  std::string cpp_type_name() const override;
  std::string pb_type_name() const override;
  virtual void generate_setter_and_getter(google::protobuf::io2::Printer *p){};

 protected:
  std::string index() const;
};

class MessageOneofFieldGenerator : public OneofFieldGenerator {
 public:
  MessageOneofFieldGenerator(const FieldDescriptor *descriptor,
                             const Options &options);
  void generate_setter_and_getter(google::protobuf::io2::Printer *p) override;
};

class StringOneofFieldGenerator : public OneofFieldGenerator {
 public:
  StringOneofFieldGenerator(const FieldDescriptor *descriptor,
                            const Options &options);
  void generate_setter_and_getter(google::protobuf::io2::Printer *p) override;
};

class EnumOneofFieldGenerator : public OneofFieldGenerator {
 public:
  EnumOneofFieldGenerator(const FieldDescriptor *descriptor,
                          const Options &options);
  void generate_setter_and_getter(google::protobuf::io2::Printer *p) override;
};

class PrimitiveOneofFieldGenerator : public OneofFieldGenerator {
 public:
  PrimitiveOneofFieldGenerator(const FieldDescriptor *descriptor,
                               const Options &options);
  void generate_setter_and_getter(google::protobuf::io2::Printer *p) override;
};

}  // namespace compiler
}  // namespace struct_pb
