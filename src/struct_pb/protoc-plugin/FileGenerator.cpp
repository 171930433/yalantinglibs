#include "FileGenerator.h"

#include <google/protobuf/compiler/code_generator.h>

#include "EnumGenerator.h"
#include "MessageGenerator.h"
#include "helpers.hpp"

#include <iostream>

using namespace google::protobuf;
using namespace google::protobuf::compiler;
namespace struct_pb {
namespace compiler {

FileGenerator::FileGenerator(const google::protobuf::FileDescriptor *file,
                             Options options)
    : GeneratorBase(options), file_(file) {
  //  std::vector<const Descriptor*> msgs = flatten_messages_in_file(file);
  for (int i = 0; i < file_->message_type_count(); ++i) {
    message_generators_.push_back(
        std::make_unique<MessageGenerator>(file_->message_type(i), options));
  }
  for (int i = 0; i < file->enum_type_count(); ++i) {
    enum_generators_.push_back(
        std::make_unique<EnumGenerator>(file->enum_type(i), options));
  }
}

void FileGenerator::generate_enum_mapper_decls(
    google::protobuf::io2::Printer *p) {
  for (int i = 0; i < enum_generators_.size(); ++i) {
    enum_generators_[i]->generateMapDeclaration(p);
  }
}

void FileGenerator::generate_enum_definitions(
    google::protobuf::io2::Printer *p) {
  for (int i = 0; i < enum_generators_.size(); ++i) {
    enum_generators_[i]->generate_definition(p);
  }
}
void FileGenerator::generate_message_definitions(
    google::protobuf::io2::Printer *p) {
  for (int i = 0; i < message_generators_.size(); ++i) {
    message_generators_[i]->generate_struct_definition(p);
  }
}
void FileGenerator::generate_shared_header_code(
    google::protobuf::io2::Printer *p) {
  generate_fwd_decls(p);            // 代码前置声明
  generate_enum_definitions(p);     // 枚举定义
  generate_message_definitions(p);  // 消息定义
}
void FileGenerator::generate_header(google::protobuf::io2::Printer *p) {
  p->Print(
      {{"filename", file_->name()}},
      R"(// Generated by the protocol buffer compiler (struct_pb).  DO NOT EDIT!
// source: $filename$

#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "nlohmann/json.hpp"
#include "iguana/json_reader.hpp"
#include "iguana/json_writer.hpp"
)");

  generate_dependency_includes(p);  // 包含的其他头文件
  generate_ns_open(p);              // 写命名空间
  generate_shared_header_code(p);
  generate_message_tostring_func_definitions(p);
  generate_ns_close(p);  // 关闭命名空间
  // generate_message_struct_pb_func_definitions(p);
  p->Print("// clang-format on\n");
}

void FileGenerator::generate_fwd_decls(google::protobuf::io2::Printer *p) {
  Formatter format(p);
  for (int i = 0; i < file_->message_type_count(); ++i) {
    auto m = file_->message_type(i);
    auto eigen_name = m->options().GetExtension(eigen_typename);
    if (eigen_name.empty()) {
      format("struct $1$;\n", resolve_keyword(m->name()));
    }
    else {
      format("using $1$ = $2$;\n", resolve_keyword(m->name()), eigen_name);
    }
    std::cerr << " i= " << i << ", name = " << m->name() << "\n";
  }
}
void FileGenerator::generate_dependency_includes(
    google::protobuf::io2::Printer *p) {
  Formatter format(p);
  for (int i = 0; i < file_->dependency_count(); ++i) {
    auto dep = file_->dependency(i);
    std::string basename = strip_proto(dep->name());
    if(basename == "proto_to_struct") {continue;}
    std::string header_name = basename + ".struct_pb.h";
    format("#include \"$1$\"\n", header_name);
    //
    std::cerr << "\n header_name = " << header_name << "\n";
  }
}
void FileGenerator::generate_source(google::protobuf::io2::Printer *p) {
  // generate_message_struct_pb_func_source(p);
  generate_ns_open(p);              // 写命名空间

  generate_message_tostring_func_source(p);
  generate_ns_close(p);  // 关闭命名空间

}
void FileGenerator::generate_message_struct_pb_func_definitions(
    google::protobuf::io2::Printer *p) {
  std::vector<const Descriptor *> msgs = flatten_messages_in_file(file_);
  Formatter format(p);
  format("namespace struct_pb {\n");
  format("namespace internal {\n");
  for (auto msg : msgs) {
    auto name = qualified_class_name(msg, options_);
    format("// $1$\n", name);
    format(
        "template<>\n"
        "std::size_t get_needed_size<$1$>(const $1$& t, const "
        "::struct_pb::UnknownFields& unknown_fields);\n",
        name);
    format(
        "template<>\n"
        "void serialize_to<$1$>(char* data, std::size_t size, const $1$& t, "
        "const "
        "::struct_pb::UnknownFields& unknown_fields);\n",
        name);
    format(
        "template<>\n"
        "bool deserialize_to<$1$>($1$& t, const char* data, std::size_t size, "
        "::struct_pb::UnknownFields& unknown_fields);\n",
        name);
    format(
        "template<>\n"
        "bool deserialize_to<$1$>($1$& t, const char* data, std::size_t "
        "size);\n",
        name);
    format("\n");
  }
  format("} // internal\n");
  format("} // struct_pb\n");
}
void FileGenerator::generate_message_struct_pb_func_source(
    google::protobuf::io2::Printer *p) {
  std::vector<const Descriptor *> msgs = flatten_messages_in_file(file_);
  Formatter format(p);
  format("namespace struct_pb {\n");
  format("namespace internal {\n");
  for (auto msg : msgs) {
    auto name = qualified_class_name(msg, options_);
    MessageGenerator g(msg, options_);
    format("// $1$\n", name);
    format(
        "template<>\n"
        "std::size_t get_needed_size<$1$>(const $1$& t, const "
        "::struct_pb::UnknownFields& unknown_fields) {\n",
        name);
    format.indent();
    g.generate_get_needed_size(p);
    format.outdent();
    format(
        "} // std::size_t get_needed_size<$1$>(const $1$& t, const "
        "::struct_pb::UnknownFields& unknown_fields)\n",
        name);

    format(
        "template<>\n"
        "void serialize_to<$1$>(char* data, std::size_t size, const $1$& t, "
        "const ::struct_pb::UnknownFields& unknown_fields) {\n",
        name);
    format.indent();
    g.generate_serialize_to(p);
    format.outdent();
    format(
        "} // void serialize_to<$1$>(char* data, std::size_t size, const $1$& "
        "t, "
        "const ::struct_pb::UnknownFields& unknown_fields)\n",
        name);

    format(
        "template<>\n"
        "bool deserialize_to<$1$>($1$& t, const char* data, std::size_t size, "
        "::struct_pb::UnknownFields& unknown_fields) {\n",
        name);
    format.indent();
    g.generate_deserialize_to(p);
    format.outdent();
    format("return true;\n");
    format("} // bool deserialize_to<$1$>($1$&, const char*, std::size_t)\n",
           name);
    format("// end of $1$\n", name);
    format(
        "template<>\n"
        "bool deserialize_to<$1$>($1$& t, const char* data, std::size_t size) "
        "{\n",
        name);
    format.indent();
    format("::struct_pb::UnknownFields unknown_fields{};\n");
    format("return deserialize_to(t,data,size,unknown_fields);\n");
    format.outdent();
    format("}\n");
    format("\n");
  }
  format("} // internal\n");
  format("} // struct_pb\n");
}
void FileGenerator::generate_ns_open(google::protobuf::io2::Printer *p) {
  NamespaceOpener(p, options_.ns).open();
}
void FileGenerator::generate_ns_close(google::protobuf::io2::Printer *p) {
  NamespaceOpener(p, options_.ns).close();
}

void FileGenerator::generate_message_tostring_func_definitions(
    google::protobuf::io2::Printer *p) {
  std::vector<const Descriptor *> msgs = flatten_messages_in_file(file_);
  Formatter format(p);

  
  for (auto msg : msgs) {
    if(is_eigen_type(msg)) {continue;}

    auto raw_name = msg->name();
    auto name = qualified_class_name(msg, options_);
    format("// $1$\n", name);
    // format("void to_json(nlohmann::json& j, $1$ const& t);\n", name);
    format("REFLECTION($1$", raw_name);
    for(int i = 0; i < msg->field_count();++i)
    {
      format(", $1$", msg->field(i)->name());
    }
    format(");\n");
  }

  // enum
  std::vector<const EnumDescriptor *> enums = flatten_enums_in_file(file_);
  for (auto single_enum : enums) {
    auto name = qualified_enum_name(single_enum, options_);
    format("// $1$\n", name);
    format("void to_json(nlohmann::json& j, $1$ const& t);\n", name);
    format("\n");
  }

}

void FileGenerator::generate_message_tostring_func_source(
    google::protobuf::io2::Printer *p) {
  std::vector<const Descriptor *> msgs = flatten_messages_in_file(file_);
  Formatter format(p);
  for (auto msg : msgs) {
    auto name = qualified_class_name(msg, options_);
    MessageGenerator g(msg, options_);
    format("// $1$\n", name);
    format("void to_json(nlohmann::json& j, $1$ const& t) { // $1$ \n", name);
    g.generate_to_string_to(p);
    format("}\n");
  }

  // enum
  std::vector<const EnumDescriptor *> enums = flatten_enums_in_file(file_);
  for (auto single_enum : enums) {
    auto name = qualified_enum_name(single_enum, options_);
    EnumGenerator g(single_enum, options_);
    format("// $1$\n", name);
    format("void to_json(nlohmann::json& j, $1$ const& t) { // $1$ \n", name);
    g.generateMapDefinition(p);
    format("}\n");
    
  }

}

}  // namespace compiler

}  // namespace struct_pb