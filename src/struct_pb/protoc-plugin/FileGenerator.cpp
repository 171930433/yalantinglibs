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


void FileGenerator::generate_eigen_helper(google::protobuf::io2::Printer *p)
{
  if (!file_->options().GetExtension(write_eigen_helper)) {
    return;
  }

  p->PrintRaw(
R"(namespace Eigen
{

template <bool Is_writing_escape, typename Stream, typename _Scalar, int _Row,int _Col>
inline void to_json_impl(Stream& s, const Eigen::Matrix<_Scalar,_Row,_Col>& t) {
  iguana::to_json<Is_writing_escape>(*(_Scalar(*)[_Row * _Col]) & t, s);
}

template <typename It, typename _Scalar, int _Row,int _Col>
IGUANA_INLINE void from_json_impl(Eigen::Matrix<_Scalar,_Row,_Col>& value, It&& it, It&& end) {
  iguana::from_json(*(_Scalar(*)[_Row * _Col]) & value, it, end);
}

template <bool Is_writing_escape, typename Stream, typename _Scalar>
inline void to_json_impl(Stream& s, const Eigen::Quaternion<_Scalar>& t) {
  iguana::to_json<Is_writing_escape>(*(_Scalar(*)[4]) & t, s);
}

template <typename It, typename _Scalar>
IGUANA_INLINE void from_json_impl(Eigen::Quaternion<_Scalar>& value, It&& it, It&& end) {
  iguana::from_json(*(_Scalar(*)[4]) & value, it, end);
}

})");

}

void FileGenerator::generate_header(google::protobuf::io2::Printer *p) {
  auto basename = strip_proto(file_->name());
  p->Print(
      {{"filename", file_->name()},
       {"pb_header_file", basename + ".pb.h"}
      },
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
#include <iostream>

#include "iguana/json_reader.hpp"
#include "iguana/json_writer.hpp"
#include "$pb_header_file$"
)");

  generate_dependency_includes(p);  // 包含的其他头文件
  generate_ns_open(p);              // 写命名空间
  generate_shared_header_code(p);
  generate_message_tostring_func_definitions(p);
  generate_message_struct2class_definitions(p);
  generate_ns_close(p);  // 关闭命名空间
  // generate_message_struct_pb_func_definitions(p);
  generate_eigen_helper(p);
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

  // generate_message_tostring_func_source(p);
  generate_message_tostring_source(p);
  generate_message_struct2class_source(p);
  generate_message_class2struct_source(p);

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
    if (is_eigen_type(msg)) {
      continue;
    }

    auto raw_name = msg->name();
    auto name = qualified_class_name(msg, options_);
    format("// $1$\n", name);
    format("REFLECTION($1$", raw_name);
    for (int i = 0; i < msg->field_count(); ++i) {
      auto fd = msg->field(i);
      if (fd->options().GetExtension(inherits_from)) {
        auto fd2 = fd->message_type();
        for (int j = 0; j < fd2->field_count(); ++j) {
          format(", $1$", fd2->field(j)->name());
        }
      }
      else {
        format(", $1$", fd->name());
      }
    }
    format(");\n");
  }

  // enum
  // std::vector<const EnumDescriptor *> enums = flatten_enums_in_file(file_);
  // for (auto single_enum : enums) {
  //   auto name = qualified_enum_name(single_enum, options_);
  //   format("// $1$\n", name);
  //   format("extern std::map<$1$, std::string> k$2$_EnumToString;", name, resolve_keyword(single_enum->name()));
  //   format("\n");
  // }

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
    g.generate_struct_to_class_to(p);
    format("}\n");
  }

  // enum
  std::vector<const EnumDescriptor *> enums = flatten_enums_in_file(file_);
  for (auto single_enum : enums) {
    auto name = qualified_enum_name(single_enum, options_);
    EnumGenerator g(single_enum, options_);
    format("// $1$\n", name);
    // 枚举字符串
    // auto name = qualified_enum_name(d_, options_);
    format.indent();
    format("std::map<$1$, std::string> k$1$_EnumToString = { \n", single_enum->name());
    for (int i = 0; i < single_enum->value_count(); ++i) {
      auto value = resolve_keyword(single_enum->value(i)->name());
      auto number = single_enum->value(i)->number();
      auto enum_value = name + "::" + value;
      format.indent();
      format("{$1$ , \"$2$\"},\n", enum_value, value);
      format.outdent();
    }
    format("};\n");
    format.outdent();    
  }

}

void FileGenerator::generate_message_struct2class_definitions(
    google::protobuf::io2::Printer *p) {
  std::vector<const Descriptor *> msgs = flatten_messages_in_file(file_);
  Formatter format(p);
  for (auto msg : msgs) {
    auto struct_name = qualified_class_name(msg, options_);
    auto class_name = file_->package() + "::" + msg->name();
    // 非eigen 类型增加tostring
    if (msg->options().GetExtension(eigen_typename).empty()) {
      format("std::ostream& operator<<(std::ostream& os,$1$ const& in);\n",
             struct_name);
    }
    format("$1$ InnerStructToInnerClass($2$ const& in);\n",class_name,struct_name);
    format("$1$ InnerClassToInnerStruct($2$ const& in);\n\n",struct_name, class_name);
  }
}

void FileGenerator::generate_message_struct2class_source(
    google::protobuf::io2::Printer *p) {
  std::vector<const Descriptor *> msgs = flatten_messages_in_file(file_);
  Formatter format(p);
  for (auto msg : msgs) {
    MessageGenerator g(msg, options_);
    auto struct_name = qualified_class_name(msg, options_);
    auto class_name = file_->package() + "::" + msg->name();
    format("$1$ InnerStructToInnerClass($2$ const& in) {\n",class_name,struct_name);
    format.indent();
    format("$1$ result;\n",class_name);
    g.generate_struct_to_class_to(p);
    format("\nreturn result;\n");
    format.outdent();
    format("}\n");
  }
}

void FileGenerator::generate_message_class2struct_source(
    google::protobuf::io2::Printer *p) {
  std::vector<const Descriptor *> msgs = flatten_messages_in_file(file_);
  Formatter format(p);
  for (auto msg : msgs) {
    MessageGenerator g(msg, options_);
    auto struct_name = qualified_class_name(msg, options_);
    auto class_name = file_->package() + "::" + msg->name();
    format("$1$ InnerClassToInnerStruct($2$ const& in) {\n",struct_name, class_name);
    format.indent();
    format("$1$ result;\n",struct_name);
    g.generate_class_to_struct_to(p);
    format("\nreturn result;\n");
    format.outdent();
    format("}\n");
  }
}

void FileGenerator::generate_message_tostring_source(
    google::protobuf::io2::Printer *p) {


  std::vector<const Descriptor *> msgs = flatten_messages_in_file(file_);
  for (auto msg : msgs) {
    // eigen类型无 operatot <<定义
    if (!msg->options().GetExtension(eigen_typename).empty()) {
      continue;
    }
    MessageGenerator g(msg, options_);
    auto struct_name = qualified_class_name(msg, options_);
    auto class_name = file_->package() + "::" + msg->name();

    p->Print({{"struct_name",struct_name}},
  R"(std::ostream& operator<<(std::ostream& os, $struct_name$ const& in) {
  std::string result;
  iguana::to_json(in, result);
  os << result;
  return os;
}
)");

  }
}

}  // namespace compiler

}  // namespace struct_pb