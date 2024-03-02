#include "EnumGenerator.h"

#include "helpers.hpp"
namespace struct_pb {
namespace compiler {
void EnumGenerator::generate(google::protobuf::io::Printer *p) {
  std::string enum_name = resolve_keyword(d_->name());
  p->Print({{"enum_name", enum_name}}, R"(
enum class $enum_name$ {
)");
  for (int j = 0; j < d_->value_count(); ++j) {
    auto v = d_->value(j);
    p->Print({{"name", resolve_keyword(v->name())},
              {"value", std::to_string(v->number())}},
             R"(
$name$ = $value$,
)");
  }
  p->Print(
      R"(
};
)");
}
void EnumGenerator::generate_definition(google::protobuf::io::Printer *p) {
  Formatter format(p);
  format("enum class $1$: int {\n", resolve_keyword(d_->name()));
  format.indent();
  for (int i = 0; i < d_->value_count(); ++i) {
    auto value = resolve_keyword(d_->value(i)->name());
    auto number = d_->value(i)->number();
    format("$1$ = $2$,\n", value, number);
  }
  format.outdent();
  format("};\n");
}

void EnumGenerator::generateMapDeclaration(google::protobuf::io::Printer *p)
{
  Formatter format(p);

  format("extern std::map<$1$, std::string> k$2$_EnumToString \n", resolve_keyword(d_->full_name()), resolve_keyword(d_->name()));
  
}

void EnumGenerator::generateMapDefinition(google::protobuf::io::Printer *p)
{
  Formatter format(p);
  // 定义静态的map
  auto name = qualified_enum_name(d_, options_);
  format.indent();
  format("static std::map<$1$, std::string> k$2$_EnumToString = { \n", name, resolve_keyword(d_->name()));
  for (int i = 0; i < d_->value_count(); ++i) {
    auto value = resolve_keyword(d_->value(i)->name());
    auto number = d_->value(i)->number();
    auto enum_value = name + "::" + value;
    format.indent();
    format("{$1$ , \"$2$\"},\n", enum_value, value);
    format.outdent();
  }
  format("};\n");
  format.outdent();
  // tostring
  format.indent();
  format("return k$1$_EnumToString[t];\n", resolve_keyword(d_->name()));
  format.outdent();

  
}


}  // namespace compiler
}  // namespace struct_pb
