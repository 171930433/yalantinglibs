#pragma once
#include <string>
#include <algorithm>

#include "google/protobuf/descriptor.h"

// added
#include "proto_to_struct.pb.h"

struct Options {
  Options(const google::protobuf::FileDescriptor* f) : f(f) {}
  bool generate_eq_op = false;
  std::string ns;
  const google::protobuf::FileDescriptor* f;
};