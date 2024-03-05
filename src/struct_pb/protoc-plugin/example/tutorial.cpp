#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>

// #include "addressbook.struct_pb.h"
#include "imu.struct_pb.h"
// protobuf
#include <google/protobuf/util/json_util.h>




int main() {

  inner_struct::ZImu imu1{1,2,3,inner_struct::ZFrameType::IMU,"channel_name", {4,5,6}, {7,8,9}, 10};
  inner_class::ZImu imu2 = inner_struct::InnerStructToInnerClass(imu1);
  inner_struct::ZImu imu3 = inner_struct::InnerClassToInnerStruct(imu2);




  std::cout << imu1 << std::endl;
  std::cout << imu2.DebugString() << std::endl;
  std::cout << imu3 << std::endl;



  std::cout << "Done!!!" << std::endl;

  return 0;
}
