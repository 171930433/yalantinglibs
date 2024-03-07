#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>

// #include "addressbook.struct_pb.h"
#include "imu.struct_pb.h"

int main() {

  inner_struct::ZImu imu1{1,2,3,inner_struct::ZFrameType::IMU,"channel_name", {4,5,6}, {7,8,9}, 10};
  inner_class::ZImu imu2 = converter::StructToClass(imu1);
  inner_struct::ZImu imu3 = converter::ClassToStruct(imu2);


  std::cout << imu1 << std::endl;
  std::cout << imu2.ShortDebugString() << std::endl;
  std::cout << imu3 << std::endl;


  // inner_struct::ZPointCloudXYZI pc1;
  // pc1.points.push_back(pcl::PointXYZI());
  // pc1.points.push_back(pcl::PointXYZI());
  // pcl::PointXYZI p1;

  // std::cout << p1 << std::endl;


  std::cout << "Done!!!" << std::endl;

  return 0;
}
