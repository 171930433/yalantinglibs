#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>

// #include "addressbook.struct_pb.h"
#include "src/example/message/imu.struct_pb.h"
// #include "src/message/zimage.hpp"
// #include "src/message/zpointcloud.hpp"

// void PointcloudDemo() {
//   using namespace zhito;
//   pcl::PointXYZIT p2{1, 2};

//   inner_struct::spZPointCloudXYZIT pc = std::make_shared<inner_struct::ZPointCloudXYZIT>();
//   *(std::shared_ptr<inner_struct::ZFrame>)pc =
//       inner_struct::ZFrame{1, 2, 3, inner_struct::ZFrameType::PointCloud, "/zhito/pointcloud"};
//   for (int i = 0; i < 1e5; ++i) {
//     p2.x += (rand() % 10000 * 0.0001);
//     pc->push_back(p2);
//   }

//   inner_class::ZPointCloudXYZIT pc2 = converter::StructToClass(pc);
//   inner_struct::spZPointCloudXYZIT pc3 = converter::ClassToStruct(pc2);

//   // pcl::io::savePCDFileBinaryCompressed("1.pcd", *pc3);

//   std::cout << *pc3 << std::endl;
// }

void ImuDemo() {
  using namespace zhito;

  inner_struct::ZImu imu1{1, 2, 3, inner_struct::ZFrameType::IMU, "channel_name", {4, 5, 6}, {7, 8, 9}, 10};
  inner_class::ZImu imu2 = converter::StructToClass(imu1);
  inner_struct::ZImu imu3 = converter::ClassToStruct(imu2);

  std::cout << imu1 << std::endl;
  std::cout << imu2.ShortDebugString() << std::endl;
  std::cout << imu3 << std::endl;
}

// void ImageDemo() {
//   using namespace zhito;

//   inner_struct::ZImage image1{1, 2, 3, inner_struct::ZFrameType::CvImage, "/zhito/image"};
//   (cv::Mat&)image1 = cv::Mat::eye(4, 4, CV_64F);

//   inner_class::ZImage image2 = converter::StructToClass(image1);
//   inner_struct::ZImage image3 = converter::ClassToStruct(image2);

//   std::cout << image1 << std::endl;
//   std::cout << image2.ShortDebugString() << std::endl;
//   std::cout << image3 << std::endl;
// }

int main() {
  ImuDemo();
  // PointcloudDemo();
  // ImageDemo();

  std::cout << "Done!!!" << std::endl;

  return 0;
}
