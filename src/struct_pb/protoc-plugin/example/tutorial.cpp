#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>

// #include "addressbook.struct_pb.h"
#include "imu.struct_pb.h"
#include <pcl/compression/organized_pointcloud_compression.h>

#include <pcl/point_types.h>
namespace pcl 
{
struct EIGEN_ALIGN16 _PointXYZIT {
  PCL_ADD_POINT4D  // This adds the members x,y,z which can also be accessed
                   // using the point (which is float[4])
  union {
    struct {
      float intensity;
      float dt;  // 添加 intensity 属性
    };
    float data_c[4];
  };
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

struct PointXYZIT : public _PointXYZIT
{
  inline constexpr PointXYZIT (const _PointXYZIT &p) : PointXYZIT{p.x, p.y, p.z, p.intensity, p.dt} {}

  inline constexpr PointXYZIT (float _intensity = 0.f, float _dt = 0.f) : PointXYZIT(0.f, 0.f, 0.f, _intensity,_dt) {}

  inline constexpr PointXYZIT (float _x, float _y, float _z, float _intensity = 0.f, float _dt = 0.f) : _PointXYZIT{{{_x, _y, _z, 1.0f}}, {{_intensity,_dt}}} {}
  
  friend std::ostream& operator << (std::ostream& os, const PointXYZIT& p);
};

inline std::ostream& operator << (std::ostream& os, const PointXYZIT& p)
{
    os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.intensity <<" " << p.dt << ")";
    return (os); 
}

}

// 声明 PCL 的宏定义以实现点类型
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIT,               // 自定义点类型的名称
                                  (float, x, x)             // 属性 x
                                  (float, y, y)             // 属性 y
                                  (float, z, z)             // 属性 z
                                  (float, intensity, intensity) // 属性 intensity
                                  (float, dt, dt) // 属性 dt
)


int main() {

  inner_struct::ZImu imu1{1,2,3,inner_struct::ZFrameType::IMU,"channel_name", {4,5,6}, {7,8,9}, 10};
  inner_class::ZImu imu2 = converter::StructToClass(imu1);
  inner_struct::ZImu imu3 = converter::ClassToStruct(imu2);


  // std::cout << imu1 << std::endl;
  // std::cout << imu2.ShortDebugString() << std::endl;
  // std::cout << imu3 << std::endl;


  // inner_struct::ZPointCloudXYZI pc1;
  // pc1.points.push_back(pcl::PointXYZI());
  // pc1.points.push_back(pcl::PointXYZI());
  // pcl::PointXYZI p1;
  pcl::PointXYZI p1 {1};
  std::cout << p1 << std::endl;

  pcl::PointXYZIT p2 {1,2};
  std::cout << p2 << std::endl;


  std::cout << "Done!!!" << std::endl;

  return 0;
}
