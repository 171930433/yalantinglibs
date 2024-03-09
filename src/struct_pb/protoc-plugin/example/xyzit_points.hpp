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