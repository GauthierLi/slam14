#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

using namespace std;

namespace Eigen{
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
}

int main(int argc, char **argv) {
  // rotation 90 degree by z axis
  Eigen::Matrix3d R =
     Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
  Eigen::Quaterniond q(R); // Quaternion
  
  Sophus::SO3d SO3_R(R); // Sophus
  Sophus::SO3d SO3_q(q); // Sophus

  // 二者等价
  std::cout << "SO3 from matrix: \n" << SO3_R.matrix() << std::endl;
  std::cout << "SO3 from quaternion: \n" << SO3_q.matrix() << std::endl;
  std::cout << "Those are equal!" << std::endl;

  // 使用对数映射获得他的李代数
  Eigen::Vector3d so3 = SO3_R.log();
  std::cout << "so3 = " << so3.transpose() << std::endl;
  // hat 为向量得到的反对称矩阵
  std::cout << "so3 hat = \n" << Sophus::SO3d::hat(so3) << std::endl;
  // 相反的，vee 为反对称向量 
  std::cout << "so3 vee = \n" 
            << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() 
            << std::endl;

  // 增量扰动模型的更新
  Eigen::Vector3d update_so3(1e-6, 0, 0); // 设置更新量
  Sophus::SO3d SO3_update = Sophus::SO3d::exp(update_so3) * SO3_R; // 更新
  std::cout << "SO3 update: \n" << SO3_update.matrix() << std::endl;
  std::cout << "*************************************" << std::endl;


  // 对SE(3)的操作
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Quaterniond q_rotation(rotation); // 单位四元数
  Eigen::Vector3d t(1, 0, 0); // 沿着x轴平移1
  Sophus::SE3d SE3_Rt(R, t); // 旋转+平移
  Sophus::SE3d SE3_qt(q, t); 
  std::cout <<"rotation: \n" << rotation << std::endl;

  std::cout << "SE3 from Rotation: \n" << SE3_Rt.matrix() << std::endl;
  std::cout << "SE3 from quaternion: \n" << SE3_qt.matrix() << std::endl;

  Eigen::Vector6d se3 = SE3_Rt.log(); // 对数映射
  std::cout << "se3 = " << se3.transpose() << std::endl;
  cout << "se3 hat: \n" << Sophus::SE3d::hat(se3) << endl;
  cout << "se3 hat vee: \n" << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl;
  return 0;
}
