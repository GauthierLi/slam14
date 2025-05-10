#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

int main(int argc, char **argv){
  std::cout << "Hello Eigen!" << std::endl;
  Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
  Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0,0,1));
  rotation_matrix = rotation_vector.toRotationMatrix();

  cout.precision(3);
  cout <<"rotation_matrix = \n"
       << rotation_matrix << endl;

  cout <<"rotation_vector = \n"
       << rotation_vector.matrix() << endl;

  Eigen::Vector3d eular_angle = rotation_matrix.eulerAngles(2,1,0); // roll pitch yaw
  cout << "eular_angle = \n"
       << eular_angle.transpose() << endl;

  // 欧式变换矩阵
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  cout << "Eula trainsform matric: \n" << T.matrix() << endl;

  Eigen::Vector3d v(0,0,1);
  cout << "Identity transformation: \n"<< T * v << endl;

  // quaternion
  Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
  cout << "quaternion from rotation vector: \n(x, y, z, w) \n"
       << q.coeffs().transpose() << endl;
  return 0;
}
