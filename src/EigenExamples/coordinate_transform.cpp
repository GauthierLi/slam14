#include <cmath>
#include <iostream>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;

int main (int argc, char *argv[]) {
  Quaterniond q1(0.35, 0.2, 0.3, 0.1), q2(-0.5, 0.4, -0.1, 0.2);
  Vector3d v1(0.3, 0.1, 0.1), v2(-0.1, 0.5, 0.3);
  q1.normalize();
  q2.normalize();

  Vector3d p1(0.5, 0, 0.2);

  Isometry3d T1w(q1), T2w(q2);
  T1w.pretranslate(v1);
  T2w.pretranslate(v2);

  cout << "T1w = \n" << T1w.matrix() << endl;
  cout << "T2w = \n" << T2w.matrix() << endl;

  Vector3d p2;
  p2 = T2w * T1w.inverse() * p1;
  cout << "p1's coord in q2: \n" << p2.transpose() << endl;
  return 0;
}
