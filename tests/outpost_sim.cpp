#include <iostream>
#include <Eigen/Dense>
#include "tools/extended_kalman_filter.hpp"
#include "tools/math_tools.hpp"

using namespace std;
using namespace tools;

// This is a small offline simulation that mimics the EKF measurement
// for a 3-panel outpost. It demonstrates that the EKF can estimate h.

Eigen::Vector4d h_measure(const Eigen::VectorXd &x, int id, int armor_num)
{
  // x: [cx, vx, cy, vy, cz, vz, a, w, r, l, h]
  double angle = tools::limit_rad(x[6] + id * 2 * M_PI / armor_num);
  double r = x[8];
  double half = (armor_num - 1) / 2.0;
  double dz = (half - id) * x[10];
  double armor_x = x[0] - r * cos(angle);
  double armor_y = x[2] - r * sin(angle);
  double armor_z = x[4] + dz;

  // approximate ypd from xyz: yaw, pitch, distance
  double yaw = atan2(armor_y, armor_x);
  double distance = sqrt(armor_x * armor_x + armor_y * armor_y + armor_z * armor_z);
  double pitch = atan2(-armor_z, sqrt(armor_x * armor_x + armor_y * armor_y));

  return Eigen::Vector4d(yaw, pitch, distance, angle);
}

int main()
{
  int armor_num = 3;
  // true state
  Eigen::VectorXd x_true(11);
  x_true << 1.0, 0, 0.5, 0, 0.5, 0, 0.3, 0.1, 0.2, 0.0, 0.10; // h=0.1m

  // initial estimate (h unknown)
  Eigen::VectorXd x0(11);
  x0 << 1.0, 0, 0.5, 0, 0.5, 0, 0.3, 0.0, 0.2, 0.0, 0.0; // h=0

  Eigen::VectorXd P0_dig(11);
  P0_dig << 1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 1e-2;
  Eigen::MatrixXd P0 = P0_dig.asDiagonal();

  auto x_add = [](const Eigen::VectorXd &a, const Eigen::VectorXd &b) {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  tools::ExtendedKalmanFilter ekf(x0, P0, x_add);

  // simple predict model: identity with small motion
  double dt = 0.05;
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(11, 11);
  F(0,1) = dt; F(2,3) = dt; F(4,5) = dt; F(6,7) = dt;

  double v1 = 10, v2 = 0.1;
  double a = dt*dt*dt*dt/4;
  double b = dt*dt*dt/2;
  double c = dt*dt;
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(11,11);
  Q(0,0)=a*v1; Q(0,1)=b*v1; Q(1,0)=b*v1; Q(1,1)=c*v1;
  Q(2,2)=a*v1; Q(2,3)=b*v1; Q(3,2)=b*v1; Q(3,3)=c*v1;
  Q(4,4)=a*v1; Q(4,5)=b*v1; Q(5,4)=b*v1; Q(5,5)=c*v1;
  Q(6,6)=a*v2; Q(6,7)=b*v2; Q(7,6)=b*v2; Q(7,7)=c*v2;

  for (int step = 0; step < 60; ++step) {
    // simulate true motion (static here)

    // predict
    ekf.predict(F, Q, [&](const Eigen::VectorXd &x){
      Eigen::VectorXd xp = F * x; xp[6] = tools::limit_rad(xp[6]); return xp; });

    // generate a noisy measurement from panel id=0,1,2 in turn
    int id = step % armor_num;
    Eigen::Vector4d z_true = h_measure(x_true, id, armor_num);
    Eigen::Vector4d z = z_true;
    // add small noise
    z[0] += 0.01 * ((rand()%100)/100.0 - 0.5);
    z[1] += 0.005 * ((rand()%100)/100.0 - 0.5);
    z[2] += 0.02 * ((rand()%100)/100.0 - 0.5);
    z[3] += 0.01 * ((rand()%100)/100.0 - 0.5);

    // compute H numerically (finite difference)
    Eigen::MatrixXd H(4,11);
    double eps = 1e-6;
    Eigen::Vector4d h0 = h_measure(ekf.x, id, armor_num);
    for (int j=0;j<11;j++){
      Eigen::VectorXd xt = ekf.x;
      xt[j] += eps;
      Eigen::Vector4d hj = h_measure(xt, id, armor_num);
      H.col(j) = (hj - h0) / eps;
    }

    // measurement noise R
    Eigen::Vector4d Rdig; Rdig << 4e-3, 4e-3, 1, 9e-2;
    Eigen::MatrixXd R = Rdig.asDiagonal();

    // update
    ekf.update(z, H, R, [&](const Eigen::VectorXd &x){ return h_measure(x, id, armor_num); });

    cout << "step="<<step<<" id="<<id<<" est_h="<<ekf.x[10]<<" true_h="<<x_true[10]<<"\n";
  }

  return 0;
}
