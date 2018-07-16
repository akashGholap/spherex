#ifndef SPACETREX_KALMAN_FILTER_H
#define SPACETREX_KALMAN_FILTER_H
#include <Eigen/Dense>
//#include <armadillo>
#pragma once
namespace kf{


class kalman_filter{
public:

  int m_,n_;

  double u;

  bool setup_done;

  double T0 , t;

  double dt_;

  double Rx,Ry,Rz;

  Eigen::MatrixXd A_, B_;

  Eigen::MatrixXd C_;
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd K_;
  Eigen::MatrixXd P0_;

  Eigen::MatrixXd I;

  Eigen::VectorXd xhat, xhat_new, x_;

  std::vector<double> x_cm;



kalman_filter(int m, int n, double dt)
    :  dt_(dt),A_(m,n),B_(m,1),C_(m,n),Q_(m,n),R_(m,n),P_(m,n),K_(m,n),P0_(m,n),m_(m), n_(n), setup_done(true),
    I(n, n), xhat(n), xhat_new(n),x_(n), T0(0), t(0),u(1),Rx(0),Ry(0),Rz(0)
    {
       I.setIdentity();
       xhat.setZero();
       xhat_new.setZero();
       x_.setZero();
    }

void set_filter(double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& K,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P,
    const Eigen::MatrixXd& P0)
    {
      dt_ = dt;
      A_  = A;
      B_ = B;
      C_ = C;
      Q_ =Q;
      K_ =K;
      R_ = R;
      P_ = P;
      P0_ = P0;
    }

void predict() //following the trend class function implementations is done in headers
{
  xhat_new = A_ * xhat + B_*u;
  P_ = A_*P_*A_.transpose() + Q_;
}

void estimate(Eigen::Vector3d& V)
{
  xhat_new = A_ * xhat;

  P_ = A_*P_*A_.transpose() + Q_;

  K_ = P_*C_.transpose()*(C_*P_*C_.transpose() + R_).inverse();

  xhat_new += K_ * (V - C_*xhat_new);

  P_ = (I - K_*C_)*P_;

  xhat = xhat_new;

  t += dt_;

}


};
}

void set_filter_main();
#endif
