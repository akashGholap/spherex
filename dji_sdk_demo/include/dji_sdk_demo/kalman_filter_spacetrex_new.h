#ifndef SPACETREX_KALMAN_FILTER_H
#define SPACETREX_KALMAN_FILTER_H
#include <eigen/dense>
#include <armadillo>
#pragma once
namespace kf{


class kalman_filter{
public:

  int m,n;

  double u;

  bool setup_done;

  double T0 , t;

  double dt_,

  Eigen::MatrixXd A_, B_, C_, Q_, R_, P_, K_, P0_;

  Eigen::MatrixXd I;

  Eigen::VectorXd xhat, xhat_new, x_;

  std::vector<double> x_cm;



kalman_filter(  double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P,
    const Eigen::MatrixXd& P0,
  )
    :  dt_(dt),A_(A),B_(B),C_(C),Q_(Q),R_(R),P_(P),K_(K),P0_(P0),m(C.rows()), n(A.rows()), setup_done(true),
    I(n, n), xhat(n), xhat_new(n),x_(n), T0(0), t(0),u(1)
    {
       I.setIdentity();
       xhat.setZero();
       xhat_new.setZero();
       x_.setZero();
    }


void predict() //following the trend class function implementations is done in headers 
{
  xhat_new = A * x_hat + B*u;
  P = A*P*A.transpose() + Q;
}

void estimate(Eigen::Vector3d& V)
{
  xhat_new = A * xhat;

  P = A*P*A.transpose() + Q;

  K = P*C.transpose()*(C*P*C.transpose() + R).inverse();

  xhat_new += K * (V - C*xhat_new);

  P = (I - K*C)*P;

  xhat = xhat_new;

  t += dt;

}


};
}

void set_filter();
