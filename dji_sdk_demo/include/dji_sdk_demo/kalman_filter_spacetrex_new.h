#ifndef SPACETREX_KALMAN_FILTER_H
#define SPACETREX_KALMAN_FILTER_H
#include <eigen/dense>
#include <armadillo>
#pragma once
namespace kf{


class kalman_filter{
public:

  int m,n;

  bool setup_done;

  double T0 , t;

  double dt_,

  Eigen::MatrixXd A_, B_, C_, Q_, R_, P_, K_, P0_;

  Eigen::MatrixXd I;

  Eigen::VectorXd x_hat, x_hat_new, x_;

  std::vector<double> x_cm;



kalman_filter(  double dt,
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P,
    const Eigen::MatrixXd& P0,
  )
    :  dt_(dt),A_(A),B_(B),C_(C),Q_(Q),R_(R),P_(P),K_(K),P0_(P0),m(C.rows()), n(A.rows()), setup_done(false),
    I(n, n), x_hat(n), x_hat_new(n),x_(n){ I.setIdentity();}


void predict()
{
  x_hat_new = A * x_hat;
  P = A*P*A.transpose() + Q;
}

void estimate()
{
  x_hat_new = A * x_hat;
  P = A*P*A.transpose() + Q;
  K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
  x_hat_new += K * (y - C*x_hat_new);
  P = (I - K*C)*P;
  x_hat = x_hat_new;

  t += dt;

}


};
}
