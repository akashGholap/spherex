#ifndef SPACETREX_KALMAN_FILTER_H
#define SPACETREX_KALMAN_FILTER_H
#include <eigen/dense>
#include <armadillo>
#pragma once
namespace kf{


class kalman_filter{
public:

kalmanFilter( double delt, const Eigen::MatrixXd& A,
      const Eigen::MatrixXd& C,
      const Eigen::MatrixXd& B,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& P );

void set_filter();

void predict();

void estimate();
