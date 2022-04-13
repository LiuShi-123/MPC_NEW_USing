/*
 * @Author: LiuShi
 * @Date: 2022-04-08 17:32:31
 * @LastEditors: LiuShi
 * @LastEditTime: 2022-04-13 17:06:56
 * @FilePath: /mpc/include/MPCPlan.h
 * @Description: 
 * @custom_string_LiuShi: WeChat:18857488400; QQ:487196835; Email:liushibb@sina.com
 * @custom_string_LiuShi_copyright: Copyright (c) 2022 by Niubi Sparkle, All Rights Reserved. 
 */
#pragma once
#include <Eigen/Dense>
#include "osqp.h"
#include "OsqpEigen/OsqpEigen.h"
#include "Paras.h"
#include <iostream>
#include <stdio.h>
using Eigen::MatrixXd;
using Eigen::VectorXd;
class _MPCPlan
{
public:
  void cacu_MN(MatrixXd matrixC, MatrixXd matrixA,MatrixXd matrixB);
  Eigen::SparseMatrix<double> _matrixHessian();
  VectorXd _vectorGradient(MatrixXd _Kesi);
  Eigen::SparseMatrix<double> _matrixLinearMatrix();
  VectorXd _vectorLowerBound(MatrixXd _Kesi);
  VectorXd _vectorUpperBound(MatrixXd _Kesi);
  
private:
  _Paras _prs;
  MatrixXd _M;
  MatrixXd _N;
  MatrixXd _matrixPower(MatrixXd matrix, int power);
  void _matrixM(MatrixXd matrixC, MatrixXd matrixA);
  void _matrixN(MatrixXd matrixC, MatrixXd matrixA,MatrixXd matrixB);
};
