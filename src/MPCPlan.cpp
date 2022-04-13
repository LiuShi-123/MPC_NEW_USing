/*
 * @Author: LiuShi
 * @Date: 2022-04-08 17:32:41
 * @LastEditors: LiuShi
 * @LastEditTime: 2022-04-13 17:44:30
 * @FilePath: /mpc/src/MPCPlan.cpp
 * @Description:
 * @custom_string_LiuShi: WeChat:18857488400; QQ:487196835; Email:liushibb@sina.com
 * @custom_string_LiuShi_copyright: Copyright (c) 2022 by Niubi Sparkle, All Rights Reserved.
 */
#include "MPCPlan.h"
MatrixXd _MPCPlan::_matrixPower(MatrixXd matrix, int power)
{
  MatrixXd matrixP = matrix;
  int i = 1;
  if (power == 0) {
    matrixP = MatrixXd::Identity(_MPCPlan::_prs.Nu + _MPCPlan::_prs.Nx, _MPCPlan::_prs.Nu + _MPCPlan::_prs.Nx);
  } else if (power == 1) {
    matrixP = matrixP;
  } else {
    while (i <= power - 1) {
      matrixP = matrixP * matrix;
      i = i + 1;
    }
    matrixP = matrixP;
  }
  return matrixP;
}
void _MPCPlan::_matrixM(MatrixXd matrixC, MatrixXd matrixA)//youwenti!!!!!!!!!!
{
  MatrixXd _m(_MPCPlan::_prs.Nx * _MPCPlan::_prs.Np, _MPCPlan::_prs.Nu + _MPCPlan::_prs.Nx);
  MatrixXd _temp = matrixA;
  int i = 0;
  while(i < _MPCPlan::_prs.Np){
    _temp = _MPCPlan::_matrixPower(matrixA, i + 1);
    _m.block(_MPCPlan::_prs.Nx * i, 0, _MPCPlan::_prs.Nx, _MPCPlan::_prs.Nx + _MPCPlan::_prs.Nu) = matrixC * _temp;
    i = i + 1;
  }
  _MPCPlan::_M =_m;
}
void _MPCPlan::_matrixN(MatrixXd matrixC, MatrixXd matrixA, MatrixXd matrixB)//youwenti!!!!!!!!!!
{
  int h = 0;
  int l = 0;
  MatrixXd _n(_MPCPlan::_prs.Nx * _MPCPlan::_prs.Np, _MPCPlan::_prs.Nu * _MPCPlan::_prs.Nc);
  MatrixXd temp_Block = Eigen::MatrixXd::Zero(_MPCPlan::_prs.Nx, _MPCPlan::_prs.Nu);
  MatrixXd temp_Power = matrixA;
  int  j = 0;
  while(l < _MPCPlan::_prs.Nc){
    h = 0;
    j = 0;
    while(h < _MPCPlan::_prs.Np){
      if(h < l){
        temp_Block = MatrixXd::Zero(_MPCPlan::_prs.Nx, _MPCPlan::_prs.Nu);
      }else if(h >= l){
        temp_Power = _MPCPlan::_matrixPower(matrixA, j);
        temp_Block = matrixC * temp_Power;
        temp_Block = temp_Block * matrixB;//???这里吗
        j = j + 1;
      }
      _n.block(_MPCPlan::_prs.Nx * h, _MPCPlan::_prs.Nu * l, _MPCPlan::_prs.Nx, _MPCPlan::_prs.Nu) = temp_Block;
      h = h + 1;
    }
    l = l + 1;
  }
  _MPCPlan::_N =_n;
}
Eigen::SparseMatrix<double> _MPCPlan::_matrixHessian(){//youwenti!!!!!!!!!!!!!!!!!!!!!!
  MatrixXd _Q = _MPCPlan::_prs.Q * MatrixXd::Identity(_MPCPlan::_prs.Nx * _MPCPlan::_prs.Np, _MPCPlan::_prs.Nx * _MPCPlan::_prs.Np);
  MatrixXd _R = _MPCPlan::_prs.R * MatrixXd::Identity(_MPCPlan::_prs.Nc * _MPCPlan::_prs.Nu, _MPCPlan::_prs.Nc * _MPCPlan::_prs.Nu);
  MatrixXd _Hessian = MatrixXd::Zero(_MPCPlan::_prs.Nc * _MPCPlan::_prs.Nu + 1, _MPCPlan::_prs.Nc * _MPCPlan::_prs.Nu + 1);
  _Hessian << _MPCPlan::_N.transpose() * _Q * _MPCPlan::_N + _R, MatrixXd::Zero(_MPCPlan::_prs.Nc * _MPCPlan::_prs.Nu, 1),
           MatrixXd::Zero(1,_MPCPlan::_prs.Nc * _MPCPlan::_prs.Nu), _MPCPlan::_prs.Row;
  //_Hessian = 2 * _Hessian;
  _Hessian = 0.5 * (_Hessian + _Hessian.transpose());
  return _Hessian.sparseView();
}

Eigen::SparseMatrix<double> _MPCPlan::_matrixLinearMatrix(){
  MatrixXd _LinearMatrix= MatrixXd::Zero(_MPCPlan::_prs.Nc * _MPCPlan::_prs.Nu, _MPCPlan::_prs.Nc * _MPCPlan::_prs.Nu + 1);//change
  MatrixXd _OneCol= MatrixXd::Zero(_MPCPlan::_prs.Nc * _MPCPlan::_prs.Nu, 1);
  int h = 0;
  int l = 0;
  while(l < _MPCPlan::_prs.Nc){
    h = 0;
    while(h < _MPCPlan::_prs.Nc){
      if(h < l){
        _LinearMatrix.block(h * _MPCPlan::_prs.Nu, l * _MPCPlan::_prs.Nu, _MPCPlan::_prs.Nu, _MPCPlan::_prs.Nu) = MatrixXd::Zero(_MPCPlan::_prs.Nu, _MPCPlan::_prs.Nu);
      }else if(h >= l){
        _LinearMatrix.block(h * _MPCPlan::_prs.Nu, l * _MPCPlan::_prs.Nu, _MPCPlan::_prs.Nu, _MPCPlan::_prs.Nu) = MatrixXd::Identity(_MPCPlan::_prs.Nu, _MPCPlan::_prs.Nu);
      }
      h = h + 1;
    }
    l = l + 1;
  }
  h = 0;
  while(h < _MPCPlan::_prs.Nu * _MPCPlan::_prs.Nc){
    _OneCol(h, 0) = _MPCPlan::_prs.yp;
    h = h + 1;
  }
  _LinearMatrix.block(0, _MPCPlan::_prs.Nu * _MPCPlan::_prs.Nc, _MPCPlan::_prs.Nu * _MPCPlan::_prs.Nc, 1) = _OneCol;//change
  return _LinearMatrix.sparseView();
}

VectorXd _MPCPlan::_vectorGradient(MatrixXd _Kesi){//youwenti!!!!!!!!!!!!!!!!!
  MatrixXd __Q = _MPCPlan::_prs.Q * MatrixXd::Identity(_MPCPlan::_prs.Nx * _MPCPlan::_prs.Np, _MPCPlan::_prs.Nx * _MPCPlan::_prs.Np);
  MatrixXd _GT= MatrixXd::Zero(1, _MPCPlan::_prs.Nu * _MPCPlan::_prs.Nc + 1);
  MatrixXd _E = _MPCPlan::_M * _Kesi;
  _GT << _E.transpose() * __Q * _MPCPlan::_N, 0.;
  //_GT = 2 * _GT;
  VectorXd _G(_MPCPlan::_prs.Nu * _MPCPlan::_prs.Nc + 1);
  int i = 0;
  while(i < _MPCPlan::_prs.Nu * _MPCPlan::_prs.Nc + 1){
    _G(i) = _GT(0, i);
    i = i + 1;
  }
  return _G;
}

VectorXd _MPCPlan::_vectorLowerBound(MatrixXd _Kesi){
  MatrixXd UT = MatrixXd::Zero(_MPCPlan::_prs.Nc * _MPCPlan::_prs.Nu, 1);
  MatrixXd ut = MatrixXd::Zero(_MPCPlan::_prs.Nu, 1);
  MatrixXd umin = MatrixXd::Zero(_MPCPlan::_prs.Nu, 1);
  MatrixXd U_piao_min = MatrixXd::Zero(_MPCPlan::_prs.Nu * _MPCPlan::_prs.Nc, 1);
  umin << -_MPCPlan::_prs.U0,-_MPCPlan::_prs.U1;
  ut << _Kesi(_MPCPlan::_prs.Nx, 0), _Kesi(_MPCPlan::_prs.Nx + 1, 0);
  int i = 0;
  while (i < _MPCPlan::_prs.Nc)
  {
    U_piao_min.block(i * _MPCPlan::_prs.Nu, 0, _MPCPlan::_prs.Nu, 1) = umin;
    i = i + 1;
  }
  i = 0;
  while (i < _MPCPlan::_prs.Nc)
  {
    UT.block(i * _MPCPlan::_prs.Nu, 0, _MPCPlan::_prs.Nu, 1) = ut;
    i = i + 1;
  }
  VectorXd U_min(_MPCPlan::_prs.Nu * _MPCPlan::_prs.Nc);
  MatrixXd temp = U_piao_min - UT;
  i = 0;
  while (i < _MPCPlan::_prs.Nc * _MPCPlan::_prs.Nu)
  {
    U_min(i) = temp(i, 0);
    i = i + 1;
  }
  return U_min;
}

VectorXd _MPCPlan::_vectorUpperBound(MatrixXd _Kesi){
  MatrixXd UT = MatrixXd::Zero(_MPCPlan::_prs.Nc * _MPCPlan::_prs.Nu, 1);
  MatrixXd ut = MatrixXd::Zero(_MPCPlan::_prs.Nu, 1);
  MatrixXd umax = MatrixXd::Zero(_MPCPlan::_prs.Nu, 1);
  MatrixXd U_piao_max = MatrixXd::Zero(_MPCPlan::_prs.Nu * _MPCPlan::_prs.Nc, 1);
  umax << _MPCPlan::_prs.U0, _MPCPlan::_prs.U1;
  ut << _Kesi(_MPCPlan::_prs.Nx, 0), _Kesi(_MPCPlan::_prs.Nx + 1, 0);
  int i = 0;
  while (i < _MPCPlan::_prs.Nc)
  {
    U_piao_max.block(i * _MPCPlan::_prs.Nu, 0, _MPCPlan::_prs.Nu, 1) = umax;
    i = i + 1;
  }
  i = 0;
  while (i < _MPCPlan::_prs.Nc)
  {
    UT.block(i * _MPCPlan::_prs.Nu, 0, _MPCPlan::_prs.Nu, 1) = ut;
    i = i + 1;
  }
  VectorXd U_max(_MPCPlan::_prs.Nu * _MPCPlan::_prs.Nc);
  MatrixXd temp = U_piao_max - UT;
  i = 0;
  while (i < _MPCPlan::_prs.Nc * _MPCPlan::_prs.Nu)
  {
    U_max(i) = temp(i, 0);
    i = i + 1;
  }
  return U_max;
}

void _MPCPlan::cacu_MN(MatrixXd matrixC, MatrixXd matrixA,MatrixXd matrixB){
  _MPCPlan::_matrixM(matrixC, matrixA);
  _MPCPlan::_matrixN(matrixC, matrixA, matrixB);
}
