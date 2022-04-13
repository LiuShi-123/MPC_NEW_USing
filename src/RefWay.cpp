/*
 * @Author: LiuShi
 * @Date: 2022-04-08 16:35:10
 * @LastEditors: LiuShi
 * @LastEditTime: 2022-04-08 16:35:10
 * @FilePath: /MPC_NEW/src/RefWay.cpp
 * @Description: 
 * @custom_string_LiuShi: WeChat:18857488400; QQ:487196835; Email:liushibb@sina.com
 * @custom_string_LiuShi_copyright: Copyright (c) 2022 by Niubi Sparkle, All Rights Reserved. 
 */
#include "RefWay.h"
#include "CarModule.h"
#include <iostream>
#include <stdio.h>
using namespace std;
MatrixXd _RefWay::cacu_RW(string choose, double te)
{
  MatrixXd res = MatrixXd::Zero(_RefWay::_prs.Nx +_RefWay::_prs.Nx, 1);
  if (choose == "Circle") {
    res = _RefWay::_Circle(te);
  } else if (choose == "StrLine") {
    res = _RefWay::_StrLine(te);
  }
  return res;
}

MatrixXd _RefWay::_Circle(double te)
{
  _RefWay::rx = _RefWay::_prs.Ox + _RefWay::_prs.Or * sin(te * _RefWay::_prs.Ov / _RefWay::_prs.Or); 
  _RefWay::ry = _RefWay::_prs.Oy - _RefWay::_prs.Or * cos(te * _RefWay::_prs.Ov / _RefWay::_prs.Or); 
  _RefWay::rp = te * _RefWay::_prs.Ov / _RefWay::_prs.Or;
  _RefWay::rv = _RefWay::_prs.Ov;
  _RefWay::rd = atan(_RefWay::_prs.Carl / _RefWay::_prs.Or);
  MatrixXd RS = MatrixXd::Zero(_RefWay::_prs.Nx + _RefWay::_prs.Nu, 1);
  RS << _RefWay::rx, _RefWay::ry,_RefWay::rp,_RefWay::rv,_RefWay::rd;
  return RS;
}

MatrixXd _RefWay::_StrLine(double te)
{
  _RefWay::rx = _RefWay::_prs.Ov * te;
  _RefWay::ry = 0.;
  _RefWay::rp = 0.;
  _RefWay::rv = _RefWay::_prs.Ov;
  _RefWay::rd = 0.;
  MatrixXd RS = MatrixXd::Zero(_RefWay::_prs.Nx + _RefWay::_prs.Nu, 1);
  RS << _RefWay::rx, _RefWay::ry,_RefWay::rp,_RefWay::rv,_RefWay::rd;
  return RS;
}
