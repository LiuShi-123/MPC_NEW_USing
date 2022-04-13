/*
 * @Author: LiuShi
 * @Date: 2022-04-08 16:35:19
 * @LastEditors: LiuShi
 * @LastEditTime: 2022-04-08 16:35:19
 * @FilePath: /MPC_NEW/include/RefWay.h
 * @Description: 
 * @custom_string_LiuShi: WeChat:18857488400; QQ:487196835; Email:liushibb@sina.com
 * @custom_string_LiuShi_copyright: Copyright (c) 2022 by Niubi Sparkle, All Rights Reserved. 
 */
#pragma once
#include "Paras.h"
#include <Eigen/Dense>
#include <string>
#include <cmath>
using namespace std;
using Eigen::MatrixXd;
class _RefWay
{
public:
  double rx;
  double ry;
  double rp;
  double rv;
  double ra;
  double rd;
  MatrixXd cacu_RW(string choose,double te);
private:
  _Paras _prs;
  MatrixXd _Circle(double te);
  MatrixXd _StrLine(double te);
};
