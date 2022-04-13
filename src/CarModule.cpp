/*
 * @Author: LiuShi
 * @Date: 2022-04-13 14:05:38
 * @LastEditors: LiuShi
 * @LastEditTime: 2022-04-13 18:06:53
 * @FilePath: /mpc/src/CarModule.cpp
 * @Description: 
 * @custom_string_LiuShi: WeChat:18857488400; QQ:487196835; Email:liushibb@sina.com
 * @custom_string_LiuShi_copyright: Copyright (c) 2022 by Niubi Sparkle, All Rights Reserved. 
 */
#include "CarModule.h"
#include <iostream>
#include <stdio.h>
void _CarModule::initialStatus()
{
  _CarModule::Xk = 0.;
  _CarModule::Yk = 0.;
  _CarModule::Phik = 0.;
  _CarModule::Xkp1 = 0.;
  _CarModule::Ykp1 = 0.;
  _CarModule::Phikp1 = 0.;
}
void _CarModule::CarMove(double v, double delta, double Carl)
{
  _CarModule::Xkp1 = _CarModule::Xk + v * cos(_CarModule::Phik) * _CarModule::_pr.dt;
  _CarModule::Ykp1 = _CarModule::Yk + v * sin(_CarModule::Phik) * _CarModule::_pr.dt;
  _CarModule::Phikp1 = _CarModule::Phik + (v / _CarModule::_pr.Carl) * tan(delta) * _CarModule::_pr.dt;
  _CarModule::Xk = _CarModule::Xkp1;
  _CarModule::Yk = _CarModule::Ykp1;
  _CarModule::Phik = _CarModule::Phikp1;
  
}

void _CarModule::cacu_ABC(double rv, double rp, double rd)
{
  _CarModule::A = MatrixXd::Zero(_CarModule::_pr.Nu + _CarModule::_pr.Nx, _CarModule::_pr.Nu + _CarModule::_pr.Nx);
  _CarModule::B = MatrixXd::Zero(_CarModule::_pr.Nu + _CarModule::_pr.Nx, _CarModule::_pr.Nu);
  _CarModule::C = MatrixXd::Zero(_CarModule::_pr.Nx, _CarModule::_pr.Nu + _CarModule::_pr.Nx);
  _CarModule::A_Start = MatrixXd::Identity(_CarModule::_pr.Nx, _CarModule::_pr.Nx);
  _CarModule::B_Start = MatrixXd::Zero(_CarModule::_pr.Nx, _CarModule::_pr.Nu);
  _CarModule::A_Start(0, 2) = -rv * _CarModule::_pr.dt * sin(rp);
  _CarModule::A_Start(1, 2) = rv * _CarModule::_pr.dt * cos(rp);
  _CarModule::B_Start(0, 0) = _CarModule::_pr.dt * cos(rp);
  _CarModule::B_Start(1, 0) = _CarModule::_pr.dt * sin(rp);
  _CarModule::B_Start(2, 0) = _CarModule::_pr.dt * tan(rd) / _CarModule::_pr.Carl;
  _CarModule::B_Start(2, 1) = _CarModule::_pr.dt * rv / _CarModule::_pr.Carl / cos(rd) / cos(rd);
  _CarModule::A << _CarModule::A_Start, _CarModule::B_Start, MatrixXd::Zero(_CarModule::_pr.Nu, _CarModule::_pr.Nx), MatrixXd::Identity(_CarModule::_pr.Nu, _CarModule::_pr.Nu);
  _CarModule::B << _CarModule::B_Start, MatrixXd::Identity(_CarModule::_pr.Nu, _CarModule::_pr.Nu);
  _CarModule::C << MatrixXd::Identity(_CarModule::_pr.Nx, _CarModule::_pr.Nx), MatrixXd::Zero(_CarModule::_pr.Nx, _CarModule::_pr.Nu);
}
