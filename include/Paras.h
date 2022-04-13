/*
 * @Author: LiuShi
 * @Date: 2022-04-08 14:07:28
 * @LastEditors: LiuShi
 * @LastEditTime: 2022-04-13 17:05:26
 * @FilePath: /mpc/include/Paras.h
 * @Description: 
 * @custom_string_LiuShi: WeChat:18857488400; QQ:487196835; Email:liushibb@sina.com
 * @custom_string_LiuShi_copyright: Copyright (c) 2022 by Niubi Sparkle, All Rights Reserved. 
 */
#pragma once
#include "yaml-cpp/yaml.h"
#include <string>
using namespace std;
class _Paras {
  public:
  YAML::Node _Prs = YAML::LoadFile("/home/ls/Project/CPlus/mpc/doc/Para.yaml");
  int Np = _Prs["Np"].as<int>();
  int Nc = _Prs["Nc"].as<int>();
  int Nx = _Prs["Nx"].as<int>();
  int Nu = _Prs["Nu"].as<int>();
  int tt = _Prs["tt"].as<int>();
  double Carl = _Prs["Carl"].as<double>();      
  double dt = _Prs["dt"].as<double>();
  double Ox = _Prs["Ox"].as<double>();
  double Oy = _Prs["Oy"].as<double>();
  double Or = _Prs["Or"].as<double>();
  double Ov = _Prs["Ov"].as<double>();
  double Row = _Prs["Row"].as<double>();
  double Q = _Prs["Q"].as<double>();
  double R = _Prs["R"].as<double>();
  double U0 = _Prs["U0"].as<double>();
  double U1 = _Prs["U1"].as<double>();
  double Du0 = _Prs["Du0"].as<double>();
  double Du1 = _Prs["Du1"].as<double>();
  double ypslmin = _Prs["ypslmin"].as<double>();
  double yp = _Prs["yp"].as<double>();
  double ypslmax = _Prs["ypslmax"].as<double>();
  string choose = _Prs["Choose"].as<string>();
};
