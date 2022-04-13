/*
 * @Author: LiuShi
 * @Date: 2022-04-07 21:34:36
 * @LastEditors: LiuShi
 * @LastEditTime: 2022-04-13 18:09:44
 * @FilePath: /mpc/include/CarModule.h
 * @Description: Car model header file.
 * @custom_string_LiuShi: 
 * @custom_string_LiuShi_copyright: Copyright (c) 2022 by Niubi Sparkle, All Rights Reserved. 
 */
#pragma once
#include <cmath>
#include <Eigen/Dense>
#include "Paras.h"
#include "RefWay.h"
using Eigen::MatrixXd;
class _CarModule
{
private:
  MatrixXd A_Start;
  MatrixXd B_Start;
public:
    _Paras _pr;
    float Xk;
    float Yk;
    float Phik;
    float Xkp1;
    float Ykp1;
    float Phikp1;
    void initialStatus();
    void CarMove(double v, double delta, double Carl);
    MatrixXd A;
    MatrixXd B;
    MatrixXd C;
    void cacu_ABC(double rv, double rp, double rd);   
};
