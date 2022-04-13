/*
 * @Author: LiuShi
 * @Date: 2022-04-07 21:53:57
 * @LastEditors: LiuShi
 * @LastEditTime: 2022-04-13 18:06:17
 * @FilePath: /mpc/main.cpp
 * @Description: Main file for MPC simulation.
 * @custom_string_LiuShi:
 * @custom_string_LiuShi_copyright: Copyright (c) 2022 by Niubi Sparkle, All Rights Reserved.
 */
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <cmath>
#include "CarModule.h"
#include "Paras.h"
#include "MPCPlan.h"
#include "RefWay.h"
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

int main (int argc, char *argv[])
{
  _CarModule CarModule;
  _Paras Prs;
  _RefWay Rw;
  _MPCPlan Mpc;
  double te = 0.;
  MatrixXd RealC = MatrixXd::Zero(Prs.Nu, 1);
  MatrixXd RealS = MatrixXd::Zero(Prs.Nx, 1);
  MatrixXd DeltaK = MatrixXd::Zero(Prs.Nx + Prs.Nu, 1);
  CarModule.initialStatus();

  while (te < Prs.tt) {
    MatrixXd RefS = Rw.cacu_RW(Prs.choose, te);
    RealS << CarModule.Xk, CarModule.Yk, CarModule.Phik;
    DeltaK << RealS,RealC;
    MatrixXd Kesi = DeltaK - RefS;
    CarModule.cacu_ABC(Rw.rv,Rw.rp,Rw.rd);
    Mpc.cacu_MN(CarModule.C, CarModule.A, CarModule.B);
    Eigen::SparseMatrix<double> Hessian = Mpc._matrixHessian();
    VectorXd Gradient = Mpc._vectorGradient(Kesi);
    Eigen::SparseMatrix<double> LinearMatrix = Mpc._matrixLinearMatrix();
    VectorXd LowerBound = Mpc._vectorLowerBound(Kesi);
    VectorXd UpperBound = Mpc._vectorUpperBound(Kesi);
    
    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(LinearMatrix.cols());
    solver.data()->setNumberOfConstraints(LinearMatrix.rows());
    if(!solver.data()->setHessianMatrix(Hessian)) return 1;
    if(!solver.data()->setGradient(Gradient)) return 1;
    if(!solver.data()->setLinearConstraintsMatrix(LinearMatrix)) return 1;
    if(!solver.data()->setLowerBound(LowerBound)) return 1;
    if(!solver.data()->setUpperBound(UpperBound)) return 1;
    if(!solver.initSolver()) return 1;
    VectorXd QPSolution;
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return 1;
    QPSolution = solver.getSolution();
    
    if(QPSolution(0) > Prs.Du0){
      QPSolution(0) = Prs.Du0;
    }else if(QPSolution(0) < -Prs.Du0){
      QPSolution(0) = -Prs.Du0;
    }
    if(QPSolution(1) > Prs.Du1){
      QPSolution(1) = Prs.Du1;
    }else if(QPSolution(1) < -Prs.Du1){
      QPSolution(1) = -Prs.Du1;
    }
    RealC(0, 0) = RefS(Prs.Nx, 0) + QPSolution(0) + Kesi(Prs.Nx, 0);
    RealC(1, 0) = RefS(Prs.Nx + 1, 0) + QPSolution(1) + Kesi(Prs.Nx + 1, 0);
    if(RealC(1, 0) > 39 * 3.1415926 / 180){
      RealC(1, 0) = 39 * 3.1415926 / 180;
    }else if(RealC(1, 0) < -39 * 3.1415926 / 180){
      RealC(1, 0) = -39 * 3.1415926 / 180;
    }
    //if(RealC(0, 0) > Prs.Ov){
      //RealC(0, 0) = Prs.Ov;
    //}else if(RealC(0, 0) < 0.)
    //{
      //RealC(0, 0) = 0.;
    //}
    CarModule.CarMove(RealC(0, 0), RealC(1, 0), Prs.Carl);
    cout<<"======================================================="<<endl;
    cout<<Kesi<<endl;
    cout<<"======================================================="<<endl;

    te = te + Prs.dt;
  }
  return 0;
}
