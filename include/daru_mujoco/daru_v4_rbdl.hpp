#ifndef DARU_V4_RBDL_
#define DARU_V4_RBDL_

#include <iostream>
#include <functional>
#include <Eigen/Dense>
#include <complex>
#include <cmath>
#include <vector>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

struct JOINT_
{
  double A_acc{}; double A_tau{};
  double A_vel{}; double A_vel_old{};
  double A_pos{}; double A_pos_old{};

  double A_vel_lpf{};

  double R_vel{};
  double R_pos{}; double R_pos_old{};
  double R_tau{};
};

struct JOY
{
  bool L1;
  double L2{};
  bool R1{};
  double R2{};
  double X1{};
  double Y1{};
  double X2{};
  double Y2{};
  bool Left{};
  bool Right{};
  bool Up{};
  bool Down{};
  bool Rec{};
  bool Cir{};
  bool Tri{};
  bool Cro{};
  bool PS{};
  bool flag[255]{};
};

enum
{
    WAI = 0, S_P, S_R, S_Y, E_P, E_Y, W_R, W_P
};

namespace RigidBodyDynamics
{
    using namespace RigidBodyDynamics::Math;
    class DARU_RBDL
    {
    public:
        Model model;
        DARU_RBDL(const char* urdf_path)
        {
            Addons::URDFReadFromFile(urdf_path, &model, false, false);

            dof = model.q_size;
            q = VectorNd::Zero(dof);
            qdot = VectorNd::Zero(dof);
            qddot = VectorNd::Zero(dof);
            tau = VectorNd::Zero(dof);
            CG = VectorNd::Zero(dof);

            qdot_h = VectorNd::Zero(dof);
            q_h = VectorNd::Zero(dof);

            jacobian = Eigen::MatrixXd::Zero(6, dof);
        }
        Eigen::MatrixXd PseudoInverse(const Eigen::MatrixXd& mat);
        void updateQ(const JOINT_* JOINT);
        void calCG(VectorNd& CG);
        void calG(VectorNd& G);
        void calM(MatrixNd& M);
        void calJ();
        void calJq(VectorNd q_);
        void calJqWaist(VectorNd q_);
        void calJqWaistPos(VectorNd q_);
        void calJqWaistRot(VectorNd q_);
        void calJqL(VectorNd q_);
        void calJqLPos(VectorNd q_);
        unsigned int Dof() {return dof;}
        void calEEPos(Vector3d& L_POS, Matrix3d& L_ROT, Vector3d& R_POS, Matrix3d& R_ROT);
        void calEEPosq(Vector3d& L_POS, Matrix3d& L_ROT, Vector3d& R_POS, Matrix3d& R_ROT, VectorNd q);
        void calNullSpace(MatrixNd& N);
        void calIK(VectorNd& X, VectorNd& Q);
        void Log(Matrix3d& R, Vector3d& phi);
        void RotVtoRotM(Matrix3d& R, Vector3d& phi);
        void CalStaticToque(const VectorNd& F,VectorNd& T);
    private:
        unsigned int dof;
        VectorNd q;
        VectorNd qdot;
        VectorNd qddot;
        VectorNd tau;
        VectorNd CG;
        VectorNd qdot_h;
        VectorNd q_h;
    public:
        Eigen::MatrixXd jacobian;
    };
}
#endif
