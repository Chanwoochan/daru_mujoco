#include "daru_mujoco/daru_v4_rbdl.hpp"

namespace RigidBodyDynamics
{
  using namespace RigidBodyDynamics::Math;

  Eigen::MatrixXd DARU_RBDL::PseudoInverse(const Eigen::MatrixXd& mat)
  {
    int m = mat.rows();
    int n = mat.cols();
    // lambda^2 항을 미리 만듭니다
    double lambda2 = 0.01 * 0.01;
  
    if (m >= n)
    {
      // 정방향 또는 m > n: (J^T J + λ^2 I)^{-1} J^T
      Eigen::MatrixXd A = mat.transpose() * mat
                        + lambda2 * Eigen::MatrixXd::Identity(n, n);
      // LDLT 분해로 빠르고 안정적으로 풉니다
      return A.ldlt().solve(mat.transpose());
    }
    else
    {
      // m < n: J^T (J J^T + λ^2 I)^{-1}
      Eigen::MatrixXd B = mat * mat.transpose()
                        + lambda2 * Eigen::MatrixXd::Identity(m, m);
      return mat.transpose() * B.ldlt().solve(
               Eigen::MatrixXd::Identity(m, m));
    }
  }

  void DARU_RBDL::updateQ(const JOINT_* JOINT)
  {
    for(unsigned int i{};i<dof;++i)
    {
      q    (i) = JOINT[i].A_pos;
      qdot (i) = JOINT[i].A_vel;
      qddot(i) = JOINT[i].A_acc;
      tau  (i) = JOINT[i].A_tau;
    }
  }

  void DARU_RBDL::calCG(VectorNd& CG)
  {
    model.gravity = Vector3d(0.0, 0.0, -9.81);
    InverseDynamics(model, q, qdot, VectorNd::Zero(dof), CG);
  }

  void DARU_RBDL::calG(VectorNd& G)
  {
    model.gravity = Vector3d(0.0, 0.0, -9.81);
    InverseDynamics(model, q, VectorNd::Zero(dof), VectorNd::Zero(dof), G);
  }

  void DARU_RBDL::calM(MatrixNd& M)
  {
    CompositeRigidBodyAlgorithm(model, q, M, false);
  }

  void DARU_RBDL::calJ()
  {
    Vector3d point_position(0.0, 0.0, -0.0825); // 링크의 끝단 좌표
    Eigen::MatrixXd J_l_rbdl = Eigen::MatrixXd::Zero(6, 15);
    Eigen::MatrixXd J_r_rbdl = Eigen::MatrixXd::Zero(6, 15);
    Eigen::MatrixXd J_l(6,8);
    Eigen::MatrixXd J_r(6,8);
    Eigen::MatrixXd J(12,15);
    Eigen::MatrixXd J_full(13,15);

    RigidBodyDynamics::CalcPointJacobian6D(model, q, model.GetBodyId("LWP_Link"), point_position, J_l_rbdl, false);
    J_l << J_l_rbdl.block(0, 0, 6, 8);
    RigidBodyDynamics::CalcPointJacobian6D(model, q, model.GetBodyId("RWP_Link"), point_position, J_r_rbdl, false);
    J_r << J_r_rbdl.block(0, 0, 6, 1), J_r_rbdl.block(0, 8, 6, 7);
    
    // std::cout << "=[J_l]==============================" << std::endl;
    // std::cout << J_l << std::endl;
    // std::cout << "====================================" << std::endl;
    // std::cout << "=[J_r]==============================" << std::endl;
    // std::cout << J_r << std::endl;
    // std::cout << "====================================" << std::endl;

    J.setZero();

    // 2) 허리(q_w) 공유 칼럼 (MATLAB의 J(1:6,1), J(7:12,1))
    J.block(0, 0, 6, 1) = J_l.col(0);
    J.block(6, 0, 6, 1) = J_r.col(0);

    // 3) 왼팔 (MATLAB의 J(1:6,2:8) = J_L(:,2:8))
    J.block(0, 1, 6, 7) = J_l.block(0, 1, 6, 7);

    // 4) 오른팔 (MATLAB의 J(7:12,9:15) = J_R(:,2:8))
    J.block(6, 8, 6, 7) = J_r.block(0, 1, 6, 7);

    // 5) 최종 13×15 자코비안: 맨 위에 [1 0 … 0] 행을 붙임 (MATLAB의 [eye(1,15); J])
    
    J_full.setZero();
    J_full(0,0) = 1;                     // q1 = q_w
    J_full.block(1, 0, 12, 15) = J;     // 나머지 12×15 블록 복사
    // Jacobian = J_full;
    jacobian = J_full;
    // std::cout << "=[J]==============================" << std::endl;
    // std::cout << J_full << std::endl;
    // std::cout << "====================================" << std::endl;
  }

  void DARU_RBDL::calJq(VectorNd q_)
  {
    Vector3d point_position(0.0, 0.0, -0.0825); // 링크의 끝단 좌표
    Eigen::MatrixXd J_l_rbdl = Eigen::MatrixXd::Zero(6, 15);
    Eigen::MatrixXd J_r_rbdl = Eigen::MatrixXd::Zero(6, 15);
    Eigen::MatrixXd J_l(6,8);
    Eigen::MatrixXd J_r(6,8);
    Eigen::MatrixXd J(12,15);
    Eigen::MatrixXd J_full(13,15);

    RigidBodyDynamics::CalcPointJacobian6D(model, q_, model.GetBodyId("LWP_Link"), point_position, J_l_rbdl, false);
    J_l << J_l_rbdl.block(0, 0, 6, 8);
    RigidBodyDynamics::CalcPointJacobian6D(model, q_, model.GetBodyId("RWP_Link"), point_position, J_r_rbdl, false);
    J_r << J_r_rbdl.block(0, 0, 6, 1), J_r_rbdl.block(0, 8, 6, 7);
    
    // std::cout << "=[J_l]==============================" << std::endl;
    // std::cout << J_l << std::endl;
    // std::cout << "====================================" << std::endl;
    // std::cout << "=[J_r]==============================" << std::endl;
    // std::cout << J_r << std::endl;
    // std::cout << "====================================" << std::endl;

    J.setZero();

    // 2) 허리(q_w) 공유 칼럼 (MATLAB의 J(1:6,1), J(7:12,1))
    J.block(0, 0, 6, 1) = J_l.col(0);
    J.block(6, 0, 6, 1) = J_r.col(0);

    // 3) 왼팔 (MATLAB의 J(1:6,2:8) = J_L(:,2:8))
    J.block(0, 1, 6, 7) = J_l.block(0, 1, 6, 7);

    // 4) 오른팔 (MATLAB의 J(7:12,9:15) = J_R(:,2:8))
    J.block(6, 8, 6, 7) = J_r.block(0, 1, 6, 7);

    // 5) 최종 13×15 자코비안: 맨 위에 [1 0 … 0] 행을 붙임 (MATLAB의 [eye(1,15); J])
    
    J_full.setZero();
    J_full(0,0) = 1;                     // q1 = q_w
    J_full.block(1, 0, 12, 15) = J;     // 나머지 12×15 블록 복사
    // Jacobian = J_full;
    jacobian = J_full;
    // std::cout << "=[J]==============================" << std::endl;
    // std::cout << J_full << std::endl;
    // std::cout << "====================================" << std::endl;
  }

  void DARU_RBDL::calJqWaist(VectorNd q_)
  {
    Vector3d point_position(0.0, 0.0, -0.0825); // 링크의 끝단 좌표
    Eigen::MatrixXd J_l_rbdl = Eigen::MatrixXd::Zero(6, 15);
    Eigen::MatrixXd J_r_rbdl = Eigen::MatrixXd::Zero(6, 15);
    Eigen::MatrixXd J_l(6,8);
    Eigen::MatrixXd J_r(6,8);
    Eigen::MatrixXd J(12,15);
    
    RigidBodyDynamics::CalcPointJacobian6D(model, q_, model.GetBodyId("LWP_Link"), point_position, J_l_rbdl, false);
    J_l << J_l_rbdl.block(0, 0, 6, 8);
    RigidBodyDynamics::CalcPointJacobian6D(model, q_, model.GetBodyId("RWP_Link"), point_position, J_r_rbdl, false);
    J_r << J_r_rbdl.block(0, 0, 6, 1), J_r_rbdl.block(0, 8, 6, 7);

    J.setZero();

    // 2) 허리(q_w) 공유 칼럼 (MATLAB의 J(1:6,1), J(7:12,1))
    J.block(0, 0, 6, 1) = J_l.col(0);
    J.block(6, 0, 6, 1) = J_r.col(0);

    // 3) 왼팔 (MATLAB의 J(1:6,2:8) = J_L(:,2:8))
    J.block(0, 1, 6, 7) = J_l.block(0, 1, 6, 7);

    // 4) 오른팔 (MATLAB의 J(7:12,9:15) = J_R(:,2:8))
    J.block(6, 8, 6, 7) = J_r.block(0, 1, 6, 7);

    jacobian = J;
    // std::cout << "=[J]==============================" << std::endl;
    // std::cout << J_full << std::endl;
    // std::cout << "====================================" << std::endl;
  }

  void DARU_RBDL::calJqL(VectorNd q_)
  {
    Vector3d point_position(0.0, 0.0, -0.0825); // 링크의 끝단 좌표
    Eigen::MatrixXd J_l_rbdl = Eigen::MatrixXd::Zero(6, 15);
    Eigen::MatrixXd J_l(6,8);
    
    RigidBodyDynamics::CalcPointJacobian6D(model, q_, model.GetBodyId("LWP_Link"), point_position, J_l_rbdl, false);
    J_l << J_l_rbdl.block(0, 0, 6, 8);

    jacobian = J_l;
    // std::cout << "=[J]==============================" << std::endl;
    // std::cout << J_full << std::endl;
    // std::cout << "====================================" << std::endl;
  }
  void DARU_RBDL::calJqLPos(VectorNd q_)
  {
    Vector3d point_position(0.0, 0.0, -0.0825); // 링크의 끝단 좌표
    Eigen::MatrixXd J_l_rbdl = Eigen::MatrixXd::Zero(6, 15);
    Eigen::MatrixXd J_l(3,8);
    
    RigidBodyDynamics::CalcPointJacobian6D(model, q_, model.GetBodyId("LWP_Link"), point_position, J_l_rbdl, false);
    J_l << J_l_rbdl.block(3, 0, 3, 8);

    jacobian = J_l;
  }

  void DARU_RBDL::calJqWaistPos(VectorNd q_)
  {
    Vector3d point_position(0.0, 0.0, -0.0825); // 링크의 끝단 좌표
    Eigen::MatrixXd J_l_rbdl = Eigen::MatrixXd::Zero(6, 15);
    Eigen::MatrixXd J_r_rbdl = Eigen::MatrixXd::Zero(6, 15);
    Eigen::MatrixXd J_l(3,8);
    Eigen::MatrixXd J_r(3,8);
    Eigen::MatrixXd J(6,15);
    
    RigidBodyDynamics::CalcPointJacobian6D(model, q_, model.GetBodyId("LWP_Link"), point_position, J_l_rbdl, false);
    J_l << J_l_rbdl.block(3, 0, 3, 8);
    RigidBodyDynamics::CalcPointJacobian6D(model, q_, model.GetBodyId("RWP_Link"), point_position, J_r_rbdl, false);
    J_r << J_r_rbdl.block(3, 0, 3, 1), J_r_rbdl.block(3, 8, 3, 7);

    J.setZero();

    J.block(0, 0, 3, 1) = J_l.col(0);
    J.block(3, 0, 3, 1) = J_r.col(0);

    J.block(0, 1, 3, 7) = J_l.block(0, 1, 3, 7);

    // 4) 오른팔 (MATLAB의 J(7:12,9:15) = J_R(:,2:8))
    J.block(3, 8, 3, 7) = J_r.block(0, 1, 3, 7);

    jacobian = J;
  }

  void DARU_RBDL::calJqWaistRot(VectorNd q_)
  {
    Vector3d point_position(0.0, 0.0, -0.0825); // 링크의 끝단 좌표
    Eigen::MatrixXd J_l_rbdl = Eigen::MatrixXd::Zero(6, 15);
    Eigen::MatrixXd J_r_rbdl = Eigen::MatrixXd::Zero(6, 15);
    Eigen::MatrixXd J_l(3,8);
    Eigen::MatrixXd J_r(3,8);
    Eigen::MatrixXd J(6,15);
    
    RigidBodyDynamics::CalcPointJacobian6D(model, q_, model.GetBodyId("LWP_Link"), point_position, J_l_rbdl, false);
    J_l << J_l_rbdl.block(0, 0, 3, 8);
    RigidBodyDynamics::CalcPointJacobian6D(model, q_, model.GetBodyId("RWP_Link"), point_position, J_r_rbdl, false);
    J_r << J_r_rbdl.block(0, 0, 3, 1), J_r_rbdl.block(3, 8, 3, 7);

    J.setZero();

    J.block(0, 0, 3, 1) = J_l.col(0);
    J.block(3, 0, 3, 1) = J_r.col(0);

    J.block(0, 1, 3, 7) = J_l.block(0, 1, 3, 7);

    // 4) 오른팔 (MATLAB의 J(7:12,9:15) = J_R(:,2:8))
    J.block(3, 8, 3, 7) = J_r.block(0, 1, 3, 7);

    jacobian = J;
  }

  void DARU_RBDL::CalStaticToque(const VectorNd& F,VectorNd& T)
  {
    T = jacobian.transpose()*F;
  }

  void DARU_RBDL::calEEPos(Vector3d& L_POS, Matrix3d& L_ROT, Vector3d& R_POS, Matrix3d& R_ROT)
  {
    Vector3d point_position(0, 0, -0.0825); // 링크의 끝단 좌표
     L_POS = 
    RigidBodyDynamics::CalcBodyToBaseCoordinates(model, q, model.GetBodyId("LWP_Link"), 
                                                   point_position, true);
     R_POS = 
    RigidBodyDynamics::CalcBodyToBaseCoordinates(model, q, model.GetBodyId("RWP_Link"), 
                                                   point_position, true);
    
     L_ROT = 
    RigidBodyDynamics::CalcBodyWorldOrientation(model, q, model.GetBodyId("LWP_Link"), true);

     R_ROT = 
    RigidBodyDynamics::CalcBodyWorldOrientation(model, q, model.GetBodyId("RWP_Link"), true);
  }

  void DARU_RBDL::calEEPosq(Vector3d& L_POS, Matrix3d& L_ROT, Vector3d& R_POS, Matrix3d& R_ROT, VectorNd q_)
  {
    Vector3d point_position(0, 0, -0.0825); // 링크의 끝단 좌표
     L_POS = 
    RigidBodyDynamics::CalcBodyToBaseCoordinates(model, q_, model.GetBodyId("LWP_Link"), 
                                                   point_position, true);
     R_POS = 
    RigidBodyDynamics::CalcBodyToBaseCoordinates(model, q_, model.GetBodyId("RWP_Link"), 
                                                   point_position, true);
    
     L_ROT = 
    RigidBodyDynamics::CalcBodyWorldOrientation(model, q_, model.GetBodyId("LWP_Link"), true);

     R_ROT = 
    RigidBodyDynamics::CalcBodyWorldOrientation(model, q_, model.GetBodyId("RWP_Link"), true);
  }

  void DARU_RBDL::calIK(VectorNd& X, VectorNd& Q)
  {
    Q = PseudoInverse(jacobian) * X;
  }

  void DARU_RBDL::calNullSpace(MatrixNd& N)
  {
    N = MatrixNd::Identity(15,15) - PseudoInverse(jacobian)*jacobian;
  }

  void DARU_RBDL::Log(Matrix3d& R, Vector3d& phi)
  {
    double cos_theta = (R.trace() - 1.0) / 2.0;
    cos_theta = std::min(1.0, std::max(-1.0, cos_theta));  // 숫치 안정성
    double theta = std::acos(cos_theta);

    if (theta < 1e-8) {
        phi.setZero();
    } else {
      RigidBodyDynamics::Math::Matrix3d logR = (R - R.transpose()) * (theta / (2.0 * std::sin(theta)));
        // vee 연산: skew 대칭 행렬 → 벡터
        phi << logR(2,1), logR(0,2), logR(1,0);
    }
  }

  void DARU_RBDL::RotVtoRotM(Matrix3d& R, Vector3d& phi)
  {
    double angle = phi.norm();
    // 작은 각도에 대해서는 Identity 반환
    if (angle < 1e-12) {
      R = Matrix3d::Identity();
    }
    // 회전축 단위 벡터
    Vector3d axis = phi / angle;

    // Skew-symmetric matrix K for axis
    Matrix3d K;
    K <<      0,    -axis[2],  axis[1],
          axis[2],       0,   -axis[0],
         -axis[1],  axis[0],       0;

    // Rodrigues 공식
    R = Matrix3d::Identity()
               + std::sin(angle) * K
               + (1.0 - std::cos(angle)) * (K * K);
  }


}
