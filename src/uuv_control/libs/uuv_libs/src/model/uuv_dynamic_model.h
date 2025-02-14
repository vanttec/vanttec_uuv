#ifndef UUV_ROS2_DYNAMIC_MODEL_H
#define UUV_ROS2_DYNAMIC_MODEL_H

#include <Eigen/Dense>
#include "uuv_datatypes.h"

using namespace uuv;

class UUVDynamicModel {
public:
  UUVDynamicModel();
  UUVDynamicModel(Eigen::VectorXf);
  void update(std::array<double, 6> thrusters);
  void matricesUpdate();
  static double constrainAngle(double angle);

  /* Non-linear functions */
  Eigen::VectorXf f_x_;
  Eigen::MatrixXf g_x_;

  UUVState state{};

private:
  /* Transformation matrix */

  Eigen::MatrixXf J_;
  Eigen::Matrix3f R_;
  Eigen::Matrix3f T_;

  /* System matrices */

  Eigen::MatrixXf M_;
  Eigen::MatrixXf C_;
  Eigen::MatrixXf D_;
  Eigen::VectorXf G_;

  double dt{0.01};

  /* Physical Parameters */

  double m_{24};
  double W_{235.44};
  double volume_{0.0252};
  double B_{245};
  double Ixx_{0.9};
  double Ixy_{0};
  double Ixz_{0};
  double Iyx_{0};
  double Iyy_{1.75};
  double Iyz_{0};
  double Izx_{0};
  double Izy_{0};
  double Izz_{1.43};

  /* Added Mass Parameters */

  double X_u_dot_{16.8374};
  double Y_v_dot_{20.2748};
  double Z_w_dot_{35.318};
  double K_p_dot_{0.2165};
  double M_q_dot_{0.6869};
  double N_r_dot_{0.6157};

  /* Damping Parameters */

  double X_u_{-0.3431};
  double Y_v_{0.0518};
  double Z_w_{-0.5841};
  double K_p_{0.0064};
  double M_q_{0.04};    
  double N_r_{-0.1063};

  double X_uu_{-111.7397};
  double Y_vv_{-44.4058};
  double Z_ww_{-157.1951};
  double K_pp_{-0.4634};
  double M_qq_{-0.2902};
  double N_rr_{-2.2897};

  /* Distance from origin to center of mass */
  // They are the same
  // TODO: CALCULAR Y AGREGAR 

  /* Distance from origin to center of buoyancy  */
  // TODO: CALCULAR Y MODIFICAR

  double rb_x_{0};
  double rb_y_{0};
  double rb_z_{0};

  // TODO CALCULAR BIEN ESTO
  double MAX_FORCE_X_{100.};
  double MAX_FORCE_Y_{100.};
  double MAX_FORCE_Z_{100.};
  double MAX_TORQUE_K_{100.};
  double MAX_TORQUE_M_{100.};
  double MAX_TORQUE_N_{100.};
};


#endif //UUV_ROS2_DYNAMIC_MODEL_H