#include "oni_nav_controller/nmpc_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>

extern "C" {
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_oni_nmpc.h"
}

namespace oni_nav_controller
{

struct NmpcSolver::AcadosCapsule
{
  oni_nmpc_solver_capsule * capsule{nullptr};
  ocp_nlp_config * nlp_config{nullptr};
  ocp_nlp_dims * nlp_dims{nullptr};
  ocp_nlp_in * nlp_in{nullptr};
  ocp_nlp_out * nlp_out{nullptr};
  ocp_nlp_solver * nlp_solver{nullptr};
  void * nlp_opts{nullptr};
};

NmpcSolver::NmpcSolver()
: obstacle_model_(params_)
{
}

NmpcSolver::~NmpcSolver()
{
  if (acados_ && acados_->capsule) {
    oni_nmpc_acados_free(acados_->capsule);
    oni_nmpc_acados_free_capsule(acados_->capsule);
  }
}

bool NmpcSolver::initialize(const NmpcParams & params)
{
  params_ = params;
  obstacle_model_ = ObstacleModel(params_);

  acados_ = std::make_unique<AcadosCapsule>();
  acados_->capsule = oni_nmpc_acados_create_capsule();
  if (!acados_->capsule) {
    return false;
  }

  const int status = oni_nmpc_acados_create_with_discretization(
    acados_->capsule, ONI_NMPC_N, nullptr);
  if (status != 0) {
    return false;
  }

  acados_->nlp_config = oni_nmpc_acados_get_nlp_config(acados_->capsule);
  acados_->nlp_dims = oni_nmpc_acados_get_nlp_dims(acados_->capsule);
  acados_->nlp_in = oni_nmpc_acados_get_nlp_in(acados_->capsule);
  acados_->nlp_out = oni_nmpc_acados_get_nlp_out(acados_->capsule);
  acados_->nlp_solver = oni_nmpc_acados_get_nlp_solver(acados_->capsule);
  acados_->nlp_opts = oni_nmpc_acados_get_nlp_opts(acados_->capsule);

  ready_ = true;
  return true;
}

void NmpcSolver::set_target(const Target2D & target)
{
  target_ = target;
}

void NmpcSolver::set_obstacles(const std::vector<Eigen::Vector3d> & lidar_points_base)
{
  obstacles_ = obstacle_model_.build_obstacle_set(lidar_points_base, target_);
}

void NmpcSolver::set_initial_state(const StateVector & x0)
{
  x0_ = x0;
}

bool NmpcSolver::solve()
{
  if (!ready_ || !acados_ || !target_.valid) {
    return false;
  }

  set_state_constraints(x0_);
  warm_start_trajectory(x0_);

  const int N = ONI_NMPC_N;
  double x_guess[ONI_NMPC_NX];
  for (int stage = 0; stage <= N; ++stage) {
    if (stage == 0) {
      for (int i = 0; i < ONI_NMPC_NX; ++i) {
        x_guess[i] = x0_(i);
      }
    } else {
      ocp_nlp_out_get(acados_->nlp_config, acados_->nlp_dims, acados_->nlp_out, stage, "x", x_guess);
    }

    const double d_min = obstacle_model_.barrier_weighted_distance(
      obstacle_model_.min_distance(x_guess[0], x_guess[1], obstacles_));
    update_stage_parameters(stage, x_guess[0], x_guess[1], d_min);
  }

  int rti_phase = 0;
  ocp_nlp_solver_opts_set(acados_->nlp_config, acados_->nlp_opts, "rti_phase", &rti_phase);
  int status = oni_nmpc_acados_solve(acados_->capsule);
  rti_phase = 1;
  ocp_nlp_solver_opts_set(acados_->nlp_config, acados_->nlp_opts, "rti_phase", &rti_phase);
  status = oni_nmpc_acados_solve(acados_->capsule);

  if (status != 0) {
    return false;
  }

  double x1[ONI_NMPC_NX];
  double u0[ONI_NMPC_NU];
  ocp_nlp_out_get(acados_->nlp_config, acados_->nlp_dims, acados_->nlp_out, 1, "x", x1);
  ocp_nlp_out_get(acados_->nlp_config, acados_->nlp_dims, acados_->nlp_out, 0, "u", u0);

  velocity_out_.vx = x1[3];
  velocity_out_.vy = x1[4];
  velocity_out_.omega = x1[5];

  u_prev_[0] = u0[0];
  u_prev_[1] = u0[1];
  u_prev_[2] = u0[2];

  ocp_nlp_get(acados_->nlp_config, acados_->nlp_solver, "cost_value", &last_cost_);
  return true;
}

Velocity3 NmpcSolver::get_first_velocity() const
{
  return velocity_out_;
}

double NmpcSolver::last_cost() const
{
  return last_cost_;
}

bool NmpcSolver::is_ready() const
{
  return ready_;
}

void NmpcSolver::update_stage_parameters(int stage, double px, double py, double d_min)
{
  if (!acados_) {
    return;
  }

  double p[ONI_NMPC_NP] = {
    target_.x,
    target_.y,
    u_prev_[0],
    u_prev_[1],
    u_prev_[2],
    d_min
  };
  (void)px;
  (void)py;
  oni_nmpc_acados_update_params(acados_->capsule, stage, p, ONI_NMPC_NP);
}

void NmpcSolver::set_state_constraints(const StateVector & x0)
{
  double lbx0[ONI_NMPC_NBX0];
  double ubx0[ONI_NMPC_NBX0];
  for (int i = 0; i < ONI_NMPC_NBX0; ++i) {
    lbx0[i] = x0(i);
    ubx0[i] = x0(i);
  }

  ocp_nlp_constraints_model_set(
    acados_->nlp_config, acados_->nlp_dims, acados_->nlp_in, 0, "lbx", lbx0);
  ocp_nlp_constraints_model_set(
    acados_->nlp_config, acados_->nlp_dims, acados_->nlp_in, 0, "ubx", ubx0);
}

void NmpcSolver::warm_start_trajectory(const StateVector & x0)
{
  double x_init[ONI_NMPC_NX];
  double u_init[ONI_NMPC_NU] = {0.0, 0.0, 0.0};
  for (int i = 0; i < ONI_NMPC_NX; ++i) {
    x_init[i] = x0(i);
  }

  for (int stage = 0; stage < ONI_NMPC_N; ++stage) {
    ocp_nlp_out_set(acados_->nlp_config, acados_->nlp_dims, acados_->nlp_out, stage, "x", x_init);
    ocp_nlp_out_set(acados_->nlp_config, acados_->nlp_dims, acados_->nlp_out, stage, "u", u_init);
  }
  ocp_nlp_out_set(acados_->nlp_config, acados_->nlp_dims, acados_->nlp_out, ONI_NMPC_N, "x", x_init);
}

}  // namespace oni_nav_controller
