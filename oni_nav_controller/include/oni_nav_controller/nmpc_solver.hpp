#ifndef ONI_NAV_CONTROLLER__NMPC_SOLVER_HPP_
#define ONI_NAV_CONTROLLER__NMPC_SOLVER_HPP_

#include <array>
#include <memory>
#include <vector>

#include "oni_nav_controller/obstacle_model.hpp"
#include "oni_nav_controller/types.hpp"

namespace oni_nav_controller
{

class NmpcSolver
{
public:
  NmpcSolver();
  ~NmpcSolver();

  NmpcSolver(const NmpcSolver &) = delete;
  NmpcSolver & operator=(const NmpcSolver &) = delete;

  bool initialize(const NmpcParams & params);
  void set_target(const Target2D & target);
  void set_obstacles(const std::vector<Eigen::Vector3d> & lidar_points_base);
  void set_initial_state(const StateVector & x0);
  bool solve();
  Velocity3 get_first_velocity() const;
  double last_cost() const;
  bool is_ready() const;

private:
  void update_stage_parameters(int stage, double px, double py, double d_min);
  void set_state_constraints(const StateVector & x0);
  void warm_start_trajectory(const StateVector & x0);

  NmpcParams params_;
  ObstacleModel obstacle_model_;
  Target2D target_;
  std::vector<Eigen::Vector2d> obstacles_;
  StateVector x0_{StateVector::Zero()};
  Velocity3 velocity_out_;
  std::array<double, 3> u_prev_{ {0.0, 0.0, 0.0} };
  double last_cost_{0.0};
  bool ready_{false};

  struct AcadosCapsule;
  std::unique_ptr<AcadosCapsule> acados_;
};

}  // namespace oni_nav_controller

#endif  // ONI_NAV_CONTROLLER__NMPC_SOLVER_HPP_
