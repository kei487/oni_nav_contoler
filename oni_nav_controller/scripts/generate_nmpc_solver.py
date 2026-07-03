#!/usr/bin/env python3
"""Generate Acados NMPC solver C code for oni_nav_controller."""

import os
import sys

import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
from casadi import SX, cos, sin, vertcat, fmax

# Default weights (overridden at runtime via parameter updates where applicable)
Q_XY = 10.0
R_VAL = 0.1
S_VAL = 1.0
D_SAFE = 0.4
W_BARRIER = 50.0

V_MAX = 1.0
OMEGA_MAX = 2.0
A_MAX = 2.0
ALPHA_MAX = 4.0

N_HORIZON = 20
DT = 0.05


def export_oni_nmpc_model() -> AcadosModel:
    model = AcadosModel()
    model.name = "oni_nmpc"

    x = SX.sym("x", 6)  # px, py, theta, vx, vy, omega
    u = SX.sym("u", 3)  # ax, ay, alpha
    p = SX.sym("p", 6)  # tx, ty, u_prev(3), d_min

    theta = x[2]
    vx = x[3]
    vy = x[4]
    omega = x[5]

    x_dot = vertcat(
        vx * cos(theta) - vy * sin(theta),
        vx * sin(theta) + vy * cos(theta),
        omega,
        u[0],
        u[1],
        u[2],
    )

    target_x = p[0]
    target_y = p[1]
    u_prev = p[2:5]
    d_min = p[5]

    tracking = Q_XY * ((x[0] - target_x) ** 2 + (x[1] - target_y) ** 2)
    control_cost = R_VAL * (u[0] ** 2 + u[1] ** 2 + u[2] ** 2)
    smooth_cost = S_VAL * (
        (u[0] - u_prev[0]) ** 2 + (u[1] - u_prev[1]) ** 2 + (u[2] - u_prev[2]) ** 2
    )
    barrier = W_BARRIER * (fmax(0, D_SAFE - d_min) ** 2) / (d_min * d_min + 0.01)

    model.f_expl_expr = x_dot
    model.x = x
    model.u = u
    model.p = p
    model.cost_expr_ext_cost = tracking + control_cost + smooth_cost + barrier
    model.cost_expr_ext_cost_e = Q_XY * ((x[0] - target_x) ** 2 + (x[1] - target_y) ** 2)

    return model


def create_ocp() -> AcadosOcp:
    ocp = AcadosOcp()
    model = export_oni_nmpc_model()
    ocp.model = model

    ocp.dims.N = N_HORIZON
    ocp.solver_options.tf = N_HORIZON * DT

    ocp.cost.cost_type = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"

    nx = 6
    nu = 3
    np_param = 6

    ocp.dims.np = np_param
    ocp.parameter_values = np.zeros(np_param)

    # Control bounds
    ocp.constraints.lbu = np.array([-A_MAX, -A_MAX, -ALPHA_MAX])
    ocp.constraints.ubu = np.array([A_MAX, A_MAX, ALPHA_MAX])
    ocp.constraints.idxbu = np.arange(nu)

    # Velocity state bounds (vx, vy, omega)
    ocp.constraints.idxbx = np.array([3, 4, 5])
    ocp.constraints.lbx = np.array([-V_MAX, -V_MAX, -OMEGA_MAX])
    ocp.constraints.ubx = np.array([V_MAX, V_MAX, OMEGA_MAX])

    ocp.constraints.idxbx_e = np.array([3, 4, 5])
    ocp.constraints.lbx_e = np.array([-V_MAX, -V_MAX, -OMEGA_MAX])
    ocp.constraints.ubx_e = np.array([V_MAX, V_MAX, OMEGA_MAX])

    ocp.constraints.x0 = np.zeros(nx)

    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "EXACT"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.nlp_solver_max_iter = 10
    ocp.solver_options.print_level = 0

    return ocp


def main() -> int:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(script_dir)
    output_dir = os.path.join(pkg_dir, "acados_generated")
    os.makedirs(output_dir, exist_ok=True)

    ocp = create_ocp()
    json_path = os.path.join(output_dir, "oni_nmpc.json")

    AcadosOcpSolver.generate(ocp, json_file=json_path, simulink_opts=None, cmake_builder=None)
    print(f"Generated NMPC solver in: {output_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
