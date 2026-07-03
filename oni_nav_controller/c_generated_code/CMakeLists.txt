cmake_minimum_required(VERSION 3.8)

set(ONI_NMPC_SOURCES
  acados_solver_oni_nmpc.c
  oni_nmpc_model/oni_nmpc_expl_ode_fun.c
  oni_nmpc_model/oni_nmpc_expl_vde_forw.c
  oni_nmpc_model/oni_nmpc_expl_vde_adj.c
  oni_nmpc_model/oni_nmpc_expl_ode_hess.c
  oni_nmpc_cost/oni_nmpc_cost_ext_cost_0_fun.c
  oni_nmpc_cost/oni_nmpc_cost_ext_cost_0_fun_jac.c
  oni_nmpc_cost/oni_nmpc_cost_ext_cost_0_fun_jac_hess.c
  oni_nmpc_cost/oni_nmpc_cost_ext_cost_fun.c
  oni_nmpc_cost/oni_nmpc_cost_ext_cost_fun_jac.c
  oni_nmpc_cost/oni_nmpc_cost_ext_cost_fun_jac_hess.c
  oni_nmpc_cost/oni_nmpc_cost_ext_cost_e_fun.c
  oni_nmpc_cost/oni_nmpc_cost_ext_cost_e_fun_jac.c
  oni_nmpc_cost/oni_nmpc_cost_ext_cost_e_fun_jac_hess.c
)

add_library(oni_nmpc_acados STATIC ${ONI_NMPC_SOURCES})

target_include_directories(oni_nmpc_acados PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${ACADOS_INCLUDE_DIR}
  ${ACADOS_INCLUDE_DIR}/acados
  ${ACADOS_INCLUDE_DIR}/blasfeo/include
  ${ACADOS_INCLUDE_DIR}/hpipm/include
)

target_compile_options(oni_nmpc_acados PRIVATE -fPIC -std=c99)

target_link_libraries(oni_nmpc_acados PUBLIC
  ${ACADOS_LIB_DIR}/libacados.so
  ${ACADOS_LIB_DIR}/libhpipm.so
  ${ACADOS_LIB_DIR}/libblasfeo.so
  m
)
