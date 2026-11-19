# Private ORB-SLAM3 dependencies. These targets intentionally do not install
# headers or leak global include directories into the rest of Argus.

# ORB-SLAM3 originally depended on DBoW2 for its bag-of-words vocabulary.
# Argus standardizes on FBoW, shared with autonomy/localization/atlas. The
# vendored DBoW2 sources remain in the tree for reference but are not built.
if(NOT TARGET fbow::fbow)
  find_package(fbow CONFIG REQUIRED)
endif()

add_library(argus_orb_fbow INTERFACE)
target_link_libraries(argus_orb_fbow INTERFACE fbow::fbow)
target_compile_features(argus_orb_fbow INTERFACE cxx_std_17)

add_library(argus_orb_g2o STATIC
  g2o/g2o/types/types_sba.cpp
  g2o/g2o/types/types_six_dof_expmap.cpp
  g2o/g2o/types/types_seven_dof_expmap.cpp
  g2o/g2o/core/hyper_graph_action.cpp
  g2o/g2o/core/hyper_graph.cpp
  g2o/g2o/core/marginal_covariance_cholesky.cpp
  g2o/g2o/core/matrix_structure.cpp
  g2o/g2o/core/batch_stats.cpp
  g2o/g2o/core/parameter.cpp
  g2o/g2o/core/cache.cpp
  g2o/g2o/core/optimizable_graph.cpp
  g2o/g2o/core/solver.cpp
  g2o/g2o/core/optimization_algorithm_factory.cpp
  g2o/g2o/core/estimate_propagator.cpp
  g2o/g2o/core/factory.cpp
  g2o/g2o/core/sparse_optimizer.cpp
  g2o/g2o/core/hyper_dijkstra.cpp
  g2o/g2o/core/parameter_container.cpp
  g2o/g2o/core/optimization_algorithm.cpp
  g2o/g2o/core/optimization_algorithm_with_hessian.cpp
  g2o/g2o/core/optimization_algorithm_levenberg.cpp
  g2o/g2o/core/optimization_algorithm_gauss_newton.cpp
  g2o/g2o/core/jacobian_workspace.cpp
  g2o/g2o/core/robust_kernel.cpp
  g2o/g2o/core/robust_kernel_factory.cpp
  g2o/g2o/core/robust_kernel_impl.cpp
  g2o/g2o/stuff/timeutil.cpp
  g2o/g2o/stuff/string_tools.cpp
  g2o/g2o/stuff/property.cpp
)
target_include_directories(argus_orb_g2o PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/g2o
)
target_compile_definitions(argus_orb_g2o PRIVATE UNIX)
target_link_libraries(argus_orb_g2o PUBLIC Eigen3::Eigen)
target_compile_features(argus_orb_g2o PUBLIC cxx_std_17)

add_library(argus_orb_sophus INTERFACE)
target_include_directories(argus_orb_sophus INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/Sophus
)
target_link_libraries(argus_orb_sophus INTERFACE Eigen3::Eigen)

add_library(argus::orb_fbow ALIAS argus_orb_fbow)
add_library(argus::orb_g2o ALIAS argus_orb_g2o)
add_library(argus::orb_sophus ALIAS argus_orb_sophus)
