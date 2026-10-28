# Private ORB-SLAM3 dependencies. These targets intentionally do not install
# headers or leak global include directories into the rest of Atlas.

add_library(atlas_orb_dbow2 STATIC
  DBoW2/DBoW2/BowVector.cpp
  DBoW2/DBoW2/FORB.cpp
  DBoW2/DBoW2/FeatureVector.cpp
  DBoW2/DBoW2/ScoringObject.cpp
  DBoW2/DUtils/Random.cpp
  DBoW2/DUtils/Timestamp.cpp
)
target_include_directories(atlas_orb_dbow2 PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/DBoW2
)
target_link_libraries(atlas_orb_dbow2 PUBLIC ${OpenCV_LIBS})
target_compile_features(atlas_orb_dbow2 PUBLIC cxx_std_17)

add_library(atlas_orb_g2o STATIC
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
target_include_directories(atlas_orb_g2o PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/g2o
)
target_compile_definitions(atlas_orb_g2o PRIVATE UNIX)
target_link_libraries(atlas_orb_g2o PUBLIC Eigen3::Eigen)
target_compile_features(atlas_orb_g2o PUBLIC cxx_std_17)

add_library(atlas_orb_sophus INTERFACE)
target_include_directories(atlas_orb_sophus INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/Sophus
)
target_link_libraries(atlas_orb_sophus INTERFACE Eigen3::Eigen)

add_library(atlas::orb_dbow2 ALIAS atlas_orb_dbow2)
add_library(atlas::orb_g2o ALIAS atlas_orb_g2o)
add_library(atlas::orb_sophus ALIAS atlas_orb_sophus)
