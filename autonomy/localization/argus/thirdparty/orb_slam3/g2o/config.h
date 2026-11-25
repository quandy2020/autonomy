#ifndef ATLAS_THIRD_PARTY_ORB_G2O_CONFIG_H_
#define ATLAS_THIRD_PARTY_ORB_G2O_CONFIG_H_

// ORB-SLAM3's bundled g2o is a static, single-threaded private dependency.
#ifdef EIGEN_DEFAULT_TO_ROW_MAJOR
#error "ORB-SLAM3 g2o requires column-major Eigen matrices"
#endif

#endif  // ATLAS_THIRD_PARTY_ORB_G2O_CONFIG_H_
