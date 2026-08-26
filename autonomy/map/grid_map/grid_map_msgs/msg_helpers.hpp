/*
 * msg_helpers.hpp
 *
 * ROS-free MultiArray helpers for grid_map protobuf messages
 * (ported from grid_map_ros/GridMapMsgHelpers.hpp).
 */

#pragma once

#include <map>
#include <string>

#include <Eigen/Core>

#include <automsgs/msgs/std_msgs/float32_multi_array.pb.h>

#include "autonomy/common/logging.hpp"

namespace grid_map {

inline int nDimensions() { return 2; }

enum class StorageIndices { Column, Row };

inline const std::map<StorageIndices, std::string>& storageIndexNames() {
  static const std::map<StorageIndices, std::string> names = {
      {StorageIndices::Column, "column_index"},
      {StorageIndices::Row, "row_index"},
  };
  return names;
}

inline bool isRowMajor(
    const automsgs::msgs::std_msgs::Float32MultiArray& message) {
  if (message.layout().dim_size() < 1) {
    AERROR << "isRowMajor() failed: layout has no dimensions.";
    return false;
  }
  const std::string& label = message.layout().dim(0).label();
  if (label == storageIndexNames().at(StorageIndices::Column)) {
    return false;
  }
  if (label == storageIndexNames().at(StorageIndices::Row)) {
    return true;
  }
  AERROR << "isRowMajor() failed because layout label is not set correctly.";
  return false;
}

inline unsigned int getCols(
    const automsgs::msgs::std_msgs::Float32MultiArray& message) {
  if (isRowMajor(message)) {
    return message.layout().dim(1).size();
  }
  return message.layout().dim(0).size();
}

inline unsigned int getRows(
    const automsgs::msgs::std_msgs::Float32MultiArray& message) {
  if (isRowMajor(message)) {
    return message.layout().dim(0).size();
  }
  return message.layout().dim(1).size();
}

template <typename EigenType_>
bool matrixEigenCopyToMultiArrayMessage(
    const EigenType_& e, automsgs::msgs::std_msgs::Float32MultiArray& m) {
  auto* layout = m.mutable_layout();
  layout->clear_dim();
  layout->set_data_offset(0);

  auto* dim0 = layout->add_dim();
  dim0->set_stride(static_cast<uint32_t>(e.size()));
  dim0->set_size(static_cast<uint32_t>(e.outerSize()));

  auto* dim1 = layout->add_dim();
  dim1->set_stride(static_cast<uint32_t>(e.innerSize()));
  dim1->set_size(static_cast<uint32_t>(e.innerSize()));

  if (EigenType_::IsRowMajor) {
    dim0->set_label(storageIndexNames().at(StorageIndices::Row));
    dim1->set_label(storageIndexNames().at(StorageIndices::Column));
  } else {
    dim0->set_label(storageIndexNames().at(StorageIndices::Column));
    dim1->set_label(storageIndexNames().at(StorageIndices::Row));
  }

  m.clear_data();
  m.mutable_data()->Reserve(static_cast<int>(e.size()));
  for (Eigen::Index i = 0; i < e.size(); ++i) {
    m.add_data(e.data()[i]);
  }
  return true;
}

template <typename EigenType_>
bool multiArrayMessageCopyToMatrixEigen(
    const automsgs::msgs::std_msgs::Float32MultiArray& m, EigenType_& e) {
  if (m.layout().dim_size() < 2) {
    AERROR << "multiArrayMessageCopyToMatrixEigen() failed: bad layout.";
    return false;
  }
  if (static_cast<bool>(EigenType_::IsRowMajor) != isRowMajor(m)) {
    AERROR << "multiArrayMessageCopyToMatrixEigen() failed: storage order "
              "mismatch.";
    return false;
  }
  const unsigned int rows = getRows(m);
  const unsigned int cols = getCols(m);
  if (static_cast<size_t>(m.data_size()) <
      static_cast<size_t>(rows) * static_cast<size_t>(cols)) {
    AERROR << "multiArrayMessageCopyToMatrixEigen() failed: data size too "
              "small.";
    return false;
  }
  EigenType_ temp(rows, cols);
  temp = Eigen::Map<const EigenType_>(m.data().data(), rows, cols);
  e = temp;
  return true;
}

}  // namespace grid_map
