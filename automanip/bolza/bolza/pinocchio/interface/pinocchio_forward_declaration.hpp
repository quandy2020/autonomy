#pragma once

/**
 * @file pinocchio_forward_declaration.hpp
 * @brief pinocchio forward declaration (bolza).
 */


/* Forward declaration of main pinocchio types */
namespace pinocchio {
template <typename Scalar, int Options>
struct JointCollectionDefaultTpl;
template <typename Scalar, int Options, template <typename S, int O> class JointCollectionTpl>
struct ModelTpl;
template <typename Scalar, int Options, template <typename S, int O> class JointCollectionTpl>
struct DataTpl;
template <typename Scalar, int Options, template <typename S, int O> class JointCollectionTpl>
struct JointModelTpl;
}  // namespace pinocchio
