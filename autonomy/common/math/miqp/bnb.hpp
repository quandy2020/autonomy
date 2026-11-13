/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file bnb.hpp
 * @brief Branch-and-bound over binary simple bounds.
 *
 * Vendored DAQP (MIT). A binary constraint must sit at either its lower or
 * its upper bound. The lower-bound choice is stored in bit DAQP_LOWER_BIT of
 * the packed identifier so a node can encode both the constraint and the
 * side that was fixed.
 */

#ifndef DAQP_BNB_H
# define DAQP_BNB_H


#include "autonomy/common/math/miqp/types.hpp"
#include "autonomy/common/math/miqp/constants.hpp"
#include "autonomy/common/math/miqp/daqp.hpp"

namespace autonomy {
namespace common {
namespace math {
namespace miqp {

/**
 * @brief Solve the mixed-integer quadratic program in @p work.
 *
 * @param[in,out] work Workspace with binaries marked DAQP_BINARY.
 * @return DAQP exit flag (positive on success).
 */
int DaqpBnb(DAQPWorkspace* work);

/**
 * @brief Solve the continuous relaxation at @p node.
 *
 * @param[in,out] node Tree node being processed.
 * @param[in,out] work Workspace that holds the relaxation.
 * @return DAQP exit flag of the relaxation.
 */
int DaqpProcessNode(DAQPNode* node, DAQPWorkspace* work);

/**
 * @brief Pick the most fractional binary to branch on.
 *
 * @param[in] work Workspace of a solved relaxation.
 * @return Packed binary identifier, or DAQP_EMPTY_IND if the node is integer.
 */
int DaqpGetBranchId(DAQPWorkspace* work);

/**
 * @brief Push the two children of @p node onto the tree.
 *
 * @param[in,out] node Parent whose warm start is saved.
 * @param[in] branch_id Packed binary identifier of the split.
 * @param[in,out] work Workspace that owns the tree.
 */
void DaqpSpawnChildren(DAQPNode* node, const int branch_id,
                         DAQPWorkspace* work);

/**
 * @brief Drop mutable actives down to @p n_clean.
 *
 * @param[in] n_clean Number of working-set entries to keep.
 * @param[in,out] work Workspace to clean.
 */
void DaqpNodeCleanupWorkspace(int n_clean, DAQPWorkspace* work);

/**
 * @brief Restore the saved working set of @p node.
 *
 * @param[in] node Node whose warm start is loaded.
 * @param[in,out] work Workspace that receives the working set.
 * @return Negative if the restored equalities are inconsistent.
 */
int DaqpWarmstartNode(DAQPNode* node, DAQPWorkspace* work);

/**
 * @brief Store the current mutable working set on @p node.
 *
 * @param[in,out] node Node that receives WS_start / WS_end.
 * @param[in,out] work Workspace whose working set is copied into tree_WS.
 */
void DaqpSaveWarmstart(DAQPNode* node, DAQPWorkspace* work);

/**
 * @brief Activate a packed upper/lower binary identifier.
 *
 * @param[in] add_id Constraint index with an optional lower-bound flag.
 * @param[in,out] work Workspace that receives the equality.
 * @return Nonzero when the addition succeeded.
 */
int DaqpAddUpperLower(const int add_id, DAQPWorkspace* work);

/**
 * @brief Cold-start a node by re-fixing its path from the root.
 *
 * @param[in] node Node to restart.
 * @param[in,out] work Workspace that is rebuilt.
 * @return Negative if the path is inconsistent.
 */
int DaqpSetupColdBnb(DAQPNode* node, DAQPWorkspace* work);

/**
 * @brief Bit used to encode “fixed at the lower bound” in a packed Id.
 */
#define DAQP_LOWER_BIT 16

/**
 * @brief Extract the lower-bound flag from a packed identifier.
 *
 * @param x Packed binary identifier.
 */
#define DAQP_EXTRACT_LOWER_FLAG(x) (x >> (DAQP_LOWER_BIT - 1))

/**
 * @brief Drop the lower-bound flag from a packed identifier.
 *
 * @param x Packed binary identifier.
 */
#define DAQP_REMOVE_LOWER_FLAG(x) (x & ~(1 << DAQP_LOWER_BIT))

/**
 * @brief Set the lower-bound flag on a constraint index.
 *
 * @param x Constraint index.
 */
#define DAQP_ADD_LOWER_FLAG(x) (x | (1 << DAQP_LOWER_BIT))

/**
 * @brief Flip the lower-bound flag of a packed identifier.
 *
 * @param x Packed binary identifier.
 */
#define DAQP_TOGGLE_LOWER_FLAG(x) (x ^ (1 << DAQP_LOWER_BIT))


}  // namespace miqp
}  // namespace math
}  // namespace common
}  // namespace autonomy

#endif
