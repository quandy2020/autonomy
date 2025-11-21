/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#pragma once

#include "autonomy/localization/amcl/pf/pf_vector.hpp"

#ifdef INCLUDE_RTKGUI
#include <rtk.h>
#endif

namespace autonomy {
namespace localization {
namespace amcl {
namespace pf {

// Info for a node in the tree
typedef struct pf_kdtree_node
{
    // Depth in the tree
    int leaf, depth;

    // Pivot dimension and value
    int pivot_dim;
    double pivot_value;

    // The key for this node
    int key[3];

    // The value for this node
    double value;

    // The cluster label (leaf nodes)
    int cluster;

    // Child nodes
    struct pf_kdtree_node * children[2];
} pf_kdtree_node_t;


// A kd tree
typedef struct
{
    // Cell size
    double size[3];

    // The root node of the tree
    pf_kdtree_node_t * root;

    // The number of nodes in the tree
    int node_count, node_max_count;
    pf_kdtree_node_t * nodes;

    // The number of leaf nodes in the tree
    int leaf_count;
} pf_kdtree_t;


// Create a tree
extern pf_kdtree_t * pf_kdtree_alloc(int max_size);

// Destroy a tree
extern void pf_kdtree_free(pf_kdtree_t * self);

// Clear all entries from the tree
extern void pf_kdtree_clear(pf_kdtree_t * self);

// Insert a pose into the tree
extern void pf_kdtree_insert(pf_kdtree_t * self, pf_vector_t pose, double value);

// Cluster the leaves in the tree
extern void pf_kdtree_cluster(pf_kdtree_t * self);

// Determine the probability estimate for the given pose
// extern double pf_kdtree_get_prob(pf_kdtree_t * self, pf_vector_t pose);

// Determine the cluster label for the given pose
extern int pf_kdtree_get_cluster(pf_kdtree_t * self, pf_vector_t pose);

    
#ifdef INCLUDE_RTKGUI

// Draw the tree
extern void pf_kdtree_draw(pf_kdtree_t * self, rtk_fig_t * fig);

#endif

}   // namespace pf
}   // namespace amcl
}   // namespace localization
}   // namespace autonomy