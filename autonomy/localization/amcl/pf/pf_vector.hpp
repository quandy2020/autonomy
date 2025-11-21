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

#include <stdio.h>

namespace autonomy {
namespace localization {
namespace amcl {
namespace pf {

#ifdef __cplusplus
extern "C" {
#endif

// The basic vector
typedef struct
{
  double v[3];
} pf_vector_t;


// The basic matrix
typedef struct
{
  double m[3][3];
} pf_matrix_t;


// Return a zero vector
pf_vector_t pf_vector_zero(void);

// Check for NAN or INF in any component
// int pf_vector_finite(pf_vector_t a);

// Print a vector
// void pf_vector_fprintf(pf_vector_t s, FILE * file, const char * fmt);

// Simple vector addition
// pf_vector_t pf_vector_add(pf_vector_t a, pf_vector_t b);

// Simple vector subtraction
pf_vector_t pf_vector_sub(pf_vector_t a, pf_vector_t b);

// Transform from local to global coords (a + b)
pf_vector_t pf_vector_coord_add(pf_vector_t a, pf_vector_t b);

// Transform from global to local coords (a - b)
// pf_vector_t pf_vector_coord_sub(pf_vector_t a, pf_vector_t b);


// Return a zero matrix
pf_matrix_t pf_matrix_zero(void);

// Check for NAN or INF in any component
// int pf_matrix_finite(pf_matrix_t a);

// Print a matrix
// void pf_matrix_fprintf(pf_matrix_t s, FILE * file, const char * fmt);

// Compute the matrix inverse.  Will also return the determinant,
// which should be checked for underflow (indicated singular matrix).
// pf_matrix_t pf_matrix_inverse(pf_matrix_t a, double *det);

// Decompose a covariance matrix [a] into a rotation matrix [r] and a
// diagonal matrix [d] such that a = r * d * r^T.
void pf_matrix_unitary(pf_matrix_t * r, pf_matrix_t * d, pf_matrix_t a);

#ifdef __cplusplus
}
#endif

}   // namespace pf
}   // namespace amcl
}   // namespace localization
}   // namespace autonomy