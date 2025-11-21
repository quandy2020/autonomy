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

namespace autonomy {
namespace localization {
namespace amcl {
namespace pf {

// #include <gsl/gsl_rng.h>
// #include <gsl/gsl_randist.h>

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************
 * Gaussian
 *************************************************************************/

// Gaussian PDF info
typedef struct
{
  // Mean, covariance and inverse covariance
  pf_vector_t x;
  pf_matrix_t cx;
  // pf_matrix_t cxi;
  double cxdet;

  // Decomposed covariance matrix (rotation * diagonal)
  pf_matrix_t cr;
  pf_vector_t cd;

  // A random number generator
  // gsl_rng *rng;
} pf_pdf_gaussian_t;


// Create a gaussian pdf
pf_pdf_gaussian_t * pf_pdf_gaussian_alloc(pf_vector_t x, pf_matrix_t cx);

// Destroy the pdf
void pf_pdf_gaussian_free(pf_pdf_gaussian_t * pdf);

// Compute the value of the pdf at some point [z].
// double pf_pdf_gaussian_value(pf_pdf_gaussian_t *pdf, pf_vector_t z);

// Draw randomly from a zero-mean Gaussian distribution, with standard
// deviation sigma.
// We use the polar form of the Box-Muller transformation, explained here:
//   http://www.taygeta.com/random/gaussian.html
double pf_ran_gaussian(double sigma);

// Generate a sample from the pdf.
pf_vector_t pf_pdf_gaussian_sample(pf_pdf_gaussian_t * pdf);

#ifdef __cplusplus
}
#endif

}   // namespace pf
}   // namespace amcl
}   // namespace localization
}   // namespace autonomy