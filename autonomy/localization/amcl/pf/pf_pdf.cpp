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

#include "autonomy/localization/amcl/pf/pf_pdf.hpp"

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>


#include "autonomy/localization/amcl/portable_utils.hpp"


namespace autonomy {
namespace localization {
namespace amcl {
namespace pf {

// // Random number generator seed value
// static unsigned int pf_pdf_seed;


// /**************************************************************************
//     * Gaussian
//     *************************************************************************/

// // Create a gaussian pdf
// pf_pdf_gaussian_t * pf_pdf_gaussian_alloc(pf_vector_t x, pf_matrix_t cx)
// {
//     pf_matrix_t cd;
//     pf_pdf_gaussian_t * pdf;

//     pdf = static_cast<pf_pdf_gaussian_t*>(calloc(1, sizeof(pf_pdf_gaussian_t)));

//     pdf->x = x;
//     pdf->cx = cx;
//     // pdf->cxi = pf_matrix_inverse(cx, &pdf->cxdet);

//     // Decompose the convariance matrix into a rotation
//     // matrix and a diagonal matrix.
//     pf_matrix_unitary(&pdf->cr, &cd, pdf->cx);
//     pdf->cd.v[0] = sqrt(cd.m[0][0]);
//     pdf->cd.v[1] = sqrt(cd.m[1][1]);
//     pdf->cd.v[2] = sqrt(cd.m[2][2]);

//     // Initialize the random number generator
//     // pdf->rng = gsl_rng_alloc(gsl_rng_taus);
//     // gsl_rng_set(pdf->rng, ++pf_pdf_seed);
//     srand48(++pf_pdf_seed);

//     return pdf;
// }


// // Destroy the pdf
// void pf_pdf_gaussian_free(pf_pdf_gaussian_t * pdf)
// {
//     // gsl_rng_free(pdf->rng);
//     free(pdf);
// }


// /*
// // Compute the value of the pdf at some point [x].
// double pf_pdf_gaussian_value(pf_pdf_gaussian_t *pdf, pf_vector_t x)
// {
//     int i, j;
//     pf_vector_t z;
//     double zz, p;

//     z = pf_vector_sub(x, pdf->x);

//     zz = 0;
//     for (i = 0; i < 3; i++)
//     for (j = 0; j < 3; j++)
//         zz += z.v[i] * pdf->cxi.m[i][j] * z.v[j];

//     p =  1 / (2 * M_PI * pdf->cxdet) * exp(-zz / 2);

//     return p;
// }
// */


// // Generate a sample from the pdf.
// pf_vector_t pf_pdf_gaussian_sample(pf_pdf_gaussian_t * pdf)
// {
//     int i, j;
//     pf_vector_t r;
//     pf_vector_t x;

//     // Generate a random vector
//     for (i = 0; i < 3; i++) {
//     // r.v[i] = gsl_ran_gaussian(pdf->rng, pdf->cd.v[i]);
//     r.v[i] = pf_ran_gaussian(pdf->cd.v[i]);
//     }

//     for (i = 0; i < 3; i++) {
//     x.v[i] = pdf->x.v[i];
//     for (j = 0; j < 3; j++) {
//         x.v[i] += pdf->cr.m[i][j] * r.v[j];
//     }
//     }

//     return x;
// }

// // Draw randomly from a zero-mean Gaussian distribution, with standard
// // deviation sigma.
// // We use the polar form of the Box-Muller transformation, explained here:
// //   http://www.taygeta.com/random/gaussian.html
// double pf_ran_gaussian(double sigma)
// {
//     double x1, x2, w, r;

//     do {
//     do {
//         r = drand48();
//     } while (r == 0.0);
//     x1 = 2.0 * r - 1.0;
//     do {
//         r = drand48();
//     } while (r == 0.0);
//     x2 = 2.0 * r - 1.0;
//     w = x1 * x1 + x2 * x2;
//     } while (w > 1.0 || w == 0.0);

//     return sigma * x2 * sqrt(-2.0 * log(w) / w);
// }

}   // namespace pf
}   // namespace amcl
}   // namespace localization
}   // namespace autonomy