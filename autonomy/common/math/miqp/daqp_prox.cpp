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
 * @file daqp_prox.cpp
 * @brief Proximal-point wrapper around the dual active-set method.
 *
 * Declarations and the algorithm contract live in the matching header.
 * This file holds the definitions of the vendored DAQP C solver.
 */

#include "autonomy/common/math/miqp/daqp_prox.hpp"
#include "autonomy/common/math/miqp/auxiliary.hpp"
#include "autonomy/common/math/miqp/utils.hpp"
#include <math.h>


namespace autonomy {
namespace common {
namespace math {
namespace miqp {
/**
 * @brief Inner iterations before a face-projection proximal step is used.
 */
#define DAQP_PROX_FACE_START 16

/**
 * @brief Number of consecutive face-projection steps once they start.
 */
#define DAQP_PROX_FACE_STEPS 3

static int ProxProjectStep(DAQPWorkspace* work, c_float eps);
static int ProxStep(DAQPWorkspace* work, c_float* s_prev, c_float eps, int projected);
static void ProxRescale(DAQPWorkspace* work, c_float eps, c_float eps_new);
static int ProxIsInfeasible(const DAQPWorkspace* work);

/**
 * @brief Outer proximal-point / semi-proximal loop.
 *
 * Quadratic programs (Rinv or RinvD is set): diagonal Hessians use a
 * semi-proximal method, perturbing only singular coordinate directions.
 * Dense Hessians use a full proximal shift when Cholesky detects numerical
 * singularity. If H is already positive definite (n_prox == 0), the inner
 * program equals the original and the loop exits after one solve.
 *
 * Linear programs (Rinv and RinvD are both NULL) use classical
 * regularisation-based smoothing with adaptive eps.
 *
 * @param[in,out] work Workspace whose Hessian needs a proximal shift.
 * @return DAQP exit flag (positive on success).
 */
int DaqpProx(DAQPWorkspace *work){
    int i, total_iter = 0;
    c_float s_prev = -1; // Exact step length of the latest ProxStep (-1: none)
    int center_relaxed = 0;
    int nx, is_lp;
    const c_float relaxation = 1.5;
    int exitflag = DAQP_EXIT_ITERLIMIT; // If no iteration can be taken
    c_float *swp_ptr;
    c_float max_diff, tol_stat;
    c_float eta = work->settings->eta_prox;
    c_float eps;

    // nh act as a counter for outer iterations
    work->nh = 0;

    nx = work->n;
    is_lp = (work->Rinv == NULL && work->RinvD == NULL);
    eps = is_lp ? 1.0 : DaqpGetProximalRegularization(work);

    // For a QP whose Hessian is already positive definite (n_prox == 0),
    // no direction needs a proximal shift.  The inner QP equals the
    // original problem, so one solve gives the exact solution.
    const int all_pd = (!is_lp) && (work->n_prox == 0);

    // eps of semi-proximal directions can be changed cheaply (ProxRescale).
    // A failed inner problem (often due to a small eps) is resolved with eps_max
    const int adaptive = !is_lp && !all_pd && work->avi == NULL &&
        work->prox_mask != NULL && work->n_prox < nx;
    c_float eps_max = eps;
    int rescaled = 0;
    if(adaptive){
        c_float hmax = 0;
        for(i = 0; i < nx; i++){
            c_float hii = work->qp->H[(size_t)i*nx+i];
            if(hii < 0) hii = -hii;
            if(hii > hmax) hmax = hii;
        }
/**
 * @brief Reset an eps that an earlier solve has raised (as in DaqpUpdateRinv)
 */
        c_float eps0 = work->settings->eps_prox < 0 ? -work->settings->eps_prox : work->settings->eps_prox;
        if(eps0 < sqrt(work->settings->zero_tol)*hmax) eps0 = sqrt(work->settings->zero_tol)*hmax;
        if(eps > 1.01*eps0){
            ProxRescale(work,eps,eps0);
            eps = eps0;
            rescaled = 1;
        }
        eps_max = DAQP_PROX_EPS_MAX*hmax > eps ? DAQP_PROX_EPS_MAX*hmax : eps;
    }

    // A negative eta selects an automatic tolerance. Preserve the established
    // default, but tighten it when the user requests a non-default dual
    // tolerance. Skip this entirely for positive-definite QPs, where no
    // proximal convergence test is needed.
    if(!all_pd && eta < 0.0){
        eta = DAQP_AUTO_ETA_CAP;
        if(work->settings->dual_tol != DAQP_DEFAULT_DUAL_TOL &&
           0.1 * work->settings->dual_tol < eta)
            eta = 0.1 * work->settings->dual_tol;
    }

    while(total_iter < work->settings->iter_limit){
        /* ----------------------------------------------------------------
         * Perturb the problem: form v = R'\(f - eps_mask * x_old)
         * ----------------------------------------------------------------*/
        if(is_lp){
            // No Hessian factor.  Adapt eps heuristically: grow when the
            // inner LP stalls (iterations==1), shrink otherwise to improve
            // accuracy. Keep the deterministic initial value for the first
            // solve; work->iterations has no current-loop value yet.
            if(total_iter > 0)
                eps *= (work->iterations == 1) ? 10.0 : 0.9;
            if(eps > 1e3) eps = 1e3;
            for(i = 0; i < nx; i++)
                work->v[i] = work->qp->f[i]*eps - work->x[i];
        }
        else{
            if(work->prox_mask == NULL || work->n_prox == nx){
                // Dense singular Hessians use a full shift. Avoid a mask
                // load and branch for every component on every outer step.
                if(work->qp->f != NULL)
                    for(i = 0; i < nx; i++)
                        work->v[i] = work->qp->f[i] - eps * work->x[i];
                else
                    for(i = 0; i < nx; i++)
                        work->v[i] = -eps * work->x[i];
            }
            else{
                // Diagonal Hessians can regularize only singular directions.
                if(work->qp->f != NULL)
                    for(i = 0; i < nx; i++)
                        work->v[i] = work->qp->f[i]
                                     - (work->prox_mask[i] ? eps : 0.0) * work->x[i];
                else
                    for(i = 0; i < nx; i++)
                        work->v[i] = -(work->prox_mask[i] ? eps : 0.0)
                                     * work->x[i];
            }
            DaqpUpdateV(work->v, work);
        }

        DaqpUpdateD(work, work->qp->bupper, work->qp->blower);
        if(rescaled){ // The working set is factored anew after a rescaling
            ResetDaqpWorkspace(work);
            DaqpActivateConstraints(work);
            rescaled = 0;
        }

        // xold <-- x  (pointer swap avoids copying)
        swp_ptr = work->xold; work->xold = work->x; work->x = swp_ptr;

        /* ----------------------------------------------------------------
         * Solve the (regularised) least-distance problem
         * ----------------------------------------------------------------*/
        work->u = work->x;
        work->nh++;
        exitflag = DaqpLdp(work);

        total_iter += work->iterations;
        if(adaptive && eps < eps_max && total_iter < work->settings->iter_limit &&
                (exitflag == DAQP_EXIT_CYCLE ||
                 (exitflag == DAQP_EXIT_INFEASIBLE && !ProxIsInfeasible(work)))){
            ProxRescale(work,eps,eps_max);
            eps = eps_max;
            rescaled = 1;
            s_prev = -1;
            for(i = 0; i < nx; i++) work->x[i] = work->xold[i]; // The center
            continue;
        }
        if(exitflag < 0)
            break;              // Inner solver failed -- propagate error
        Ldp2qpSolution(work); // Recover QP primal from LDP dual

        if(eps == 0) break;     // No regularisation -> single outer step

        /* ----------------------------------------------------------------
         * If H is fully positive definite, the inner QP is the original
         * problem.  The first solve gives the exact solution.
         * ----------------------------------------------------------------*/
        if(all_pd){
            exitflag = DAQP_EXIT_OPTIMAL;
            break;
        }

        /* ----------------------------------------------------------------
         * Convergence check: fixed point  ||x - x_old||_inf < tol_stat.
         *
         * A fixed point is a valid stationarity certificate regardless of
         * how many active-set changes the inner solve needed.  Checking it
         * after every successful solve avoids extra outer iterations and,
         * unlike objective stagnation, cannot label a non-stationary point
         * as optimal.
         * ----------------------------------------------------------------*/
        tol_stat = is_lp ? eta*eps : eta/eps;
        for(i = 0; i < nx; i++){
            max_diff = work->x[i] - work->xold[i];
            if(max_diff > tol_stat || max_diff < -tol_stat) break;
        }
        if(i == nx){
            if(center_relaxed &&
                    total_iter < work->settings->iter_limit){
                center_relaxed = 0;
                continue; // Confirm convergence from the feasible iterate.
            }
            exitflag = DAQP_EXIT_OPTIMAL;
            break;
        }

        // Unchanged working set => accelerate by moving the center along the
        // step (relax the step for an AVI). Convergence is confirmed afterwards.
        // After many outer steps, also accelerate small working-set changes,
        // projecting the step onto the new face first.
        center_relaxed = 0;
        int projected = 0;
        if(work->iterations != 1){ // The working set has changed
            s_prev = -1;
            projected = !is_lp && work->avi == NULL && work->settings->eps_prox < 0 &&
                work->nh >= DAQP_PROX_FACE_START && work->iterations <= 3 &&
                ProxProjectStep(work,eps);
        }
        if((work->iterations == 1 || projected) && work->n_active < nx &&
                total_iter < work->settings->iter_limit){
            if(work->avi != NULL){
                for(i = 0; i < nx; i++)
                    work->x[i] = work->xold[i]
                        + relaxation*(work->x[i] - work->xold[i]);
                center_relaxed = 1;
            }
            else{
                const int step_flag = ProxStep(work,&s_prev,eps,projected);
                if(step_flag == DAQP_EXIT_UNBOUNDED){
                    exitflag = DAQP_EXIT_UNBOUNDED;
                    break;
                }
                center_relaxed = step_flag;
            }
        }
    }

    // Finalize
    if(total_iter >= work->settings->iter_limit) exitflag = DAQP_EXIT_ITERLIMIT;
    // Refine x (skipped for short solves)
    if(exitflag > 0 && total_iter > DAQP_REFINE_MIN_ITER) DaqpRefinePrimal(work);
    if(is_lp){
        for(i = 0; i < work->n_active; i++)
            work->lam_star[i] /= eps; // Rescale dual variables
    }
    else{
        /*
         * DaqpExtractResult forms 0.5*(fval - ||v||^2). Correct the
         * regularized objective here while the reconstructed eps is local,
         * avoiding a second reconstruction during result extraction.
         */
        c_float prox_norm = 0.0;
        for(i = 0; i < nx; i++){
            if(work->prox_mask == NULL || work->prox_mask[i])
                prox_norm += work->x[i]*work->x[i];
        }
        work->fval += eps*prox_norm;
    }
    work->iterations = total_iter;
    return exitflag;
}

/* At the inner optimum, -g = E*(x-xold) + A_W'*lambda (E the proximal shift).
 * The steepest descent direction on the active face, in the metric of H+E, is
 * therefore Rinv*(I-M_W'*(M_W*M_W')^{-1}*M_W)*Rinv'*E*(x-xold). It is stored
 * as x-xold for ProxStep (xold is not needed as a center again).
 */
static int ProxProjectStep(DAQPWorkspace* work, c_float eps){
    int i, j, Id;
    const int n = work->n, na = work->n_active;
    c_float *r = work->xold, *saved_v = work->v, sum;
    // Near a vertex, the inner solver resolves the remaining face cheaper
    if(na >= 9*n/10 || work->sing_ind != DAQP_EMPTY_IND || work->qp->H == NULL) return 0;
    for(i = 0; i < na; i++) if(DAQP_IS_SOFT(work->WS[i])) return 0;
    // r = Rinv'*E*(x-xold) (DaqpUpdateV transforms in place into v)
    for(i = 0; i < n; i++)
        r[i] = (work->prox_mask == NULL || work->prox_mask[i]) ? eps*(work->x[i]-r[i]) : 0;
    work->v = r;
    DaqpUpdateV(r,work);
    work->v = saved_v;
    // r <-- r - M_W'*(M_W*M_W')^{-1}*M_W*r
    for(i = 0; i < na; i++){
        Id = work->WS[i];
        if(Id >= work->ms)
            for(j = 0, sum = 0; j < n; j++) sum += work->M[(size_t)(Id-work->ms)*n+j]*r[j];
        else if(work->Rinv != NULL)
            for(j = Id, sum = 0; j < n; j++) sum += work->Rinv[DAQP_R_OFFSET(Id,n)+j]*r[j];
        else sum = r[Id];
        work->xldl[i] = sum;
    }
    DaqpSolveWorkingSet(work);
    DaqpSubWorkingSetRows(work,work->xldl,r);
    DaqpApplyRinv(work,r);
    for(i = 0; i < n; i++) work->xold[i] = work->x[i]-r[i];
    work->reuse_ind = 0;
    return 1;
}

/**
 * @brief Step length -g'd/d'Hd that minimizes the objective along d = x-x_old (d in xldl).
 *        Returns DAQP_INF if d has no curvature, -1 if d is not a descent direction
 */
static c_float ProxCurvatureStep(DAQPWorkspace* work){
    int i, j;
    const int n = work->n;
    const DAQPProblem* qp = work->qp;
/**
 * @brief Scratch (reuse_ind is reset by the next DaqpUpdateD)
 */
    c_float *d = work->xldl, *hd = work->zldl;
    c_float gd = 0, dhd = 0, dd = 0, hmax = 0;

    for(i = 0; i < n; i++) d[i] = work->x[i] - work->xold[i];
    if(qp->f != NULL) for(i = 0; i < n; i++) gd += qp->f[i]*d[i];
    if(qp->H != NULL){
        for(i = 0; i < n; i++){
            const c_float* Hi = qp->H+(size_t)i*n;
            const c_float hii = Hi[i] < 0 ? -Hi[i] : Hi[i];
            c_float sum = 0;
            for(j = 0; j < n; j++) sum += Hi[j]*d[j];
            hd[i] = sum;
            if(hii > hmax) hmax = hii;
        }
        for(i = 0; i < n; i++){
            gd += work->x[i]*hd[i];
            dhd += d[i]*hd[i];
            dd += d[i]*d[i];
        }
    }
    if(gd >= 0) return -1;
    return (dhd > work->settings->zero_tol*hmax*dd) ? -gd/dhd : DAQP_INF;
}

/**
 * @brief First inactive constraint that blocks x + S*(x-x_old) for S < *S (*S is shortened to
 *        the blocking step, *lower marks a lower bound)
 */
static int ProxBlockingConstraint(DAQPWorkspace* work, c_float* S, int* lower){
    int i, j, ind = DAQP_EMPTY_IND;
    const int n = work->n, m = work->m, ms = work->ms;
    const DAQPProblem* qp = work->qp;
    c_float ad, ax, sb;
    for(i = 0; i < m; i++){
        if(work->sense[i] & (DAQP_ACTIVE + DAQP_IMMUTABLE + DAQP_SET_ASIDE)) continue;
        if(i < ms){ ax = work->x[i]; ad = ax - work->xold[i]; }
        else{
            const c_float* a = qp->A+(size_t)(i-ms)*n;
            for(j = 0, ad = 0, ax = 0; j < n; j++){
                ax += a[j]*work->x[j];
                ad += a[j]*(work->x[j] - work->xold[j]);
            }
        }
        if(ad > 0 && qp->bupper[i] < DAQP_INF) sb = (qp->bupper[i]-ax)/ad;
        else if(ad < 0 && qp->blower[i] > -DAQP_INF) sb = (qp->blower[i]-ax)/ad;
        else continue;
        if(sb < *S){
            *S = sb;
            *lower = ad < 0;
            ind = i;
        }
    }
    return ind;
}

/* --------------------------------------------------------------------------
 * ProxStep  --  step along the latest proximal step d = x - x_old
 *
 * With an unchanged working set, the proximal iterates converge slowly along
 * d. x is therefore moved to the minimizer along d, or to the first blocking
 * constraint, which is added to the working set (dependent ones are set aside).
 *
 * Returns 1 if x was moved, 0 otherwise, and DAQP_EXIT_UNBOUNDED for an LP
 * with an unblocked descent direction.
 * --------------------------------------------------------------------------*/
static int ProxStep(DAQPWorkspace* work, c_float* s_prev, c_float eps, int projected){
    int i, k, ind, lower = 0, moved = 0, skipped = 0, first = 1, n_projections = 0;
    c_float S;
    while((S = ProxCurvatureStep(work)) >= 0){
        const int null_direction = S >= DAQP_INF;
        const c_float* d = work->xldl;
        if(first){ // Lagged (Barzilai-Borwein) step length
            const c_float s_exact = S;
            if(s_exact < DAQP_INF && *s_prev >= 0)
                S = (*s_prev < 2*s_exact) ? *s_prev : 2*s_exact;
            *s_prev = (s_exact < DAQP_INF) ? s_exact : -1;
            first = 0;
        }
        ind = ProxBlockingConstraint(work,&S,&lower);
        // Roundoff in a projected direction is amplified by a long step.
        // Keep the inner iterate if the step would leave the active face.
        if(projected && S > 0 && S < DAQP_INF){
            for(i = 0; i < work->n_active; i++){
                const int Id = work->WS[i];
                c_float ad = 0;
                if(Id < work->ms) ad = d[Id];
                else for(k = 0; k < work->n; k++)
                    ad += work->qp->A[(size_t)(Id-work->ms)*work->n+k]*d[k];
                if(fabs(S*ad) > work->settings->primal_tol) break;
            }
            if(i < work->n_active) break;
        }
        if(ind == DAQP_EMPTY_IND){
            if(S < DAQP_INF){ // The minimizer along d
                for(k = 0; k < work->n; k++) work->x[k] += S*d[k];
                moved = 1;
            }
            else if(work->qp->H == NULL && !moved && !skipped)
                return DAQP_EXIT_UNBOUNDED;
            break;
        }
        *s_prev = -1; // Blocked: the working set changes
        // Advance to the blocking constraint and activate it (no step back if
        // it is already violated)
        if(S >= 0) for(k = 0; k < work->n; k++) work->x[k] += S*d[k];
        moved = 1;
        if(lower) DAQP_SET_LOWER(ind);
        else DAQP_SET_UPPER(ind);
        DaqpAddConstraint(work, ind, lower ? -1.0 : 1.0);
        if(work->sing_ind == DAQP_EMPTY_IND){
            // Along a null direction of H, continue on the new face
            if(null_direction && work->settings->eps_prox < 0 &&
                    work->nh >= DAQP_PROX_FACE_START &&
                    n_projections++ < DAQP_PROX_FACE_STEPS &&
                    ProxProjectStep(work,eps)){
                projected = 1;
                continue;
            }
            break;
        }
        // Linearly dependent on the active constraints: set it aside
        work->sense[DaqpDropSingularLast(work)] |= DAQP_SET_ASIDE;
        skipped = 1;
    }
    if(skipped)
        for(i = 0; i < work->m; i++) work->sense[i] &= ~DAQP_SET_ASIDE;
    return moved;
}

/**
 * @brief Change eps of the semi-proximal directions to eps_new. The directions are decoupled,
 *        so only their columns of Rinv and M are scaled (the working set has to be refactored
 *        afterwards)
 */
static void ProxRescale(DAQPWorkspace* work, c_float eps, c_float eps_new){
    int i, j, disp;
    const int n = work->n, ms = work->ms;
    const int* mask = work->prox_mask;
    const c_float* H = work->qp->H;
/**
 * @brief The scaling of column j of Rinv
 */
#define DAQP_PROX_RATIO(j) sqrt((H[(size_t)(j)*n+(j)]+eps)/(H[(size_t)(j)*n+(j)]+eps_new))
    for(i = 0; i < n; i++){
        if(!mask[i]) continue;
        const c_float r = DAQP_PROX_RATIO(i);
        if(work->Rinv == NULL){
            work->RinvD[i] *= r;
            if(i < ms) work->scaling[i] /= r;
        }
        else if(i < ms && (work->state & DAQP_STATE_RINV_NORMALIZED))
            work->scaling[i] /= r;
        else
            work->Rinv[DAQP_R_OFFSET(i,n)+i] *= r;
    }
    for(i = ms, disp = 0; i < work->m; i++, disp += n){
        c_float* mi = work->M+disp;
        c_float norm2 = 1, sc;
        for(j = 0; j < n; j++){
            if(!mask[j] || mi[j] == 0) continue;
            const c_float r = DAQP_PROX_RATIO(j);
            norm2 += (r*r-1)*mi[j]*mi[j];
            mi[j] *= r;
        }
        if(norm2 == 1) continue;
        sc = 1/sqrt(norm2);
        for(j = 0; j < n; j++) mi[j] *= sc;
        work->scaling[i] *= sc;
    }
#undef DAQP_PROX_RATIO
}

/**
 * @brief Whether the certificate of infeasibility from DaqpLdp (the dependency lam_star of a
 *        singular working set) is valid and implies a violation above primal_tol. In the
 *        constraints of the QP, q = S*lam_star gives sum q_i a_i = 0, and hence max violation
 *        >= (-sum q_i b_i - sum_wrong |q_i| (bu_i-bl_i))/sum |q_i|, where the sign of q_i is
 *        wrong for a violated b_i (only acceptable for two-sided constraints)
 */
static int ProxIsInfeasible(const DAQPWorkspace* work){
    int i, Id, lower;
    c_float q, gap = 0, norm = 0, wrong = 0;
    if(work->sing_ind == DAQP_EMPTY_IND) return 0;
    for(i = 0; i < work->n_active; i++){
        Id = work->WS[i];
        lower = DAQP_IS_LOWER(Id);
        q = work->scaling != NULL ? work->lam_star[i]*work->scaling[Id] : work->lam_star[i];
        gap -= q*(lower ? work->qp->blower[Id] : work->qp->bupper[Id]);
        if(!DAQP_IS_IMMUTABLE(Id) && (lower ? q > 0 : q < 0)){
            if(work->qp->bupper[Id] < DAQP_INF && work->qp->blower[Id] > -DAQP_INF)
                gap -= (q < 0 ? -q : q)*(work->qp->bupper[Id]-work->qp->blower[Id]);
            else wrong += q < 0 ? -q : q;
        }
        norm += q < 0 ? -q : q;
    }
    return wrong <= 1e-8*norm && gap > work->settings->primal_tol*norm;
}
}  // namespace miqp
}  // namespace math
}  // namespace common
}  // namespace autonomy
