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
 * @file auxiliary.cpp
 * @brief Working-set updates, primal recovery, and blocking tests.
 *
 * Declarations and the algorithm contract live in the matching header.
 * This file holds the definitions of the vendored DAQP C solver.
 */

#include "autonomy/common/math/miqp/auxiliary.hpp"
#include <float.h>
#include "autonomy/common/math/miqp/factorization.hpp"
#include "autonomy/common/math/miqp/daqp.hpp"


namespace autonomy {
namespace common {
namespace math {
namespace miqp {
/**
 * @brief True when any soft constraint has a linear (L1) weight.
 *
 * Soft constraints (see types.hpp) contribute 0.5 rho (|lam| - w)_+^2 to the
 * dual. The slack is zero while |lam| <= w (DAQP_SLACK_FIXED) and
 * rho (|lam| - w) beyond that (DAQP_SLACK_FREE). The penalty is continuously
 * differentiable, so |lam| passing w is an ordinary blocking event.
 *
 * @param work Workspace pointer.
 */
#ifdef DAQP_SOFT_WEIGHTS
#define DAQP_HAS_L1(work) \
    ((work)->settings->w_soft != 0 || (work)->rho_ls != NULL)
#else
#define DAQP_HAS_L1(work) ((work)->settings->w_soft != 0)
#endif

/**
 * @brief Reciprocal quadratic weight of the active side of constraint Id (zero selects
 *        settings->rho_soft, which is given in the normalized formulation)
 */
static inline c_float DaqpSoftRho(DAQPWorkspace *work, const int Id){
#ifdef DAQP_SOFT_WEIGHTS
    if(work->rho_ls != NULL){
        const c_float rho = DAQP_IS_LOWER(Id) ? work->rho_ls[Id] : work->rho_us[Id];
        if(rho != 0)
            return work->scaling ? rho*work->scaling[Id]*work->scaling[Id] : rho;
    }
#else
    (void)Id;
#endif
    return work->settings->rho_soft;
}

/**
 * @brief Linear weight of the active side of constraint Id, i.e. what the multiplier has to
 *        exceed for the slack to become nonzero (zero selects settings->w_soft)
 */
static inline c_float DaqpSoftW(DAQPWorkspace *work, const int Id){
#ifdef DAQP_SOFT_WEIGHTS
    if(work->w_ls != NULL){
        const c_float w = DAQP_IS_LOWER(Id) ? work->w_ls[Id] : work->w_us[Id];
        if(w != 0)
            return work->scaling ? w/work->scaling[Id] : w;
    }
#else
    (void)Id;
#endif
    return work->settings->w_soft;
}

/**
 * @brief Signed violation of a free soft constraint for multiplier lam. At lam = 0, its
 *        negative is the shift of the dual linear term.
 */
static inline c_float DaqpSoftResidual(DAQPWorkspace *work, const int Id,
        const c_float lam){
    const c_float w = DaqpSoftW(work,Id);
    if(w == 0)
        return lam == 0 ? 0 : DaqpSoftRho(work,Id)*lam;
    return DaqpSoftRho(work,Id)
        *(lam + (DAQP_IS_LOWER(Id) ? w : -w));
}

/**
 * @brief Contribution to the objective from the slack of constraint Id
 */
static inline c_float DaqpSoftPenalty(DAQPWorkspace *work, const int Id,
        const c_float lam){
    const c_float w = DaqpSoftW(work,Id);
    if(w == 0) return DaqpSoftRho(work,Id)*lam*lam; // Plain quadratic penalty
    if(DAQP_IS_SLACK_FIXED(Id)) return 0; // The slack is zero
    return DaqpSoftRho(work,Id)*(lam*lam-w*w);
}

/**
 * @brief Largest violation of a soft constraint, in the units of the original problem (only
 *        needed once a solution has been found)
 */
c_float DaqpMaxSoftSlack(DAQPWorkspace *work){
    int i;
    c_float smax = 0;
    for(i = 0; i < work->n_active; i++){
        const int Id = work->WS[i];
        if(!DAQP_IS_SOFT(Id)) continue;
        c_float S = DaqpSoftSlack(work,i);
        if(S < 0) S = -S;
        if(work->scaling != NULL) S /= work->scaling[Id]; // Undo the normalization
        if(S > smax) smax = S;
    }
    return smax;
}

/*
 * Cycling that persists after the working set has been refactorized: constraints
 * are then only added if their violation also exceeds the rounding level of
 * u = -M'lam, and DaqpSolve checks the solution with primal_tol afterwards
 * (DaqpViolatesHard).
 * Returns 0 (the cycling is reported) for the proximal method and hierarchical
 * problems, which handle cycling themselves, for a nonsymmetric AVI, or if the
 * floor is already used.
 */
int DaqpSetNoiseFloor(DAQPWorkspace *work){
    if(work->n_prox > 0 || DAQP_IS_HIERARCHICAL(work) ||
            (work->avi != NULL && !work->avi->is_symmetric) ||
            (work->state & DAQP_STATE_NOISE_FLOOR)) return 0;
    work->state |= DAQP_STATE_NOISE_FLOOR;
    return 1;
}

/**
 * @brief The (negative) rounding level of u = -M'lam, below which violations are not added (0
 *        if the floor is not used; the rows of M are normalized)
 */
c_float DaqpNoiseFloor(DAQPWorkspace *work){
    int i;
    c_float S = 0;
    if(!(work->state & DAQP_STATE_NOISE_FLOOR)) return 0;
    for(i = 0; i < work->n_active; i++) S += work->lam_star[i] < 0 ? -work->lam_star[i] : work->lam_star[i];
    return -DAQP_ADD_NOISE_GAIN*(sizeof(c_float) == sizeof(float) ? FLT_EPSILON : DBL_EPSILON)*S;
}

/**
 * @brief Slack of the soft constraint that is active at working set index i
 */
c_float DaqpSoftSlack(DAQPWorkspace *work, const int i){
    const int Id = work->WS[i];
    if(DAQP_IS_SLACK_FIXED(Id)) return 0;
    return DaqpSoftResidual(work,Id,work->lam_star[i]);
}

void DaqpRemoveConstraint(DAQPWorkspace* work, const int rm_ind){
    int i;
    // Update data structures
    DAQP_SET_INACTIVE(work->WS[rm_ind]);
    DaqpUpdateLDLRemove(work,rm_ind);
    (work->n_active)--;

    for(i=rm_ind;i<work->n_active;i++){
        work->WS[i] = work->WS[i+1];
        work->lam[i] = work->lam[i+1];
    }
    // Can only reuse work less than the ind that was removed
    if(rm_ind < work->reuse_ind)
        work->reuse_ind = rm_ind;

    // Check if the removal lead to singularity (can happen due to numerics).
    if(work->n_active > 0 && work->D[work->n_active-1] < work->settings->sing_tol)
        work->sing_ind = work->n_active-1;
    else{ // Pivot for improved numerics
        DaqpPivotLast(work);
    }
}
/**
 * @brief Add a constraint, keeping the slack state that is marked in sense
 */
static void DaqpAddConstraintKeepSlack(DAQPWorkspace *work,
        const int add_ind, c_float lam){
    // Update data structures
    DAQP_SET_ACTIVE(add_ind);
    const c_float rho = DAQP_IS_SOFT(add_ind) && DAQP_IS_SLACK_FREE(add_ind)
        ? DaqpSoftRho(work,add_ind) : 0;
    DaqpUpdateLDLAdd(work,add_ind,rho);
    work->WS[work->n_active] = add_ind;
    work->lam[work->n_active] = lam;
    work->n_active++;

    // Pivot for improved numerics
    DaqpPivotLast(work);
}

void DaqpAddConstraint(DAQPWorkspace *work, const int add_ind, c_float lam){
    work->sense[add_ind] &= ~DAQP_SLACK_SWITCHED;
    // Mark whether the slack is zero, given the multiplier
    if(DAQP_IS_SOFT(add_ind)){
        DAQP_SET_MUTABLE(add_ind);
        const c_float w = DaqpSoftW(work,add_ind);
        if(w > 0){
            const c_float magnitude = (DAQP_IS_LOWER(add_ind) ? -lam : lam) >= w ? w : 0;
            lam = DAQP_IS_LOWER(add_ind) ? -magnitude : magnitude;
        }
        if(w > 0 && (DAQP_IS_LOWER(add_ind) ? -lam : lam) < w)
            DAQP_SET_SLACK_FIXED(add_ind);
        else
            DAQP_SET_SLACK_FREE(add_ind);
    }
    DaqpAddConstraintKeepSlack(work,add_ind,lam);
}

void DaqpComputePrimalAndFval(DAQPWorkspace *work){
    int i,j,disp,Id;
    c_float fval=0;
    const int has_l1 = DAQP_HAS_L1(work);
    // Reset u
    for(j=0;j<work->n;j++)
        work->u[j]=0;
    //u[m] <-- Mk'*lam_star (zero if empty set)
    for(i=0;i<work->n_active;i++){
        Id = work->WS[i];
        const c_float li = work->lam_star[i]; // hoist: invariant across the inner loop
        if(Id < work->ms){
            // Simple constraint
            if(work->Rinv!=NULL){ // Hessian is not identity
                for(j=Id, disp=DAQP_R_OFFSET(Id,work->n);j<work->n;++j)
                    work->u[j]-=work->Rinv[disp+j]*li;
            }
            else work->u[Id]-=li; // Hessian is identity
        }
        else{ // General constraint
            for(j=0,disp=work->n*(Id-work->ms);j<work->n;j++)
                work->u[j]-=work->M[disp++]*li;
        }
        if(DAQP_IS_SOFT(Id)){
            fval += has_l1 ? DaqpSoftPenalty(work,Id,li)
                : work->settings->rho_soft*li*li;
            work->sense[Id] &= ~DAQP_SLACK_SWITCHED;
        }
    }
    for(j=0;j<work->n;j++)
        fval+=work->u[j]*work->u[j];
    work->fval = fval;
}
int DaqpAddInfeasible(DAQPWorkspace *work){
    int j,disp;
    c_float ep = -work->settings->primal_tol;
    c_float min_val = 0.0;
    c_float bound;
    c_float Mu,min_cand;
    int isupper=0, add_ind=DAQP_EMPTY_IND;
    const c_float noise = DaqpNoiseFloor(work); // 0 unless cycling persisted
    // Simple bounds
    for(j=0, disp=0;j<work->ms;j++){
        // Never activate immutable or already active constraints
        if(work->sense[j]&(DAQP_ACTIVE+DAQP_IMMUTABLE)){
            disp+=work->n-j;
            continue;
        }
        if(work->Rinv==NULL){// Hessian is identify
            Mu=work->u[j];
        }
        else{
            Mu = DaqpDotInline(work->Rinv+disp,work->u+j,work->n-j);
        }
        disp+=work->n-j;
        bound = (work->scaling == NULL) ? ep : ep*work->scaling[j];
        if(bound > noise) bound = noise;
        min_cand = work->dupper[j]-Mu;
        if(min_cand < min_val && min_cand < bound){
            add_ind = j; isupper = 1;
            min_val = min_cand;
        }
        else{
            min_cand = Mu - work->dlower[j];
            if(min_cand < min_val && min_cand < bound){
                add_ind = j; isupper = 0;
                min_val = min_cand;
            }
        }
    }
    /* General two-sided constraints */
    DaqpComputeMu(work);
    for(j=work->ms, disp=0;j<work->m;j++){
        // Never activate immutable or already active constraints
        if(work->sense[j]&(DAQP_ACTIVE+DAQP_IMMUTABLE)){
            disp+=work->n;// Skip ahead in M
            continue;
        }
        Mu = work->Mu == NULL
            ? DaqpDotInline(work->M+disp,work->u,work->n)
            : work->Mu[j-work->ms];
        disp+=work->n;
        bound = (work->scaling == NULL) ? ep : ep*work->scaling[j];
        if(bound > noise) bound = noise;

        min_cand = work->dupper[j]-Mu;
        if(min_cand < min_val &&  min_cand < bound){
            add_ind = j; isupper = 1;
            min_val = min_cand;
        }
        else{
            min_cand = Mu - work->dlower[j];
            if(min_cand < min_val && min_cand < bound){
                add_ind = j; isupper = 0;
                min_val = min_cand;
            }
        }
    }
    // No constraint is infeasible => return
    if(add_ind == DAQP_EMPTY_IND) return 0;
    // Otherwise add infeasible constraint to working set
    if(isupper)
        DAQP_SET_UPPER(add_ind);
    else
        DAQP_SET_LOWER(add_ind);
/**
 * @brief Set lam = lam_star
 */
    c_float *swp_ptr;
    swp_ptr=work->lam; work->lam = work->lam_star; work->lam_star=swp_ptr;
    DaqpAddConstraint(work,add_ind,isupper ? -min_val : min_val);
    return 1;
}

/**
 * @brief Compute all general constraint products from the existing row-major M. Four rows are
 *        accumulated together to reuse u and expose independent accumulation chains. A NULL
 *        Mu buffer keeps the row-wise fallback active.
 */
void DaqpComputeMu(DAQPWorkspace *work){
    if(work->Mu == NULL) return;

    const int rows = work->m-work->ms;
    const int n = work->n;
    int row = 0;
    for(; row+3 < rows; row+=4){
        const c_float *m0 = work->M+(row+0)*n;
        const c_float *m1 = work->M+(row+1)*n;
        const c_float *m2 = work->M+(row+2)*n;
        const c_float *m3 = work->M+(row+3)*n;
        c_float sum0=0, sum1=0, sum2=0, sum3=0;
        for(int k=0; k<n; k++){
            const c_float uk = work->u[k];
            sum0 += m0[k]*uk;
            sum1 += m1[k]*uk;
            sum2 += m2[k]*uk;
            sum3 += m3[k]*uk;
        }
        work->Mu[row+0] = sum0;
        work->Mu[row+1] = sum1;
        work->Mu[row+2] = sum2;
        work->Mu[row+3] = sum3;
    }
    for(; row<rows; row++)
        work->Mu[row] = DaqpDotInline(work->M+row*n,work->u,n);
}
/**
 * @brief Lower bound sum rho*p_i^2 (free soft slacks) on the dual curvature along the
 *        singular direction p = lam_star; zero if the slacks only carry rounding
 */
static c_float DaqpSoftCurvature(DAQPWorkspace *work){
    int i;
    c_float c = 0, ps = 0, pp = 0;
    for(i = 0; i <= work->sing_ind; i++){
        const int Id = work->WS[i];
        const c_float p2 = work->lam_star[i]*work->lam_star[i];
        pp += p2;
        if(DAQP_IS_SOFT(Id) && DAQP_IS_SLACK_FREE(Id)){
            c += DaqpSoftRho(work,Id)*p2;
            ps += p2;
        }
    }
    return ps > DAQP_SOFT_CURV_TOL*pp ? c : 0;
}

/**
 * @brief Step lam toward lam_star (along lam_star if singular) until a multiplier reaches
 *        zero (removal) or passes w (slack switch). Returns 0 if full step
 */
int DaqpRemoveBlocking(DAQPWorkspace *work){
    int i, ind, rm_ind = DAQP_EMPTY_IND;
    const int singular = work->sing_ind != DAQP_EMPTY_IND;
    const int has_l1 = DAQP_HAS_L1(work);
    c_float alpha = DAQP_INF, alpha_cand, y, ystar, p, target, rm_target = 0;
    // Blocking beyond dual_tol, or beyond zero_tol for a singular direction (which
    // is not a multiplier, but scaled to 1 for the constraint that made it singular)
    const c_float tol = singular ? work->settings->zero_tol : work->settings->dual_tol;

    for(i = 0; i < work->n_active; i++){
        ind = work->WS[i];
        if(DAQP_IS_IMMUTABLE(ind)) continue;
        // Fold the sign of the multiplier (dual feasibility <=> y >= 0).
        // ystar is the end point of the step, or its direction if singular.
        const int lower = DAQP_IS_LOWER(ind);
        ystar = lower ? -work->lam_star[i] : work->lam_star[i];

        if(!has_l1 || !DAQP_IS_SOFT(ind)){ // Blocked when the multiplier reaches zero
            if(ystar >= -tol) continue;
            target = 0;
        }
        else{
            // The multiplier is confined to [0,w] while the slack is zero and
            // to [w,inf) otherwise; the state switches when it leaves its range
            const c_float w = DaqpSoftW(work,ind);
            const int fixed = DAQP_IS_SLACK_FIXED(ind);
            target = fixed ? 0 : w; // Blocked from below
            if(ystar >= (singular ? 0 : target) - tol){
                if(!fixed || w == 0 || ystar <= (singular ? 0 : w) + tol)
                    continue;
                target = w; // A zero slack is released
            }
            // Switching back at w without a primal step is rounding
            if(!singular && w > 0 && target == w && (work->sense[ind] & DAQP_SLACK_SWITCHED) &&
                    (lower ? -work->lam[i] : work->lam[i]) == w)
                continue;
        }

        y = lower ? -work->lam[i] : work->lam[i];
        p = singular ? ystar : ystar-y;
        alpha_cand = (target-y)/p;
        if(target != 0 && alpha_cand < 0) alpha_cand = 0;
        if(alpha_cand < alpha){
            alpha = alpha_cand;
            rm_ind = i;
            rm_target = target;
        }
    }
    if(rm_ind == DAQP_EMPTY_IND){
        if(singular){
            const c_float c = DaqpSoftCurvature(work);
            if(c > 0){ // Blocked by the slacks
                if(work->D[work->sing_ind] < c) work->D[work->sing_ind] = c;
                if(work->reuse_ind > work->sing_ind) work->reuse_ind = work->sing_ind;
                work->sing_ind = DAQP_EMPTY_IND;
                return 1;
            }
        }
        return 0; // Either dual feasible or primal infeasible
    }

    // A zero-length transition cannot make progress when the CSP is singular:
    // the zero slack is what makes the working set rank deficient
    if(singular && alpha <= 0) rm_target = 0;

    // Update lambda
    if(singular)
        for(i = 0; i < work->n_active; i++)
            work->lam[i] += alpha*work->lam_star[i];
    else
        for(i = 0; i < work->n_active; i++)
            work->lam[i] += alpha*(work->lam_star[i]-work->lam[i]);

    work->sing_ind = DAQP_EMPTY_IND;
    ind = work->WS[rm_ind];
    if(rm_target == 0){ // The constraint leaves the working set
        DaqpRemoveConstraint(work,rm_ind);
        // Earlier slack switches are now genuine
        if(has_l1)
            for(i = 0; i < work->n_active; i++)
                work->sense[work->WS[i]] &= ~DAQP_SLACK_SWITCHED;
        return 1;
    }

    // The slack switches state, which only adds or removes rho on the diagonal
    const c_float lam = DAQP_IS_LOWER(ind) ? -rm_target : rm_target;
    const int release = DAQP_IS_SLACK_FIXED(ind);
    if(release) DAQP_SET_SLACK_FREE(ind);
    else DAQP_SET_SLACK_FIXED(ind);
    work->sense[ind] |= DAQP_SLACK_SWITCHED;

    // Nothing in the factorization depends on the diagonal of the last row,
    // so a slack there can switch without forming its row of M*M' again
    if(rm_ind == work->n_active-1 && !singular){
        const c_float rho = DaqpSoftRho(work,ind);
        work->D[rm_ind] += release ? rho : -rho;
        work->lam[rm_ind] = lam;
        // The shift in this row'S RHS changed, so it has to be recomputed
        if(work->reuse_ind > rm_ind) work->reuse_ind = rm_ind;
/**
 * @brief Singularity as in DaqpUpdateLDLAdd: a free slack adds rho to the diagonal, and so
 *        relaxes the rank condition
 */
        int ns_active = 0;
        for(i = 0; i < work->n_active; i++)
            if(DAQP_IS_SOFT(work->WS[i]) && DAQP_IS_SLACK_FREE(work->WS[i]))
                ns_active++;
        if(work->D[rm_ind] < work->settings->sing_tol ||
                rm_ind >= work->n + ns_active)
            work->sing_ind = rm_ind;
        else
            DaqpPivotLast(work); // The new diagonal may be a worse pivot
    }
    else{
        DaqpRemoveConstraint(work,rm_ind);
        if(work->sing_ind == DAQP_EMPTY_IND)
            DaqpAddConstraintKeepSlack(work,ind,lam);
    }
    return 1;
}

void DaqpComputeCSP(DAQPWorkspace *work){
    int i,j,disp,start_disp;
    c_float sum;
    const int has_l1 = DAQP_HAS_L1(work);
    // Forward substitution (xi <-- L\d)
    for(i=work->reuse_ind,disp=DAQP_ARSUM(work->reuse_ind); i<work->n_active; i++){
        // Setup RHS
        const int Id = work->WS[i];
        sum = DAQP_IS_LOWER(Id) ? -work->dlower[Id] : -work->dupper[Id];
        // Linear weight of a nonzero slack
        if(has_l1 && DAQP_IS_SOFT(Id) && DAQP_IS_SLACK_FREE(Id))
            sum -= DaqpSoftResidual(work,Id,0);
        for(j=0; j<i; j++)
            sum -= work->L[disp++]*work->xldl[j];
        disp++; //Skip 1 in L
        work->xldl[i] = sum;
    }
    // Scale with D  (zi = xi/di)
    for(i=work->reuse_ind; i<work->n_active; i++)
        work->zldl[i] = work->xldl[i]/work->D[i];
    //Backward substitution  (lam_star <-- L'\z)
    start_disp = DAQP_ARSUM(work->n_active)-1;
    for(i = work->n_active-1;i>=0;i--){
        sum=work->zldl[i];
        disp = start_disp--;
        for(j=work->n_active-1;j>i;j--){
            sum-=work->lam_star[j]*work->L[disp];
            disp-=j;
        }
        work->lam_star[i] = sum;
    }
    work->reuse_ind = work->n_active; // Save forward substitution information
}

/**
 * @brief TODO this could probably be directly calculated in L
 */
void DaqpComputeSingularDirection(DAQPWorkspace *work){
/**
 * @brief Step direction is stored in lam_star
 */
    int i,j,disp,offset_L= DAQP_ARSUM(work->sing_ind);
    int start_disp= offset_L-1;

    // Backwards substitution (p_tidle <-- L'\(-l))
    for(i = work->sing_ind-1;i>=0;i--){
        work->lam_star[i] = -work->L[offset_L+i];
        disp = start_disp--;
        for(j=work->sing_ind-1;j>i;j--){
            work->lam_star[i]-=work->lam_star[j]*work->L[disp];
            disp-=j;
        }
    }
    work->lam_star[work->sing_ind]=1;

    // Orient the direction such that is is a descent direction
    const int Id = work->WS[work->sing_ind];
    const int lower = DAQP_IS_LOWER(Id);
    int flip = lower;
    if(DAQP_IS_SOFT(Id) && DAQP_IS_SLACK_FIXED(Id)){
        const c_float w = DaqpSoftW(work,Id);
        const c_float y = lower ? -work->lam[work->sing_ind]
                                :  work->lam[work->sing_ind];
        if(w > 0 && y >= w-work->settings->dual_tol)
            flip = !lower;
    }
    if(flip)
        for(i=0;i<=work->sing_ind;i++)
            work->lam_star[i] =-work->lam_star[i];
}


/**
 * @brief Whether -d_W'lam exceeds fval = ||u||^2 (its value at a solution) by more than fval
 *        and the active residuals allowed by primal_tol
 */
int DaqpInconsistentDual(DAQPWorkspace *work){
    int i, Id;
    c_float dl = 0, tol = 0, l;
    for(i = 0; i < work->n_active; i++){
        Id = work->WS[i];
        if(DAQP_IS_SOFT(Id)) return 0;
        l = work->lam_star[i];
        dl += l*(DAQP_IS_LOWER(Id) ? work->dlower[Id] : work->dupper[Id]);
        tol += (l < 0 ? -l : l)*(work->scaling != NULL ? work->scaling[Id] : 1);
    }
    return -dl > 2*work->fval + work->settings->primal_tol*tol;
}

void DaqpPivotLast(DAQPWorkspace *work){
    const int rm_ind = work->n_active-2;
    if(work->n_active > 1 && work->sing_ind == DAQP_EMPTY_IND &&
            work->D[rm_ind] < work->settings->pivot_tol && // element in D small enough
            work->D[rm_ind] < work->D[work->n_active-1]){ // element in D smallar than neighbor
        const int ind_old = work->WS[rm_ind];
        // Ensure that binaries never swap order (since this order is exploited)
        if(DAQP_IS_BINARY(ind_old) && DAQP_IS_BINARY(work->WS[work->n_active-1])) return;
        if(work->bnb != NULL && rm_ind < work->bnb->n_clean) return;

        c_float lam_old = work->lam[rm_ind];
        DaqpRemoveConstraint(work,rm_ind); // pivot_last might be recursively called here

        if(work->sing_ind!=DAQP_EMPTY_IND) return; // Abort if D becomes singular

        // Reordering only: the slack state of the constraint is unchanged
        DaqpAddConstraintKeepSlack(work,ind_old,lam_old);
    }
}

/**
 * @brief Add constraint Id to the working set, with a multiplier that reproduces the slack
 *        state that is marked in sense
 */
static void DaqpActivateConstraint(DAQPWorkspace *work, const int Id){
    c_float lam = 1.0;
    const c_float w = DAQP_IS_SOFT(Id) ? DaqpSoftW(work,Id) : 0;
    if(w > 0) lam = DAQP_IS_SLACK_FREE(Id) ? w+1 : 0.9*w;
    DaqpAddConstraint(work,Id, DAQP_IS_LOWER(Id) ? -lam : lam);
}

/**
 * @brief Remove the last constraint of a singular working set (not necessarily the latest
 *        added one, due to pivoting). Equalities become mutable (re-added if violated)
 */
int DaqpDropSingularLast(DAQPWorkspace *work){
    const int Id = work->WS[--work->n_active];
    DAQP_SET_INACTIVE(Id);
    if(!DAQP_IS_BINARY(Id)) work->sense[Id] &= ~DAQP_IMMUTABLE;
    work->sing_ind = DAQP_EMPTY_IND;
    if(work->reuse_ind > work->n_active)
        work->reuse_ind = work->n_active;
    return Id;
}

/**
 * @brief Activate the constraints that are marked active in sense Equalities are activated
 *        before inequalities
 */
int DaqpActivateConstraints(DAQPWorkspace *work){
    int i, j, first_mutable = work->m, exitflag = 1;
    for(i = 0; i < work->m; i++){
        if(!DAQP_IS_ACTIVE(i)) continue;
        if(!DAQP_IS_IMMUTABLE(i)){
            if(i < first_mutable) first_mutable = i;
            continue;
        }
        DaqpActivateConstraint(work,i);
        if(work->sing_ind != DAQP_EMPTY_IND){
            c_float dependency_residual = 0.0;
            c_float dependency_scale = 1.0;

            //  The new equality is linearly dependent on the active equalities
            DaqpComputeSingularDirection(work);
            for(j = 0; j < work->n_active; j++){
                int Id = work->WS[j];
                c_float bound = DAQP_IS_LOWER(Id)
                    ? work->dlower[Id] : work->dupper[Id];
                c_float term = work->lam_star[j] * bound;
                dependency_residual += term;
                dependency_scale += term < 0 ? -term : term;
            }

            // Dependency might only be numerical => keep as mutable constraint
            DaqpDropSingularLast(work);
            if(dependency_residual >
                        work->settings->primal_tol*dependency_scale ||
                    dependency_residual <
                        -work->settings->primal_tol*dependency_scale)
                exitflag = DAQP_EXIT_OVERDETERMINED_INITIAL;
        }
    }

    // Activate active inequalities
    for(i = first_mutable; i < work->m; i++){
        if(!DAQP_IS_ACTIVE(i) || DAQP_IS_IMMUTABLE(i)) continue;
        DaqpActivateConstraint(work,i);
        if(work->sing_ind != DAQP_EMPTY_IND){
            // Drop dependent constraint and leave remaining mutable ones inactive
            DaqpDropSingularLast(work);
            for(j = i+1; j < work->m; j++){
                if(!DAQP_IS_IMMUTABLE(j)) DAQP_SET_INACTIVE(j);
            }
            return exitflag;
        }
    }
    return exitflag;
}

/**
 * @brief Deactivate all active constraints that are mutable (i.e., not equality constraints)
 */
void DaqpDeactivateConstraints(DAQPWorkspace *work){
    int i;
    if(work->bnb != NULL) work->bnb->n_root_WS = 0; // Also drop the BnB warm start
    for(i =0;i<work->n_active;i++){
        if(DAQP_IS_IMMUTABLE(work->WS[i])) continue;
        DAQP_SET_INACTIVE(work->WS[i]);
    }
    ResetDaqpWorkspace(work); // The next update activates the remaining ones
}

/**
 * @brief Solve L*D*L'*dlam = r (r and dlam in xldl, zldl used as scratch)
 */
void DaqpSolveWorkingSet(DAQPWorkspace *work){
    int i, j, disp;
    const int na = work->n_active;
    c_float sum;
    // Forward substitution L*y = r
    for(i = 0, disp = 0; i < na; i++){
        sum = work->xldl[i];
        for(j = 0; j < i; j++)
            sum -= work->L[disp++]*work->xldl[j];
        disp++; // Skip the stored diagonal (= 1)
        work->xldl[i] = sum;
    }
    // Scale by D^{-1}
    for(i = 0; i < na; i++)
        work->zldl[i] = work->xldl[i]/work->D[i];
    // Backward substitution L'*dlam = zldl
    {
        int start_disp = DAQP_ARSUM(na)-1;
        for(i = na-1; i >= 0; i--){
            sum = work->zldl[i];
            disp = start_disp--;
            for(j = na-1; j > i; j--){
                sum -= work->xldl[j]*work->L[disp];
                disp -= j;
            }
            work->xldl[i] = sum;
        }
    }
}

/**
 * @brief y <-- y - M_W'*dlam for the working set W
 */
void DaqpSubWorkingSetRows(DAQPWorkspace *work, const c_float* dlam, c_float* y){
    int i, j, disp, Id;
    for(i = 0; i < work->n_active; i++){
        const c_float dl = dlam[i];
        Id = work->WS[i];
        if(Id < work->ms){
            if(work->Rinv != NULL){
                for(j = Id, disp = DAQP_R_OFFSET(Id,work->n); j < work->n; j++)
                    y[j] -= work->Rinv[disp+j]*dl;
            }
            else y[Id] -= dl;
        }
        else
            for(j = 0, disp = work->n*(Id-work->ms); j < work->n; j++)
                y[j] -= work->M[disp++]*dl;
    }
}

/**
 * @brief One step of iterative refinement for active constraints. After computing u =
 *        -M'*lam_star, numerical errors in the LDL solve cause active constraint residuals
 *        r[i] = M_i*u - d_i to be nonzero. These errors are amplified by 1/scaling[j] in the
 *        original space, potentially exceeding primal_tol for near-singular factorizations
 *        with small scaling values. This function solves (L*D*L') * delta_lam = r using the
 *        existing factorization and updates u -= M'*delta_lam to cancel the residual exactly:
 *        M*(u - M'*delta_lam) = M*u - M*M'*delta_lam = M*u - r = d.
 */
void DaqpRefineActive(DAQPWorkspace *work){
    int i, j, disp, Id;
    c_float Mu, d;

    // Refinement uses xldl and zldl as scratch, invalidating the cached CSP
    // forward substitution independently of whether the active set changes.
    work->reuse_ind = 0;

    // Compute r[i] = M_i*u - d_i and store in xldl[i].
    for(i = 0; i < work->n_active; i++){
        Id = work->WS[i];
        if(Id < work->ms){
            if(work->Rinv != NULL){
                Mu = 0;
                for(j=Id, disp=DAQP_R_OFFSET(Id,work->n); j<work->n; j++)
                    Mu += work->Rinv[disp+j] * work->u[j];
            } else {
                Mu = work->u[Id];
            }
        } else {
            Mu = 0;
            for(j=0, disp=work->n*(Id-work->ms); j<work->n; j++)
                Mu += work->M[disp++] * work->u[j];
        }
        d = DAQP_IS_LOWER(Id) ? work->dlower[Id] : work->dupper[Id];
        work->xldl[i] = Mu - d; // RHS: +r[i] (positive, so L*D*L'*dlam=r gives u-=M'*dlam zeroes residual)
        // For a nonzero soft slack the CSP system has a diagonal
        // reciprocal-weight term. Account for it (and for the linear weight)
        // when forming the refinement residual.
        if(DAQP_IS_SOFT(Id) && DAQP_IS_SLACK_FREE(Id))
            work->xldl[i] -= DaqpSoftResidual(work,Id,work->lam_star[i]);
    }

    DaqpSolveWorkingSet(work); // xldl = delta_lam

    // Update lam_star += delta_lam and u -= M'*delta_lam.
    for(i=0; i<work->n_active; i++)
        work->lam_star[i] += work->xldl[i];
    DaqpSubWorkingSetRows(work,work->xldl,work->u);

/**
 * @brief Recompute fval since both u and lam_star changed
 */
    c_float fval = 0;
    for(i=0; i<work->n_active; i++){
        Id = work->WS[i];
        if(DAQP_IS_SOFT(Id)) fval += DaqpSoftPenalty(work,Id,work->lam_star[i]);
    }
    for(j=0; j<work->n; j++)
        fval += work->u[j] * work->u[j];
    work->fval = fval;
}

/**
 * @brief Residual of active constraint Id at x: A_id x - b_id (b the active side)
 */
static c_float DaqpActiveResidual(const DAQPWorkspace *work, const int Id){
    const DAQPProblem* qp = work->qp;
    const int n = work->n, ms = work->ms;
    c_float val;
    int j;
    if(Id < ms) val = work->x[Id];
    else{
        const c_float* a = qp->A+(size_t)(Id-ms)*n;
        for(j = 0, val = 0; j < n; j++) val += a[j]*work->x[j];
    }
    return val - (DAQP_IS_LOWER(Id) ? qp->blower[Id] : qp->bupper[Id]);
}

/**
 * @brief y <-- Rinv*y (as Ldp2qpSolution, without v)
 */
void DaqpApplyRinv(DAQPWorkspace *work, c_float* y){
    int i, j, disp;
    const int n = work->n;
    if(work->Rinv != NULL){
        for(i = 0, disp = 0; i < n; i++){
            y[i] *= work->Rinv[disp++];
            for(j = i+1; j < n; j++) y[i] += work->Rinv[disp++]*y[j];
        }
        if(work->scaling != NULL)
            for(i = 0; i < work->ms; i++) y[i] /= work->scaling[i];
    }
    else if(work->RinvD != NULL)
        for(i = 0; i < n; i++) y[i] *= work->RinvD[i];
}

/**
 * @brief One step of iterative refinement of x on the active constraints, done directly in x
 *        to avoid the cancellation in x = Rinv*(u-v)
 */
void DaqpRefinePrimal(DAQPWorkspace *work){
    int i, j, Id;
    const int n = work->n, na = work->n_active;
    const DAQPProblem* qp = work->qp;
    c_float *r = work->xldl, *du = work->zldl, dfval = 0;
    if(na == 0 || qp == NULL || work->sing_ind != DAQP_EMPTY_IND) return;

    // Skip if x is accurate: a well-conditioned (unregularized) Hessian and rounding
    // errors (about DAQP_REFINE_GAIN*eps*max(|u|,|v|)/min(D), with |u|^2 = fval >= |d_W|^2)
    // below primal_tol
    if(!(work->state & DAQP_STATE_ILL_CONDITIONED) && work->n_prox == 0){
        c_float scale2 = work->fval, min_D = 1, err;
        for(i = 0; i < na; i++) if(work->D[i] < min_D) min_D = work->D[i];
        if(work->v != NULL)
            for(i = 0; i < n; i++) if(work->v[i]*work->v[i] > scale2) scale2 = work->v[i]*work->v[i];
        err = DAQP_REFINE_GAIN*(sizeof(c_float) == sizeof(float) ? FLT_EPSILON : DBL_EPSILON);
        if(err*err*scale2 <= work->settings->primal_tol*work->settings->primal_tol*min_D*min_D) return;
    }
    for(i = 0; i < na; i++) if(DAQP_IS_SOFT(work->WS[i])) return;

    // xldl and zldl are used as scratch
    work->reuse_ind = 0;

    // r = S*(A_W x - b_W)
    for(i = 0; i < na; i++){
        Id = work->WS[i];
        const c_float val = DaqpActiveResidual(work,Id);
        dfval += work->lam_star[i]*val;
        r[i] = val*(work->scaling != NULL ? work->scaling[Id] : 1);
    }

    DaqpSolveWorkingSet(work); // r = dlam

    // du = -M_W'*dlam
    for(j = 0; j < n; j++) du[j] = 0;
    DaqpSubWorkingSetRows(work,r,du);
    for(j = 0; j < n; j++) dfval += 0.5*du[j]*du[j];

    DaqpApplyRinv(work,du); // dx
    for(i = 0; i < n; i++) work->x[i] += du[i];

    for(i = 0; i < na; i++)
        work->lam_star[i] += r[i]*(work->scaling != NULL ? work->scaling[work->WS[i]] : 1);
    work->fval += 2*dfval; // fval is twice the objective function value
}
}  // namespace miqp
}  // namespace math
}  // namespace common
}  // namespace autonomy
