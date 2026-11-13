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
 * @file hierarchical.cpp
 * @brief Lexicographic hierarchical quadratic program.
 *
 * Declarations and the algorithm contract live in the matching header.
 * This file holds the definitions of the vendored DAQP C solver.
 */

#include "autonomy/common/math/miqp/hierarchical.hpp"
#include "autonomy/common/math/miqp/types.hpp"
#include "autonomy/common/math/miqp/auxiliary.hpp"
#include <stdlib.h>


namespace autonomy {
namespace common {
namespace math {
namespace miqp {
int DaqpHiqp(DAQPWorkspace *work, c_float *lambda){
    int i,j,Id;
    int start,end;
    int iterations = 0;
    int exitflag=0;

    // If only one hiearchy -> just solve normal LDP
    if(!DAQP_IS_HIERARCHICAL(work)) return DaqpLdp(work);

    // A previous solve shifted d by the slacks of the soft levels, so it has
    // to be reformed by an update first (without a qp, the caller resets d)
    if((work->state & DAQP_UPDATE_d) && work->qp != NULL) return DAQP_EXIT_UNSUPPORTED;

    // Reset lambda for output
    if(lambda != NULL) for(i=0;i<work->m;i++) lambda[i]=0;

/**
 * @brief Start moving down the hierarchy
 */
    c_float w;
    start=work->break_points[0];
    // A previous solve leaves constraints of the soft levels in the working
    // set; restart from the (hard) first level, whose active set is kept.
    for(i = 0; i < work->n_active; i++) if(work->WS[i] >= start) break;
    if(i < work->n_active){
        work->m = start;
        ResetDaqpWorkspace(work);
        exitflag = DaqpActivateConstraints(work);
        if(exitflag < 0) return exitflag;
        exitflag = 0;
    }
    int nfree = work->n;
    for(i =1; i < work->nh; i++){
        // initialize current level
        end=work->break_points[i];
        work->m = end;
        // Soften constraints and activate
        for(j =start;j<end;j++){
            DAQP_SET_SOFT(j);
            if(DAQP_IS_ACTIVE(j)){
                if(DAQP_IS_LOWER(j))
                    DaqpAddConstraint(work,j, -1.0);
                else
                    DaqpAddConstraint(work,j, 1.0);
                if(work->sing_ind != DAQP_EMPTY_IND){
                    // Dependent constraint (e.g., from a warm start): leave it
                    // out, and make it mutable so that it is not ignored
                    DAQP_SET_INACTIVE(j);
                    DAQP_SET_MUTABLE(j);
                    work->n_active--;
                    work->sing_ind = DAQP_EMPTY_IND;
                    if(work->reuse_ind > work->n_active) work->reuse_ind = work->n_active;
                }
            }
        }

        // Solve best solution in case DaqpLdp fails
        for(j = 0; j<work->n;j++) work->xold[j] = work->x[j];
        // Solve LDP
        exitflag = DaqpLdp(work);
        iterations+=work->iterations;
        if(exitflag < 0) break;

        if(iterations >= work->settings->iter_limit){
            exitflag = DAQP_EXIT_ITERLIMIT;
            break;
        }

        // Perturb rhs with slacks in level
        for(j=0; j<work->n_active;j++){
            Id=work->WS[j];
            if(DAQP_IS_SOFT(Id)){
                w = DaqpSoftSlack(work,j);
                if(w < -work->settings->primal_tol)
                    work->dlower[Id]+=w;
                else if(w > work->settings->primal_tol)
                    work->dupper[Id]+=w;
                if(lambda != NULL){
                    w += DAQP_IS_LOWER(Id) ? -1e-14 : 1e-14; // For weakly active
                    lambda[Id] = w;
                }

            }
        }

        // Make constraints in current level hard
        for(j=start; j<end;j++) DAQP_SET_HARD(j);

        if(i==work->nh-1) break;
        
        // find first active constraint in current level 
        for(j=0;j < work->n_active; j++) if(work->WS[j]>=start) break;

/**
 * @brief reactive constraint in level current (to addresss soft->hard) TODO: can
 *        factorization be directly reused?
 */
        int n_active_old = (work->n_active < work->n) ? work->n_active : work->n;
        for(int jj=n_active_old; jj < work->n_active ;jj++) {
            work->sense[work->WS[jj]]&=~(DAQP_ACTIVE+DAQP_IMMUTABLE);
        }
        work->n_active =j;
        work->reuse_ind=j;
        work->sing_ind = DAQP_EMPTY_IND;
        for(; j<n_active_old ;j++){
            DaqpAddConstraint(work,work->WS[j],work->lam_star[j]);
            // Skip if WS becomes overdtermined
            if(work->sing_ind != DAQP_EMPTY_IND){
                DaqpRemoveConstraint(work,j);
                work->sing_ind = DAQP_EMPTY_IND;
                DAQP_SET_MUTABLE(work->WS[j]);
            }
            else{
                if(DAQP_IS_IMMUTABLE(work->WS[j])) nfree--;
            }
        }

        if(nfree <= 0 ) break;  // No degrees of freedom left
        // Move up hierarchy
        start = end;
    }
    // Finalize
    if(exitflag < 0){ // Restore a point that was good before it failed
        for(j = 0; j<work->n;j++) work->x[j] = work->xold[j];
        exitflag = 3; // signify no degrees of freedoom left
    }
    work->iterations = iterations; // Append total number of iterations
    work->state |= DAQP_UPDATE_d; // The levels have shifted d
    return exitflag;
}
}  // namespace miqp
}  // namespace math
}  // namespace common
}  // namespace autonomy
