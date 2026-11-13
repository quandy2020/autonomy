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
 * @file bnb.cpp
 * @brief Branch-and-bound over binary simple bounds.
 *
 * Declarations and the algorithm contract live in the matching header.
 * This file holds the definitions of the vendored DAQP C solver.
 */

#include "autonomy/common/math/miqp/bnb.hpp"
#ifdef PROFILING
#include "autonomy/common/math/miqp/utils.hpp"

namespace autonomy {
namespace common {
namespace math {
namespace miqp {
#endif

/**
 * @brief Signed distance of binary @p Id from the nearer bound.
 *
 * @param[in] Id Constraint index.
 * @param[in] work Workspace of a solved relaxation.
 * @return Positive when the upper bound is closer, negative when the lower
 *         bound is closer.
 */
static c_float DaqpBinaryDiff(const int Id, DAQPWorkspace* work){
    int j, disp;
    c_float diff = 0.5*(work->dupper[Id]+work->dlower[Id]);

    if(Id < work->ms){//Simple bound
        if(work->Rinv==NULL) diff -= work->u[Id]; //Hessian is identity
        else{
            for(j=Id,disp=Id+DAQP_R_OFFSET(Id,work->n);j<work->n;j++)
                diff -= work->Rinv[disp++]*work->u[j];
        }
    }
    else{//General bound; DaqpAddInfeasible already computed M*u
        diff -= work->Mu[Id-work->ms];
    }
    return diff;
}

/**
 * @brief Store the free part of the working set in ids. Returns the number stored.
 */
static int DaqpBnbStoreWs(int* ids, DAQPWorkspace* work){
    int i, n_ids = 0;
    for(i=work->bnb->neq; i<work->n_active;i++){
        if((work->sense[work->WS[i]]&(DAQP_IMMUTABLE+DAQP_BINARY))!=DAQP_IMMUTABLE+DAQP_BINARY)
            ids[n_ids++] = work->WS[i]+(DAQP_IS_LOWER(work->WS[i]) << (DAQP_LOWER_BIT-1));
    }
    return n_ids;
}

/**
 * @brief Add the constraints in ids to the working set (aborted if the basis gets singular)
 */
static void DaqpBnbLoadWs(const int* ids, const int n_ids, DAQPWorkspace* work){
    int i;
    for(i=0; i < n_ids; i++){
        DaqpAddUpperLower(ids[i],work);
        if(work->sing_ind != DAQP_EMPTY_IND) {
            work->n_active--;
            DAQP_SET_INACTIVE(work->WS[work->n_active]);
            work->sing_ind = DAQP_EMPTY_IND;
            break;
        }
    }
}

/**
 * @brief Add fixed binary Id; a dependent one is left out: returns 1 if added, 0 if
 *        consistent, and -1 if inconsistent (the node is infeasible)
 */
static int DaqpBnbAddFixed(const int Id, DAQPWorkspace* work){
    int j;
    c_float residual = 0, scale = 1, term;
    const c_float tol = work->settings->primal_tol;
    DaqpAddUpperLower(Id,work);
    if(work->sing_ind == DAQP_EMPTY_IND){
        DAQP_SET_IMMUTABLE(DAQP_REMOVE_LOWER_FLAG(Id));
        return 1;
    }
    DaqpComputeSingularDirection(work);
    for(j = 0; j < work->n_active; j++){
        const int k = work->WS[j];
        term = work->lam_star[j]*(DAQP_IS_LOWER(k) ? work->dlower[k] : work->dupper[k]);
        residual += term;
        scale += term < 0 ? -term : term;
    }
    DaqpDropSingularLast(work);
    return (residual > tol*scale || residual < -tol*scale) ? -1 : 0;
}

/**
 * @brief Add the fixed binaries of node (n_clean falls back to neq if one is left out).
 *        Returns -1 if the node is infeasible
 */
static int DaqpBnbAddFixedIds(DAQPNode* node, DAQPWorkspace* work){
    int i, flag, dropped = 0;
    for(i=work->bnb->n_clean - work->bnb->neq; i< node->depth+1;i++){
        flag = DaqpBnbAddFixed(work->bnb->fixed_ids[i],work);
        if(flag < 0){
            work->bnb->n_clean = work->bnb->neq;
            return -1;
        }
        dropped |= flag == 0;
    }
    work->bnb->n_clean = dropped ? work->bnb->neq : work->bnb->neq+node->depth;
    return 0;
}

/**
 * @brief The immutable constraints (e.g., equalities) are kept fixed as a prefix of the
 *        working set throughout the tree. Returns the length of that prefix.
 */
static int DaqpBnbSetupRoot(DAQPWorkspace* work){
    int i, j, error_flag;
    for(i = 0; i < work->n_active; i++)
        if(!DAQP_IS_IMMUTABLE(work->WS[i])) break;
    for(j = i; j < work->n_active; j++)
        if(DAQP_IS_IMMUTABLE(work->WS[j])) break;
    if(j == work->n_active) return i; // Mutable constraints are a warm start

    // Immutable after mutable => only activate immutable constraints
    for(i = 0; i < work->n_active; i++)
        if(!DAQP_IS_IMMUTABLE(work->WS[i])) DAQP_SET_INACTIVE(work->WS[i]);
    ResetDaqpWorkspace(work);
    error_flag = DaqpActivateConstraints(work);
    return error_flag < 0 ? error_flag : work->n_active;
}

/**
 * @brief Use the candidate in work->x as incumbent. Returns its objective (internal scale)
 *        with u = R*x+v stored in work->xold or -1 if infeasible.
 */
static c_float DaqpBnbIncumbent(DAQPWorkspace* work){
    int i, j, disp;
    const int n = work->n;
    c_float val, fval = 0, tol = work->settings->primal_tol;
    c_float *x = work->x, *u = work->xold;
    DAQPProblem* qp = work->qp;
    if(qp == NULL) return -1;

    // Check feasibility (soft constraints are treated as hard)
    for(i=0, disp=0; i < work->m; i++){
        if(i < work->ms) val = x[i];
        else for(j=0, val=0; j < n; j++) val += qp->A[disp++]*x[j];
        if(DAQP_IS_IMMUTABLE(i) && !DAQP_IS_ACTIVE(i) && !DAQP_IS_BINARY(i)) continue; // Ignored
        if(val > qp->bupper[i]+tol || val < qp->blower[i]-tol) return -1;
        if(DAQP_IS_BINARY(i) && val > qp->blower[i]+tol && val < qp->bupper[i]-tol) return -1;
    }

    // Invert Ldp2qpSolution: u = R*x + v
    for(i=0; i < n; i++) u[i] = x[i];
    if(work->Rinv != NULL){
        if(work->scaling != NULL)
            for(i=0; i < work->ms; i++) u[i] *= work->scaling[i];
        for(i=n-1; i >= 0; i--){ // Back substitution with upper triangular Rinv
            disp = i+DAQP_R_OFFSET(i,n);
            for(j=i+1; j < n; j++) u[i] -= work->Rinv[disp+j-i]*u[j];
            u[i] /= work->Rinv[disp];
        }
    }
    else if(work->RinvD != NULL)
        for(i=0; i < n; i++) u[i] /= work->RinvD[i];
    if(work->v != NULL)
        for(i=0; i < n; i++) u[i] += work->v[i];

    for(i=0; i < n; i++) fval += u[i]*u[i];
    return fval;
}

int DaqpBnb(DAQPWorkspace* work){
    int branch_id, exitflag;
    DAQPNode* node;
    c_float *swp_ptr = NULL;

    exitflag = DaqpBnbSetupRoot(work);
    if(exitflag < 0) return exitflag;
    work->bnb->neq = exitflag;

    // Warm start the root with the root working set of the previous solve
    // (unless a warm start has been provided)
    if(work->n_active == work->bnb->neq)
        DaqpBnbLoadWs(work->bnb->root_WS,work->bnb->n_root_WS,work);

/**
 * @brief Modify upper bound based on absolute/relative suboptimality tolerance
 */
    c_float fval_bound0 = work->settings->fval_bound;
    c_float eps_r = 1/(1+work->settings->rel_subopt);
    work->settings->fval_bound = (fval_bound0 - work->settings->abs_subopt)*eps_r;

    // Start from a user-provided integer-feasible solution
    if(work->state & DAQP_STATE_INCUMBENT){
        work->state &= ~DAQP_STATE_INCUMBENT;
        c_float fval_inc = 0.5*DaqpBnbIncumbent(work);
        if(fval_inc >= 0 && fval_inc < fval_bound0){
            work->settings->fval_bound = (fval_inc - work->settings->abs_subopt)*eps_r;
            swp_ptr = work->xold; // Marks that a feasible solution is stored in xold
        }
    }

    work->bnb->itercount=0;
    work->bnb->nodecount=0;
    // Setup root node
    work->bnb->tree[0].depth=-1;
    work->bnb->tree[0].WS_start=0;
    work->bnb->tree[0].WS_end=0;
    work->bnb->tree[0].bin_id=0;
    work->bnb->n_nodes=1;
    work->bnb->n_clean=work->bnb->neq;
    work->bnb->nWS=0;

    exitflag = DAQP_EXIT_INFEASIBLE;
    // Start tree exploration
    while( work->bnb->n_nodes > 0 ){

        node = work->bnb->tree+(--work->bnb->n_nodes);
        exitflag = DaqpProcessNode(node,work); // Solve relaxation
        if(node->depth < 0 && exitflag > 0 && work->bnb->root_WS != NULL)
            work->bnb->n_root_WS = DaqpBnbStoreWs(work->bnb->root_WS,work);
#ifdef PROFILING
        // Individual relaxations are often too short to reach the timer check
        // in DaqpLdp, so also enforce the limit across the BnB tree.
        if(work->timer != NULL && (work->bnb->nodecount&31)==0){
            Toc((DAQPtimer*)work->timer);
            if(GetTime((DAQPtimer*)work->timer) > work->settings->time_limit){
                exitflag = DAQP_EXIT_TIMELIMIT;
                break;
            }
        }
#endif
        // Cut conditions
        if(exitflag==DAQP_EXIT_INFEASIBLE) continue; // Dominance cut
        if(exitflag<0) break; // Inner solver failed => exit loop

        // Find index to branch over
        branch_id = DaqpGetBranchId(work);
        if(branch_id==DAQP_EMPTY_IND){// Nothing to branch over => integer feasible
            work->settings->fval_bound = (0.5*work->fval - work->settings->abs_subopt)*eps_r;
            swp_ptr=work->xold; work->xold= work->u; work->u=swp_ptr; // Store feasible sol
        }
        else{
            DaqpSpawnChildren(node,branch_id, work);
        }
    }

    // Exploration completed
    work->iterations = work->bnb->itercount;
    // Restore the root state (unfix binaries etc.) so that the workspace can
    // be reused for subsequent solves
    DaqpNodeCleanupWorkspace(work->bnb->neq,work);
    work->bnb->n_clean = work->bnb->neq;
    if(swp_ptr==NULL){
        work->settings->fval_bound = fval_bound0;
        return exitflag < 0 ? exitflag : DAQP_EXIT_INFEASIBLE;
    }
    else{
        // Invert fval_bound = (0.5*fval_best - abs_subopt)*eps_r to recover fval_best
        work->fval = 2*work->settings->fval_bound/eps_r + 2*work->settings->abs_subopt;
        work->settings->fval_bound = fval_bound0;
        // Let work->u point to the best feasible solution
        swp_ptr=work->u; work->u= work->xold; work->xold=swp_ptr;
        return exitflag < DAQP_EXIT_INFEASIBLE ? exitflag : DAQP_EXIT_OPTIMAL;
    }
}

int DaqpProcessNode(DAQPNode* node, DAQPWorkspace* work){
    int exitflag;
    work->bnb->nodecount+=1;
    if(node->depth >=0){
        // Fix a binary constraints
        work->bnb->fixed_ids[node->depth] = node->bin_id;
        // Setup relaxation
        if(work->bnb->n_nodes==0 || (node-1)->depth!=node->depth){
            // Sibling has been processed => need to fix workspace state
            work->bnb->n_clean += (node->depth-(node+1)->depth);
            if(work->bnb->n_clean < work->bnb->neq) work->bnb->n_clean = work->bnb->neq;
            DaqpNodeCleanupWorkspace(work->bnb->n_clean,work);
            if(DaqpWarmstartNode(node,work) < 0) return DAQP_EXIT_INFEASIBLE;
        }
        else{
            DaqpAddUpperLower(node->bin_id,work);
            work->sense[DAQP_REMOVE_LOWER_FLAG(node->bin_id)] |= DAQP_IMMUTABLE; //Equality
            if(work->sing_ind != DAQP_EMPTY_IND){ // Need to cold start to not miss integer feasible
                if(DaqpSetupColdBnb(node,work) < 0) return DAQP_EXIT_INFEASIBLE;
            }
        }
        // Add binary constraint
    }
    // Solve relaxation
    exitflag = DaqpLdp(work);
    work->bnb->itercount += work->iterations;

    if(exitflag == DAQP_EXIT_CYCLE){// Try to repair (cold start)
        // A cycle can be caused by stale cached forward-substitution data.
        // Force the retained fixed prefix to be recomputed during repair.
        work->reuse_ind=0;
        if(DaqpSetupColdBnb(node,work) < 0) return DAQP_EXIT_INFEASIBLE;
        exitflag = DaqpLdp(work);
        work->bnb->itercount += work->iterations;
    }

    return exitflag;
}

int DaqpGetBranchId(DAQPWorkspace* work){
    int i;
    int Id = DAQP_EMPTY_IND;
    c_float diff, dist, tol;

    for(i=0; i < work->bnb->nb; i++){
        Id = work->bnb->bin_ids[i];
        // Skip fixed binary constraints
        if(DAQP_IS_ACTIVE(Id)) continue;

        // Compute signed distance from midpoint between bounds
        diff = DaqpBinaryDiff(Id,work);

        // A zero-dual binary constraint can lie at an endpoint without being
        // active. It is already integer feasible and does not need branching.
        dist = 0.5*(work->dupper[Id]-work->dlower[Id])
            -(diff < 0 ? -diff : diff);
        tol = work->settings->primal_tol;
        if(work->scaling != NULL) tol *= work->scaling[Id];
        if(dist <= tol) continue;

        // Explore the endpoint nearest to the relaxation first.
        return diff < 0 ? Id : DAQP_ADD_LOWER_FLAG(Id);
    }

    return DAQP_EMPTY_IND;
}

void DaqpSpawnChildren(DAQPNode* node, const int branch_id, DAQPWorkspace* work){

    DaqpSaveWarmstart(node,work);

    // Update child1 (reuse current node)
    node->bin_id = DAQP_TOGGLE_LOWER_FLAG(branch_id);
    node->depth +=1;

    // Update child2
    (node+1)->bin_id = branch_id;
    (node+1)->depth = node->depth;
    (node+1)->WS_start = node->WS_start;
    (node+1)->WS_end= node->WS_end;

    work->bnb->n_nodes+=2;
}

void DaqpNodeCleanupWorkspace(int n_clean, DAQPWorkspace* work){
    int i;
    // Cleanup sense
    for(i=n_clean; i<work->n_active; i++)
        work->sense[work->WS[i]]&= DAQP_IS_BINARY(work->WS[i]) ?
            ~(DAQP_ACTIVE+DAQP_IMMUTABLE): ~DAQP_ACTIVE;
    // Reset workspace
    work->sing_ind=DAQP_EMPTY_IND;
    work->n_active=n_clean;
    // Only the retained prefix can still have a valid cached substitution.
    if(work->reuse_ind > n_clean)
        work->reuse_ind=n_clean;
}


int DaqpWarmstartNode(DAQPNode* node, DAQPWorkspace* work){
    // Add fixed constraints
    if(DaqpBnbAddFixedIds(node,work) < 0) return -1;
    // Add free constraints
    DaqpBnbLoadWs(work->bnb->tree_WS+node->WS_start,node->WS_end-node->WS_start,work);
    work->bnb->nWS = node->WS_start; // always move up tree after warmstart
    return 0;
}

void DaqpSaveWarmstart(DAQPNode* node, DAQPWorkspace* work){
    node->WS_start = work->bnb->nWS;
    work->bnb->nWS += DaqpBnbStoreWs(work->bnb->tree_WS+work->bnb->nWS,work);
    node->WS_end = work->bnb->nWS;
}

int DaqpAddUpperLower(const int add_id, DAQPWorkspace* work){
    int true_add_id = DAQP_REMOVE_LOWER_FLAG(add_id);
    // Setup new binary constraint
    if(DAQP_EXTRACT_LOWER_FLAG(add_id)){
        DAQP_SET_LOWER(true_add_id);
        DaqpAddConstraint(work,true_add_id,-1.0);
    }
    else{
        DAQP_SET_UPPER(true_add_id);
        DaqpAddConstraint(work,true_add_id,1.0);
    }
    return 1;
}

int DaqpSetupColdBnb(DAQPNode* node,DAQPWorkspace *work){
    DaqpNodeCleanupWorkspace(work->bnb->n_clean,work);
    return DaqpBnbAddFixedIds(node,work);
}
}  // namespace miqp
}  // namespace math
}  // namespace common
}  // namespace autonomy
