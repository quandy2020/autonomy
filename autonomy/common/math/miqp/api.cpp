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
 * @file api.cpp
 * @brief Setup, solve, allocate, and free the DAQP workspace.
 *
 * Declarations and the algorithm contract live in the matching header.
 * This file holds the definitions of the vendored DAQP C solver.
 */

#include "autonomy/common/math/miqp/api.hpp"
#include "autonomy/common/math/miqp/auxiliary.hpp"
#include "autonomy/common/math/miqp/utils.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


namespace autonomy {
namespace common {
namespace math {
namespace miqp {
/**
 * @brief Solve problem from a given workspace and measure setup and solve time
 */
void DaqpSolve(DAQPResult *res, DAQPWorkspace *work){
    int reduced;
    if(work->break_points == NULL) work->nh = 1;
    // A policy that no longer eliminates takes effect before the solve
    if(DAQP_IS_REDUCED(work) &&
            work->settings->eq_reduction == DAQP_EQ_REDUCTION_OFF &&
            (res->exitflag = DaqpUpdateLdp(0,work,work->qp)) < 0)
        return;
    // Solve the reduced problem if the equalities are eliminated (unless the
    // latest update found its right-hand side to be infeasible)
    if(DAQP_IS_REDUCED(work) && work->eq->error < 0){
        res->exitflag = work->eq->error;
        return;
    }
    reduced = DaqpEqInstall(work);
#ifdef PROFILING
    DAQPtimer timer;
    Tic(&timer);
    if(work->settings->time_limit > 0)  work->timer = &timer;
#endif
    if(!(work->state & DAQP_STATE_UNCONSTRAINED)){
        // Select algorithm
        if(work->n_prox==0){
            if(work->avi == NULL || work->avi->is_symmetric){
                if(work->bnb != NULL)
                    res->exitflag = DaqpBnb(work);
                else if(DAQP_IS_HIERARCHICAL(work))
                    res->exitflag = DaqpHiqp(work,res->lam);
                else
                    res->exitflag = DaqpLdp(work);
                if(res->exitflag > 0){
                    Ldp2qpSolution(work); // Retrieve qp solution
                    // Refine x (if it might be inaccurate)
                    if(work->bnb == NULL && !DAQP_IS_HIERARCHICAL(work)){
                        DaqpRefinePrimal(work);
                        // Constraints were only added above the rounding level
                        if((work->state & DAQP_STATE_NOISE_FLOOR) && DaqpViolatesHard(work))
                            res->exitflag = DAQP_EXIT_OPTIMAL_INEXACT;
                    }
                }
            }
            else{ //AVI
                res->exitflag = DaqpSolveAvi(work);
            }
        }
        else{//Prox
            if(work->bnb != NULL)
                res->exitflag = DAQP_EXIT_NONCONVEX;
            else
                res->exitflag = DaqpProx(work);
        }
    }
    else{// Unconstrained optimum
        work->iterations = 1;
        work->fval = 0;
        work->soft_slack = 0;
        res->exitflag = DAQP_EXIT_OPTIMAL;
    }
#ifdef PROFILING
    work->timer = NULL;
    Toc(&timer);
#endif

    // Package result
    DaqpExtractResult(res,work);
    if(reduced){
        DaqpEqExpand(res,work);
        DaqpEqRestore(work);
    }
    // Add time to result
#ifdef PROFILING
    res->solve_time = GetTime(&timer);
#else
    res->solve_time = 0;
#endif
}

/**
 * @brief Setup and solve problem
 */
void DaqpQuadprog(DAQPResult *res, DAQPProblem* qp, DAQPSettings *settings){
    int setup_flag;

    DAQPWorkspace work;
    work.settings = settings;
    // The problem is solved once, so automatic equality reduction applies
    setup_flag = SetupDaqpMain(qp,&work,&(res->setup_time),
            DAQP_UPDATE_unconstrained | DAQP_UPDATE_eliminate);
    res->exitflag = setup_flag;

    if(setup_flag >= 0){
        DaqpSolve(res,&work);
        // Free memory
        if(settings != NULL) work.settings = NULL;
        FreeDaqpWorkspace(&work);
        FreeDaqpLdp(&work);
    }
}

void DaqpAvi(DAQPResult *res, DAQPProblem* problem, DAQPSettings *settings){
    // Set the flag correctly
    problem->problem_type = 1;
    DaqpQuadprog(res,problem,settings);
}

/**
 * @brief Setup workspace and transform QP to LDP
 */
int SetupDaqp(DAQPProblem* qp, DAQPWorkspace *work, c_float* setup_time){
    return SetupDaqpMain(qp,work,setup_time,0);
}

/**
 * @brief Internal function for setting workspace and transform QP to LDP
 */
int SetupDaqpMain(DAQPProblem* qp, DAQPWorkspace *work, c_float* setup_time, int init_mask){
    int errorflag;
    int own_settings=1;
    (void)setup_time; // avoids warning when compiling without profiling
#ifdef PROFILING
    DAQPtimer timer;
    if(setup_time != NULL){
        *setup_time = 0; // in case setup fails
        Tic(&timer);
    }
#endif
    // Check if QP is well-posed
    //validate_QP(qp);

/**
 * @brief Count number of soft/binary constraints (to account for it in allocation)
 */
    int ns = 0, nb = 0;
    int i;
    if(qp->sense != NULL){
        for(i = 0; i < qp->m ; i++){
            if(qp->sense[i] & DAQP_SOFT) ns++;
            if(qp->sense[i] & DAQP_BINARY) nb++;
        }
    }
    // Correct number of soft constraints if several hierarchies
    if(qp->nh > 1){
        ns = 0; // Reset
        int start = 0;
        for(int i = 0; i < qp->nh; i++){
            ns = (ns  > qp->break_points[i]-start) ? ns : qp->break_points[i]-start;
            start = qp->break_points[i];
        }
     }

    // Setup workspace
    if(work->settings == NULL)
        AllocateDaqpSettings(work);
    else
        own_settings = 0;
    AllocateDaqpWorkspace(work,qp->n,ns);

    if(qp->problem_type == 1){ // Problem type 1 == AVI
        work->avi = static_cast<decltype(work->avi)>(malloc(sizeof(DAQPAVI)));
        AllocateDaqpAvi(work->avi,qp->n);
    }

    errorflag = SetupDaqpBnb(work, qp->sense, nb, ns);
    if(errorflag < 0){
        if(own_settings==0) work->settings = NULL;
        FreeDaqpWorkspace(work);
        return errorflag;
    }

    errorflag = SetupDaqpLdp(work,qp,init_mask);
    if(errorflag < 0){
        if(own_settings==0) work->settings = NULL;
        FreeDaqpWorkspace(work);
        return errorflag;
    }

#ifdef PROFILING
    if(setup_time != NULL){
        Toc(&timer);
        *setup_time = GetTime(&timer);
    }
#endif
    return 1;
}

/**
 * @brief Setup LDP from QP
 */
int SetupDaqpLdp(DAQPWorkspace *work, DAQPProblem *qp, const int init_mask){
/**
 * @brief Always update M, d and sense
 */
    int update_mask =
        init_mask | DAQP_UPDATE_M | DAQP_UPDATE_d | DAQP_UPDATE_sense;
    int error_flag;
    int alloc_R=0, alloc_v=0;


    // Only allocate Rinv if H is not NULL
    if(qp->H!=NULL){
        alloc_R = 1;
        update_mask |= DAQP_UPDATE_Rinv;
    }
    // Only allocate v if f is not NULL (QP linear term or LP objective).
    if(qp->f!=NULL){
        alloc_v = 1;
        update_mask |= DAQP_UPDATE_v;
    }

    // For LPs (no Hessian), mark all directions as needing proximal regularisation.
    // This lets DaqpSolve dispatch to DaqpProx based on n_prox rather than eps_prox.
    if(qp->H == NULL && qp->f!=NULL)
        work->n_prox = work->n;

    // Allocate memory for LDP
    AllocateDaqpLdp(work, qp->n, qp->m, qp->ms, alloc_R, alloc_v);

    // Update hierarchy if hqp
    if(qp->nh > 1) update_mask |= DAQP_UPDATE_hierarchy;

    // Form LDP
    error_flag = DaqpUpdateLdp(update_mask, work, qp);
    if(error_flag<0){
        FreeDaqpLdp(work);
        return error_flag;
    }

    /*
     * A singular quadratic needs v for the proximal linear term even when
     * the original problem has no f. Allocate it only after factorization
     * has identified that need, keeping the positive-definite path unchanged.
     */
    if(work->n_prox > 0 && work->v == NULL && qp->H != NULL)
        work->v = static_cast<decltype(work->v)>(calloc(work->n, sizeof(c_float)));

    return 1;
}


void SetupDaqpHiqp(DAQPWorkspace* work, int* break_points, int nh){
    if(nh > 1){
        work->nh = nh;
        work->break_points= break_points;
    }
}

int SetupDaqpBnb(DAQPWorkspace* work, int* sense, int nb, int ns){
    int i, nadded;
    if(nb > work->n) return DAQP_EXIT_OVERDETERMINED_INITIAL;
    if((work->bnb == NULL) && (nb >0)){
        work->bnb = static_cast<decltype(work->bnb)>(malloc(sizeof(DAQPBnB)));

        work->bnb->nb = nb;
        // Detect which constraints are binary
        work->bnb->bin_ids = static_cast<decltype(work->bnb->bin_ids)>(malloc(nb*sizeof(int)));
        for(i = 0, nadded = 0; nadded < nb; i++){
            if(sense[i] & DAQP_BINARY)
                work->bnb->bin_ids[nadded++] = i;
        }

        // Setup tree
        work->bnb->tree = static_cast<decltype(work->bnb->tree)>(malloc((work->bnb->nb+1)*sizeof(DAQPNode)));
        work->bnb->tree_WS = static_cast<decltype(work->bnb->tree_WS)>(malloc((work->n+ns+1)*(nb+1)*sizeof(int)));
        work->bnb->n_nodes = 0;
        work->bnb->nWS= 0;
        work->bnb->fixed_ids = static_cast<decltype(work->bnb->fixed_ids)>(malloc((nb+1)*sizeof(int)));
        work->bnb->root_WS = static_cast<decltype(work->bnb->root_WS)>(malloc((work->n+ns+1)*sizeof(int)));
        work->bnb->n_root_WS= 0;
    }
    return 1;
}

/**
 * @brief Free data for LDP
 */
void FreeDaqpLdp(DAQPWorkspace *work){
    if(work->sense==NULL) return; // Already freed
    FreeDaqpEq(work); // Restores the full LDP before it is freed
    free(work->sense);
    if(work->Rinv != NULL){
        free(work->Rinv);
    }
    if(work->RinvD != NULL){
        free(work->RinvD);
    }
    if(work->v != NULL){
        free(work->v);
    }
    if(work->scaling != NULL){
        free(work->scaling);
        free(work->M);
        free(work->Mu);
        free(work->dupper);
        free(work->dlower);
    }

#ifdef DAQP_SOFT_WEIGHTS
    free(work->rho_ls); // Single block for all the weights
#endif
    work->rho_ls = NULL;
    work->rho_us = NULL;
    work->w_ls = NULL;
    work->w_us = NULL;

    work->sense = NULL;
}

void AllocateDaqpSettings(DAQPWorkspace *work){
    if(work->settings == NULL){
        work->settings = static_cast<decltype(work->settings)>(malloc(sizeof(DAQPSettings)));
        DaqpDefaultSettings(work->settings);
    }
}

void FreeDaqpBnb(DAQPWorkspace* work){
    if(work->bnb != NULL){
        free(work->bnb->bin_ids);
        free(work->bnb->tree);
        free(work->bnb->tree_WS);
        free(work->bnb->fixed_ids);
        free(work->bnb->root_WS);
        free(work->bnb);
        work->bnb = NULL;
    }
}

/**
 * @brief Allocate memory for iterates
 */
void AllocateDaqpWorkspace(DAQPWorkspace *work, int n, int ns){
    work->n = n;
    n = n + ns; //To account for soft_constraints
    work->Rinv = NULL;
    work->RinvD = NULL;
    work->v = NULL;
    work->scaling = NULL;
    work->Mu = NULL;

    work->lam = static_cast<decltype(work->lam)>(malloc((n+1)*sizeof(c_float)));
    work->lam_star = static_cast<decltype(work->lam_star)>(malloc((n+1)*sizeof(c_float)));

    work->WS = static_cast<decltype(work->WS)>(malloc((n+1)*sizeof(int)));

    work->D = static_cast<decltype(work->D)>(malloc((n+1)*sizeof(c_float)));
    work->xldl = static_cast<decltype(work->xldl)>(malloc((n+1)*sizeof(c_float)));
    work->zldl = static_cast<decltype(work->zldl)>(malloc((n+1)*sizeof(c_float)));
    work->L = static_cast<decltype(work->L)>(malloc(((n+1)*(n+2)/2)*sizeof(c_float)));



    work->u = static_cast<decltype(work->u)>(calloc(work->n,sizeof(c_float))); // calloc -> uninitialized iterate is 0
    work->x = work->u;

    work->xold = static_cast<decltype(work->xold)>(malloc(work->n*sizeof(c_float)));

    work->prox_mask = static_cast<decltype(work->prox_mask)>(calloc(work->n, sizeof(int))); // all zeros initially
    work->n_prox = 0;
    work->state = 0;
    work->soft_slack = 0;

    work->rho_ls= NULL;
    work->rho_us= NULL;
    work->w_ls= NULL;
    work->w_us= NULL;

    work->bnb = NULL;
    work->nh = 1;
    work->break_points = NULL;
    work->avi = NULL;
    work->eq = NULL;
    work->timer = NULL;

    ResetDaqpWorkspace(work);
}

void AllocateDaqpLdp(DAQPWorkspace *work, int n, int m, int ms, int alloc_R, int alloc_v){
    int i;
    // Always allocate scaling, M ,dupper, dlower,sense
    work->scaling = static_cast<decltype(work->scaling)>(malloc(m*sizeof(c_float)));
    for(i =0; i < ms; i++) work->scaling[i] =1;
    work->M = static_cast<decltype(work->M)>(malloc(n*(m-ms)*sizeof(c_float)));
    work->Mu = m > ms ? static_cast<decltype(work->Mu)>(malloc((m-ms)*sizeof(c_float))) : NULL;
    work->dupper = static_cast<decltype(work->dupper)>(malloc(m*sizeof(c_float)));
    work->dlower = static_cast<decltype(work->dlower)>(malloc(m*sizeof(c_float)));
    work->sense = static_cast<decltype(work->sense)>(malloc(m*sizeof(int)));

    // Allocate memory for Rinv
    work->Rinv = (alloc_R == 1) ? static_cast<decltype(work->Rinv)>(malloc(((n+1)*n/2)*sizeof(c_float))) : NULL;
    // Allocate memory for v
    work->v = (alloc_v == 1) ? static_cast<decltype(work->v)>(malloc(n*sizeof(c_float))) :  NULL;

}

/**
 * @brief Weights of the soft constraints, in one block (zero selects the default). They are
 *        only allocated on request, so that a solve that uses the uniform weights in settings
 *        neither spends the memory nor reads the arrays.
 */
int DaqpAllocateSoftWeights(DAQPWorkspace *work){
#ifdef DAQP_SOFT_WEIGHTS
    if(work->rho_ls != NULL) return 1; // Already allocated
    // The weights are indexed by the original problem (also if its equality
    // constraints are eliminated). An interface may set work->m to allocate
    // for a later, larger problem.
    const int m = work->m;
    if(m == 0) return 0;
    work->rho_ls = static_cast<decltype(work->rho_ls)>(calloc(4*m,sizeof(c_float)));
    if(work->rho_ls == NULL) return 0;
    work->rho_us = work->rho_ls + m;
    work->w_ls = work->rho_ls + 2*m;
    work->w_us = work->rho_ls + 3*m;
    return 1;
#else
    (void)work; return 0; // Built without support for individual weights
#endif
}

/**
 * @brief Refresh the cached active set after uniform or individual soft weights change.
 */
void DaqpRefreshSoftWeights(DAQPWorkspace *work){
    int i, rebuild = 0;
    // The working set is that of the reduced problem if equalities are eliminated
    const int reduced = DaqpEqInstall(work);
    // Only an active soft constraint makes the factorization stale.
    for(i = 0; i < work->n_active; i++)
        if(DAQP_IS_SOFT(work->WS[i])){
            rebuild = 1;
            break;
        }
    // Reset the factorization
    if(rebuild){
        ResetDaqpWorkspace(work);
        if(DAQP_IS_HIERARCHICAL(work)){
            const int m = work->m;
            work->m = work->break_points[0];
            DaqpActivateConstraints(work);
            work->m = m;
        }
        else
            DaqpActivateConstraints(work);
    }
    if(reduced) DaqpEqRestore(work);
}

/**
 * @brief Set the weights of the soft constraints, one entry per constraint of the original
 *        problem (NULL leaves that weight untouched). Returns 0 if the weights are
 *        unavailable, i.e. if the build has no support for them.
 */
int DaqpSetSoftWeights(DAQPWorkspace *work, c_float *rho_l, c_float *rho_u,
        c_float *w_l, c_float *w_u){
    int i;
    if(!DaqpAllocateSoftWeights(work)) return 0;
    const int m = work->m;
    for(i = 0; i < m; i++){
        if(rho_l != NULL) work->rho_ls[i] = rho_l[i];
        if(rho_u != NULL) work->rho_us[i] = rho_u[i];
        if(w_l != NULL) work->w_ls[i] = w_l[i];
        if(w_u != NULL) work->w_us[i] = w_u[i];
    }
    DaqpRefreshSoftWeights(work);
    return 1;
}

void AllocateDaqpAvi(DAQPAVI* avi, const int n){
    avi->is_symmetric = 0;
    avi->retry_rho_needed = 0;
    avi->rho = 0.0;


    // Allocate matrices
    avi->Hsym = static_cast<decltype(avi->Hsym)>(malloc(n*n*sizeof(c_float)));
    avi->Hs_rho = static_cast<decltype(avi->Hs_rho)>(malloc(n*n*sizeof(c_float)));
    avi->H_rho = static_cast<decltype(avi->H_rho)>(malloc(n*n*sizeof(c_float)));
    avi->P_H2 = static_cast<decltype(avi->P_H2)>(malloc(n*sizeof(int)));

    avi->LU_H = static_cast<decltype(avi->LU_H)>(malloc(n*n*sizeof(c_float)));
    avi->P_H = static_cast<decltype(avi->P_H)>(malloc(n*sizeof(int)));

    avi->kkt_buffer = static_cast<decltype(avi->kkt_buffer)>(malloc((n*n+2*n)*sizeof(c_float)));
    avi->P_S = static_cast<decltype(avi->P_S)>(malloc(n*sizeof(int)));

    // Allocate iterate (maybe reuse from normal workspace...)
    avi->Hx = static_cast<decltype(avi->Hx)>(malloc(n*sizeof(c_float)));
    avi->x = static_cast<decltype(avi->x)>(malloc(n*sizeof(c_float)));
    avi->y = static_cast<decltype(avi->y)>(malloc(n*sizeof(c_float)));
    avi->xtemp = static_cast<decltype(avi->xtemp)>(malloc(n*sizeof(c_float)));
}


/**
 * @brief Free memory for iterates
 */
void FreeDaqpWorkspace(DAQPWorkspace *work){
    if(work->lam != NULL){
        free(work->lam);
        free(work->lam_star);

        free(work->WS);

        free(work->L);
        free(work->D);

        free(work->xldl);
        free(work->zldl);

        free(work->u);

        free(work->xold);

        free(work->prox_mask);

        work->lam = NULL;
    }

    if(work->settings != NULL){
        free(work->settings);
        work->settings = NULL;
    }

    FreeDaqpBnb(work);
    FreeDaqpAvi(work);

}

void FreeDaqpAvi(DAQPWorkspace* work){
    if(work->avi != NULL){
        free(work->avi->Hsym);
        free(work->avi->Hs_rho);
        free(work->avi->H_rho);
        free(work->avi->P_H2);

        free(work->avi->LU_H);
        free(work->avi->P_H);

        free(work->avi->kkt_buffer);
        free(work->avi->P_S);

        free(work->avi->Hx);
        free(work->avi->x);
        free(work->avi->y);
        free(work->avi->xtemp);
        free(work->avi);
        work->avi = NULL;
    }
}


/**
 * @brief Extract solution information from workspace
 */
void DaqpExtractResult(DAQPResult* res, DAQPWorkspace* work){
    int i;
    // Extract primal solution
    for(i=0;i<work->n;i++) res->x[i] = work->x[i];

    // Extract dual solution for ordinary QPs. Hierarchical QPs populate the
    // output duals in DaqpHiqp(), where they represent the soft-level
    // penalties used by the public interfaces.
    if(res->lam != NULL && !DAQP_IS_HIERARCHICAL(work)){
        for(i=0;i<work->m;i++)
            res->lam[i] = 0;
        for(i=0;i<work->n_active;i++){
            const int Id = work->WS[i];
            const c_float lam = work->lam_star[i];
            // Report a multiplier of the wrong sign (within dual_tol) as zero
            if(!DAQP_IS_IMMUTABLE(Id) && !DAQP_IS_SOFT(Id) &&
                    (DAQP_IS_LOWER(Id) ? lam > 0 : lam < 0))
                res->lam[Id] = 0;
            else
                res->lam[Id] = lam;
        }
    }

    // Shift back function value
    if(work->v != NULL &&
            (work->avi == NULL || work->avi->is_symmetric) &&
            (work->Rinv != NULL || work->RinvD != NULL)){ // QP or symmetric AVI
        res->fval = work->fval;
        for(i=0;i<work->n;i++) res->fval-=work->v[i]*work->v[i];
        res->fval *=0.5;
    }
    else if(work->qp != NULL && work->qp->f != NULL ){ // LP
        res->fval = 0;
        for(i=0;i<work->n;i++) res->fval+=work->qp->f[i]*work->x[i];
    }

    // info
    res->soft_slack = work->soft_slack;
    res->iter = work->iterations;
    if(work->bnb != NULL)
        res->nodes = work->bnb->nodecount;
    else if(DAQP_IS_HIERARCHICAL(work))
        res->nodes = 1;
    else
        res->nodes = work->nh;
}

void DaqpExtractActiveDuals(DAQPResult* res, DAQPWorkspace* work){
    int i;
    if(res->lam == NULL) return;
    for(i=0;i<work->m;i++) res->lam[i] = 0;
    for(i=0;i<work->n_active;i++)
        res->lam[work->WS[i]] = work->lam_star[i];
}

void DaqpDefaultSettings(DAQPSettings* settings){
    settings->primal_tol = DAQP_DEFAULT_PRIM_TOL;
    settings->dual_tol = DAQP_DEFAULT_DUAL_TOL;
    settings->zero_tol = DAQP_DEFAULT_ZERO_TOL;
    settings->pivot_tol = DAQP_DEFAULT_PIVOT_TOL;
    settings->progress_tol= DAQP_DEFAULT_PROG_TOL;

    settings->cycle_tol = DAQP_DEFAULT_CYCLE_TOL;
    settings->iter_limit = DAQP_DEFAULT_ITER_LIMIT;
    settings->fval_bound = DAQP_INF;

    settings->eps_prox = DAQP_DEFAULT_EPS_PROX;
    settings->eta_prox = DAQP_DEFAULT_ETA;

    settings->rho_soft = DAQP_DEFAULT_RHO_SOFT;
    settings->w_soft = DAQP_DEFAULT_W_SOFT;

    settings->rel_subopt = DAQP_DEFAULT_REL_SUBOPT;
    settings->abs_subopt = DAQP_DEFAULT_ABS_SUBOPT;

    settings->sing_tol = DAQP_DEFAULT_SING_TOL;
    settings->refactor_tol = DAQP_DEFAULT_REFACTOR_TOL;
    settings->time_limit = 0;
    settings->eq_reduction = DAQP_EQ_REDUCTION_AUTO;
}

/* Remove redundant constraints*/
/**
 * @brief Determine reundant constraint of the polyhedron A*x <= b
 */
void DaqpMinrep(int* is_redundant, c_float* A, c_float* b, int n, int m, int ms)
{
    int i;
    // Setup workspace
    DAQPWorkspace work;
    work.settings = NULL;
    AllocateDaqpWorkspace(&work,n,0);
    AllocateDaqpSettings(&work);
    work.M = A;
    work.dupper = b;
    work.m = m;
    work.ms = ms;

    work.dlower = static_cast<decltype(work.dlower)>(malloc(m*sizeof(c_float)));
    work.sense = static_cast<decltype(work.sense)>(malloc(m*sizeof(int)));
    for(i = 0; i<m; i++){
        work.dlower[i] = -DAQP_INF;
        work.sense[i] = 0;
    }


    // Solve minrep
    DaqpMinrepWork(is_redundant, &work);
    // free
    FreeDaqpWorkspace(&work);
    free(work.dlower);
    free(work.sense);
}


/**
 * @brief Find which constraints the point x first violates
 */
int DaqpFirstViolating(c_float* x, c_float* A, c_float* bu, c_float* bl, int n, int m, int ms, c_float tol){
    int i,j,disp;
    c_float Ax;
    for(i=0; i < ms; i++)
        if(x[i] > bu[i]+tol || x[i] < bl[i]-tol) return i;

    for(disp=0; i < m; i++){
        Ax = 0;
        for(j=0;j<n;j++) Ax += A[disp++]*x[j];
        if(Ax > bu[i]+tol || Ax < bl[i]-tol) return i;
    }
    return m; // No constraint is violating
}


/**
 * @brief Sets the starting active-set (by modifying sense) based on a primal iterate
 */
void DaqpPrimalInitActive(DAQPProblem* qp, c_float* x){
    int i,disp;
    c_float Ax, slack;
    c_float tol= 1e-9;

    // For MIQPs, x is only used as an incumbent (see DaqpSetPrimalStart)
    for(i=0; i < qp->m; i++)
        if(qp->sense[i] & DAQP_BINARY) return;

    // Simple constraints
    for(i=0; i < qp->ms; i++){
        if(qp->sense[i] & DAQP_IMMUTABLE) continue;
        slack = x[i]- qp->bupper[i];
        if (slack < tol && slack > -tol){
            qp->sense[i] |= DAQP_ACTIVE;
            qp->sense[i] &= ~DAQP_LOWER;
        }
        else{
            slack = x[i] - qp->blower[i];
            if (slack < tol && slack > -tol){
                qp->sense[i] |= DAQP_ACTIVE+DAQP_LOWER;
            }
        }
    }

    // General constraints
    for(i=qp->ms, disp=0; i < qp->m; i++, disp+=qp->n){
        if(qp->sense[i] & DAQP_IMMUTABLE) continue;
        Ax = DaqpDotInline(x,qp->A+disp,qp->n);
        slack = Ax - qp->bupper[i];
        if (slack < tol && slack > -tol){
            qp->sense[i] |= DAQP_ACTIVE;
            qp->sense[i] &= ~DAQP_LOWER;
        }
        else{
            slack = Ax - qp->blower[i];
            if (slack < tol && slack > -tol){
                qp->sense[i] |= DAQP_ACTIVE+DAQP_LOWER;
            }
        }
    }
}

/**
 * @brief Sets the starting active-set (by modifying sense) based on a dual iterate
 */
void DaqpDualInitActive(DAQPProblem* qp, c_float* lam){
    int i;
    c_float tol = 1e-12;
    for(i=0; i < qp->m; i++){
        if(qp->sense[i] & DAQP_IMMUTABLE) continue;
        if(lam[i] > tol){
            qp->sense[i] |= DAQP_ACTIVE;
            qp->sense[i] &= ~DAQP_LOWER;
        }
        else if(lam[i] < -tol){
            qp->sense[i] |= DAQP_ACTIVE+DAQP_LOWER;
        }
    }
}

/**
 * @brief Set the starting iterate
 */
void DaqpSetPrimalStart(DAQPWorkspace* work, c_float* x){
    // x is given for the original problem, also if its equalities are eliminated
    const int reduced = DaqpEqInstall(work);
    if(!(work->state & DAQP_STATE_UNCONSTRAINED)){
        int i;
        if(reduced) DaqpEqSetPrimalStart(work,x);
        else for(i = 0; i < work->n; i++) work->x[i] = x[i];
        if(work->bnb != NULL) work->state |= DAQP_STATE_INCUMBENT;
    }
    if(reduced) DaqpEqRestore(work);
}
}  // namespace miqp
}  // namespace math
}  // namespace common
}  // namespace autonomy
