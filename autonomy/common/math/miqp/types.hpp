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
 * @file types.hpp
 * @brief Public DAQP C types: problem, settings, workspace, and helpers.
 *
 * Vendored DAQP (MIT). These structures are the C ABI used by solver.hpp.
 * A quadratic program is
 * min 0.5 x' H x + f' x subject to lb <= x <= ub and lbA <= A x <= ubA.
 * Simple bounds occupy the first ms rows of blower/bupper; general rows
 * follow. sense stores active, immutable, lower/upper, soft, and binary
 * flags. Equality elimination keeps a second least-distance program that is
 * swapped into the workspace while the reduced problem is formed or solved.
 */

#ifndef DAQP_TYPES_H
# define DAQP_TYPES_H



namespace autonomy {
namespace common {
namespace math {
namespace miqp {
#ifdef DAQP_SINGLE_PRECISION
/**
 * @brief Scalar type when DAQP is built in single precision.
 */
typedef float c_float;
#else
/**
 * @brief Scalar type when DAQP is built in double precision.
 */
typedef double c_float;
#endif

/**
 * @struct DAQPProblem
 * @brief Quadratic (or mixed-integer quadratic) program in DAQP layout.
 *
 * blower = [lb; lbA], bupper = [ub; ubA]. The number of rows in A is m - ms.
 * problem_type is 1 for an affine variational inequality, otherwise a
 * quadratic program. nh and break_points describe a hierarchical program.
 */
typedef struct {
    /**
     * @brief Number of decision variables.
     */
    int n;

    /**
     * @brief Total number of constraints, simple bounds included.
     */
    int m;

    /**
     * @brief Number of simple bounds (leading rows of blower/bupper).
     */
    int ms;

    /**
     * @brief Hessian of the quadratic, row-major, n by n.
     */
    c_float* H;

    /**
     * @brief Linear term, length n.
     */
    c_float* f;

    /**
     * @brief General constraint matrix, row-major, (m - ms) by n.
     */
    c_float* A;

    /**
     * @brief Upper bounds [ub; ubA], length m.
     */
    c_float* bupper;

    /**
     * @brief Lower bounds [lb; lbA], length m.
     */
    c_float* blower;

    /**
     * @brief Per-constraint sense bits (active, immutable, soft, binary).
     */
    int* sense;

    /**
     * @brief Hierarchical level breaks, or NULL when unused.
     */
    int* break_points;

    /**
     * @brief Number of hierarchical levels.
     */
    int nh;

    /**
     * @brief 1 for an affine variational inequality, otherwise a quadratic.
     */
    int problem_type;
} DAQPProblem;

/**
 * @struct DAQPSettings
 * @brief Tolerances, iteration limits, and penalty parameters.
 */
typedef struct {
    /**
     * @brief Primal feasibility tolerance.
     */
    c_float primal_tol;

    /**
     * @brief Dual feasibility tolerance.
     */
    c_float dual_tol;

    /**
     * @brief Numerical zero used in factorizations.
     */
    c_float zero_tol;

    /**
     * @brief Pivot tolerance for the LDL' update.
     */
    c_float pivot_tol;

    /**
     * @brief Minimum dual progress required to avoid cycling.
     */
    c_float progress_tol;

    /**
     * @brief Consecutive stall iterations before a cycle is declared.
     */
    int cycle_tol;

    /**
     * @brief Active-set iteration budget.
     */
    int iter_limit;

    /**
     * @brief Dual objective bound used to prune search.
     */
    c_float fval_bound;

    /**
     * @brief Proximal regularization; negative selects the default policy.
     */
    c_float eps_prox;

    /**
     * @brief Proximal step-size factor.
     */
    c_float eta_prox;

    /**
     * @brief Reciprocal quadratic weight of a uniform soft constraint.
     */
    c_float rho_soft;

    /**
     * @brief Relative mixed-integer suboptimality tolerance.
     */
    c_float rel_subopt;

    /**
     * @brief Absolute mixed-integer suboptimality tolerance.
     */
    c_float abs_subopt;

    /**
     * @brief Singularity tolerance on the working-set Gramian.
     */
    c_float sing_tol;

    /**
     * @brief Tolerance that triggers a refactor of the working set.
     */
    c_float refactor_tol;

    /**
     * @brief Wall-clock budget in seconds (requires PROFILING).
     */
    c_float time_limit;

    /**
     * @brief Linear soft-constraint weight; zero is a pure quadratic penalty.
     */
    c_float w_soft;

    /**
     * @brief Equality-reduction policy: DAQP_EQ_REDUCTION_{OFF,AUTO,ON}.
     */
    int eq_reduction;
} DAQPSettings;

/**
 * @struct DAQPNode
 * @brief One node of the mixed-integer branch-and-bound tree.
 */
typedef struct {
    /**
     * @brief Binary constraint that was fixed to reach this node.
     */
    int bin_id;

    /**
     * @brief Depth in the tree.
     */
    int depth;

    /**
     * @brief Start index of the stored working set in tree_WS.
     */
    int WS_start;

    /**
     * @brief End index of the stored working set in tree_WS.
     */
    int WS_end;
} DAQPNode;

/**
 * @struct DAQPBnB
 * @brief Branch-and-bound bookkeeping attached to a workspace.
 */
typedef struct {
    /**
     * @brief Indices of binary constraints.
     */
    int* bin_ids;

    /**
     * @brief Number of binary constraints.
     */
    int nb;

    /**
     * @brief Number of equalities present at the root.
     */
    int neq;

    /**
     * @brief Open nodes of the tree.
     */
    DAQPNode* tree;

    /**
     * @brief Number of stored nodes.
     */
    int n_nodes;

    /**
     * @brief Packed working sets of the tree.
     */
    int* tree_WS;

    /**
     * @brief Number of entries used in tree_WS.
     */
    int nWS;

    /**
     * @brief Number of constraints kept when a node is cleaned.
     */
    int n_clean;

    /**
     * @brief Binaries fixed on the path from the root.
     */
    int* fixed_ids;

    /**
     * @brief Nodes processed so far.
     */
    int nodecount;

    /**
     * @brief Cumulative active-set iterations.
     */
    int itercount;

    /**
     * @brief Working set of the latest root relaxation.
     */
    int* root_WS;

    /**
     * @brief Length of root_WS.
     */
    int n_root_WS;
} DAQPBnB;

/**
 * @struct DAQPAVI
 * @brief Affine variational inequality factorization and temporaries.
 */
typedef struct {
    /**
     * @brief Nonzero when the Hessian is treated as symmetric.
     */
    int is_symmetric;

    /**
     * @brief Nonzero when a smaller proximal rho should be retried.
     */
    int retry_rho_needed;

    /**
     * @brief Symmetrized Hessian storage.
     */
    c_float* Hsym;

    /**
     * @brief Hessian plus a diagonal proximal shift.
     */
    c_float* Hs_rho;

    /**
     * @brief Hessian used in the KKT system.
     */
    c_float* H_rho;

    /**
     * @brief Pivot permutation for the second factorization.
     */
    int* P_H2;

    /**
     * @brief LU factors of the Hessian.
     */
    c_float* LU_H;

    /**
     * @brief Pivot permutation of LU_H.
     */
    int* P_H;

    /**
     * @brief Scratch for the KKT solve.
     */
    c_float* kkt_buffer;

    /**
     * @brief Pivot permutation of the KKT system.
     */
    int* P_S;

    /**
     * @brief Scratch primal vector.
     */
    c_float* xtemp;

    /**
     * @brief Hessian-times-x product.
     */
    c_float* Hx;

    /**
     * @brief Current primal iterate.
     */
    c_float* x;

    /**
     * @brief Dual or complementarity slack.
     */
    c_float* y;

    /**
     * @brief Proximal parameter of the affine variational inequality.
     */
    c_float rho;
} DAQPAVI;

/**
 * @struct DAQPLDPData
 * @brief Least-distance program that the active-set method actually solves.
 *
 * Equality elimination keeps a second copy and swaps it into the workspace
 * while the reduced problem is formed or solved, so the solvers only ever see
 * an ordinary problem.
 */
typedef struct {
    /**
     * @brief Quadratic program this least-distance program was built from.
     */
    DAQPProblem* qp;

    /**
     * @brief Number of primal variables.
     */
    int n;

    /**
     * @brief Number of constraints.
     */
    int m;

    /**
     * @brief Number of simple bounds.
     */
    int ms;

    /**
     * @brief Dual Hessian factor (n by m); M' M is the dual Hessian.
     */
    c_float* M;

    /**
     * @brief Upper dual linear term, length m.
     */
    c_float* dupper;

    /**
     * @brief Lower dual linear term, length m.
     */
    c_float* dlower;

    /**
     * @brief Inverse of the upper Cholesky factor of the primal Hessian.
     */
    c_float* Rinv;

    /**
     * @brief Diagonal of Rinv when the Hessian is diagonal.
     */
    c_float* RinvD;

    /**
     * @brief v = R'\\f, used to map a quadratic program to a least-distance
     *        program.
     */
    c_float* v;

    /**
     * @brief Row normalizations of M.
     */
    c_float* scaling;

    /**
     * @brief Cached M u from the last feasibility scan.
     */
    c_float* Mu;

    /**
     * @brief Constraint sense bits.
     */
    int* sense;

    /**
     * @brief Reciprocal quadratic soft weights on the lower side.
     */
    c_float* rho_ls;

    /**
     * @brief Reciprocal quadratic soft weights on the upper side.
     */
    c_float* rho_us;

    /**
     * @brief Linear soft weights on the lower side.
     */
    c_float* w_ls;

    /**
     * @brief Linear soft weights on the upper side.
     */
    c_float* w_us;

    /**
     * @brief Workspace state bits (pending updates, incumbent, ...).
     */
    int state;

    /**
     * @brief Number of directions that needed proximal regularization.
     */
    int n_prox;

    /**
     * @brief Binary constraint indices when branch-and-bound is active.
     */
    int* bin_ids;

    /**
     * @brief Number of binary constraints.
     */
    int nb;
} DAQPLDPData;

/**
 * @struct DAQPEqElim
 * @brief Equality elimination applied before the least-distance transform.
 *
 * Equalities A_E x = b_E are written x = xp + W w. If the reduced Hessian is
 * positive definite, W satisfies W' H W = I and xp minimizes over the
 * equalities, so the reduced problem is min 0.5 ||w||^2. Otherwise W is
 * orthonormal and the reduced problem keeps W' H W (singular quadratic,
 * linear, or affine variational).
 */
typedef struct {
    /**
     * @brief Number of primal variables of the original problem.
     */
    int n;

    /**
     * @brief Number of constraints of the original problem.
     */
    int m;

    /**
     * @brief Number of simple bounds of the original problem.
     */
    int ms;

    /**
     * @brief Number of eliminated equality constraints.
     */
    int neq;

    /**
     * @brief Number of variables of the reduced problem (n - neq).
     */
    int nz;

    /**
     * @brief Number of constraints of the reduced problem.
     */
    int mr;

    /**
     * @brief Number of constraints implied by the equalities.
     */
    int ndrop;

    /**
     * @brief Number of equality candidates.
     */
    int ncand;

    /**
     * @brief Reduction path: DAQP_EQ_PATH_LDP, DAQP_EQ_PATH_QP, or
     *        DAQP_EQ_PATH_LP.
     */
    int path;

    /**
     * @brief Nonzero when the Hessian is diagonal (QR in the metric of H).
     */
    int metric;

    /**
     * @brief Nonzero when a reduction has been formed.
     */
    int active;

    /**
     * @brief Exit flag if the latest right-hand side is infeasible; else 0.
     */
    int error;

    /**
     * @brief Nonzero when the reduced problem currently occupies the
     *        workspace.
     */
    int installed;

    /**
     * @brief Eliminated equalities, stored in ascending order.
     */
    int* eq_ids;

    /**
     * @brief Candidates that the reduction was formed for.
     */
    int* cand_ids;

    /**
     * @brief Original index of each constraint of the reduced problem.
     */
    int* keep;

    /**
     * @brief Constraints implied by the equalities.
     */
    int* drop_ids;

    /**
     * @brief Householder vectors (leading neq columns), then Z = Q2.
     */
    c_float* V;

    /**
     * @brief Householder scalars.
     */
    c_float* tau;

    /**
     * @brief Normalization of the eliminated equalities.
     */
    c_float* s_eq;

    /**
     * @brief Triangular factor of the normalized A_E', packed by columns.
     */
    c_float* R;

    /**
     * @brief H^{-1/2} for a diagonal Hessian.
     */
    c_float* dsq;

    /**
     * @brief Null-space basis, n by nz, row-major.
     */
    c_float* W;

    /**
     * @brief Particular solution of the equalities.
     */
    c_float* xp;

    /**
     * @brief Objective value at xp.
     */
    c_float fp;

    /**
     * @brief Scratch of length 3 n.
     */
    c_float* tmp;

    /**
     * @brief Cached responses of xp, H xp, and bound shifts to each equality
     *        right-hand side.
     *
     * cols[k] = [xp_k (n), H xp_k (n), shift_k (mr)], formed when first needed.
     */
    c_float** cols;

    /**
     * @brief Number of entries in cols.
     */
    int ncols;

    /**
     * @brief Response of the particular solution to f.
     */
    c_float* xf;

    /**
     * @brief Response of H xp to f.
     */
    c_float* gf;

    /**
     * @brief Response of the bound shift to f.
     */
    c_float* df;

    /**
     * @brief Accumulated bound shift.
     */
    c_float* sh;

    /**
     * @brief Nonzero when xf, gf, and df are valid.
     */
    int f_valid;

    /**
     * @brief Reduced quadratic program.
     */
    DAQPProblem qp;

    /**
     * @brief Reduced Hessian storage.
     */
    c_float* Hr;

    /**
     * @brief Reduced linear term.
     */
    c_float* fr;

    /**
     * @brief Reduced general constraint matrix.
     */
    c_float* Ar;

    /**
     * @brief Reduced upper bounds.
     */
    c_float* bur;

    /**
     * @brief Reduced lower bounds.
     */
    c_float* blr;

    /**
     * @brief Reduced sense bits.
     */
    int* sr;

    /**
     * @brief Soft weights of the reduced constraints (one block).
     */
    c_float* rho_r;

    /**
     * @brief Least-distance program that is not currently in the workspace.
     */
    DAQPLDPData other;
} DAQPEqElim;

/**
 * @struct DAQPWorkspace
 * @brief Solver state: least-distance data, iterates, LDL' factors, and
 *        optional branch-and-bound / proximal / equality-reduction attachments.
 *
 * Soft penalties are in the scale of the original problem: the objective
 * gains w S + S^2 / (2 rho) per violated side. A zero entry selects
 * settings->w_soft and settings->rho_soft. The weight arrays are NULL until
 * DaqpAllocateSoftWeights is called so a uniform-weight solve spends no
 * extra memory.
 */
typedef struct {
    /**
     * @brief Quadratic program this workspace was set up from.
     */
    DAQPProblem* qp;

    /**
     * @brief Number of primal variables.
     */
    int n;

    /**
     * @brief Number of constraints.
     */
    int m;

    /**
     * @brief Number of simple bounds.
     */
    int ms;

    /**
     * @brief Dual Hessian factor; M' M is the dual Hessian (n by m).
     */
    c_float* M;

    /**
     * @brief Upper dual linear term, length m.
     */
    c_float* dupper;

    /**
     * @brief Lower dual linear term, length m.
     */
    c_float* dlower;

    /**
     * @brief Inverse of the upper Cholesky factor of the primal Hessian.
     */
    c_float* Rinv;

    /**
     * @brief v = R'\\f, used to map a quadratic program to a least-distance
     *        program.
     */
    c_float* v;

    /**
     * @brief Constraint sense bits.
     */
    int* sense;

    /**
     * @brief Row normalizations.
     */
    c_float* scaling;

    /**
     * @brief Diagonal of Rinv when the Hessian is diagonal.
     */
    c_float* RinvD;

    /**
     * @brief Final primal solution.
     */
    c_float* x;

    /**
     * @brief Previous primal iterate (proximal-point iterations).
     */
    c_float* xold;

    /**
     * @brief Dual iterate.
     */
    c_float* lam;

    /**
     * @brief Current constrained stationary point.
     */
    c_float* lam_star;

    /**
     * @brief Stores Mk' lam_star.
     */
    c_float* u;

    /**
     * @brief Dual objective value.
     */
    c_float fval;

    /**
     * @brief Unit lower factor of Mk Mk' = L D L'.
     */
    c_float* L;

    /**
     * @brief Diagonal of the LDL' factorization.
     */
    c_float* D;

    /**
     * @brief Solution of L xldl = -dk.
     */
    c_float* xldl;

    /**
     * @brief zldl_i = xldl_i / D_i.
     */
    c_float* zldl;

    /**
     * @brief Prefix of the LDL' system that can be reused.
     */
    int reuse_ind;

    /**
     * @brief Working set, capacity n + ns + 1.
     */
    int* WS;

    /**
     * @brief Number of active constraints.
     */
    int n_active;

    /**
     * @brief Active-set iterations of the current solve.
     */
    int iterations;

    /**
     * @brief Index of a singular working-set row, or DAQP_EMPTY_IND.
     */
    int sing_ind;

    /**
     * @brief Mask of directions that needed proximal regularization.
     */
    int* prox_mask;

    /**
     * @brief Number of directions that needed regularization.
     */
    int n_prox;

    /**
     * @brief Largest violation of a soft constraint in the returned solution.
     */
    c_float soft_slack;

    /**
     * @brief Solver settings; not owned unless AllocateDaqpSettings ran.
     */
    DAQPSettings* settings;

    /**
     * @brief Branch-and-bound state, or NULL.
     */
    DAQPBnB* bnb;

    /**
     * @brief Number of hierarchical levels.
     */
    int nh;

    /**
     * @brief Hierarchical level breaks.
     */
    int* break_points;

    /**
     * @brief Affine variational inequality state, or NULL.
     */
    DAQPAVI* avi;

    /**
     * @brief Equality elimination state, or NULL when the problem is not
     *        reduced.
     */
    DAQPEqElim* eq;

    /**
     * @brief Wall-clock timer used for the time limit, set by DaqpSolve.
     */
    void* timer;

    /**
     * @brief M u from the latest feasibility scan (length m - ms).
     *
     * NULL disables batching.
     */
    c_float* Mu;

    /**
     * @brief Reciprocal quadratic soft weight on the lower side.
     *
     * Defaults to settings->rho_soft when the entry is zero.
     */
    c_float* rho_ls;

    /**
     * @brief Reciprocal quadratic soft weight on the upper side.
     */
    c_float* rho_us;

    /**
     * @brief Linear soft weight on the lower side.
     *
     * Defaults to settings->w_soft when the entry is zero.
     */
    c_float* w_ls;

    /**
     * @brief Linear soft weight on the upper side.
     */
    c_float* w_us;

    /**
     * @brief Workspace state bits.
     */
    int state;
} DAQPWorkspace;

/**
 * @brief True when the workspace describes a hierarchical quadratic program.
 *
 * @param work Workspace pointer.
 */
#define DAQP_IS_HIERARCHICAL(work) \
    ((work)->break_points != NULL && (work)->nh > 1)


}  // namespace miqp
}  // namespace math
}  // namespace common
}  // namespace autonomy

#endif
