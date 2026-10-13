
#ifndef IIT_ROBCOGEN_JACOBIANS_H_
#define IIT_ROBCOGEN_JACOBIANS_H_

#include <iit/rbd/rbd.h>
#include "macros.h"

namespace iit {
namespace robcogen {

namespace internal {

template<typename Derived>
inline
rbd::MatrixBase<Derived>& constCast(const rbd::MatrixBase<Derived>& x) {
    return const_cast<rbd::MatrixBase<Derived>&>(x);
}

}

#define block3x1 template block<3,1>

/**
 * \name Columns of geometric Jacobians
 */
///@{
/**
 * \param[in] poi Point-of-Interest, the Cartesian coordinates of the point whose
 *        velocity is of interest, expressed in some reference frame B
 * \param[in] jointOrigin, the Cartesian coordinates of the origin of the reference
 *        frame of the joint, expressed in the same reference frame B.
 * \param[in] jointAxis, the 3D unit vector aligned with the joint axis, expressed
 *        in the same reference frame B.
 * \param[out] column the column of the Jacobian corresponding to the joint
 */
template<typename Derived>
inline void geometricJacobianColumn_revolute(
        const typename rbd::Vec3<typename Derived::Scalar> & poi,
        const typename rbd::Vec3<typename Derived::Scalar>& jointOrigin,
        const typename rbd::Vec3<typename Derived::Scalar>& jointAxis,
        const rbd::MatrixBase<Derived>& column)
{
    iit_rbd_matrix_assert(column.rows() == 6  &&  column.cols() == 1);
    internal::constCast(column).block3x1(rbd::AX,0) = jointAxis;
    internal::constCast(column).block3x1(rbd::LX,0) = jointAxis.cross(poi - jointOrigin);
}

/**
 *
 */
template<typename Derived>
inline void geometricJacobianColumn_prismatic(
        const typename rbd::Vec3<typename Derived::Scalar>& jointAxis,
        const rbd::MatrixBase<Derived>& column)
{
    iit_rbd_matrix_assert(column.rows() == 6  &&  column.cols() == 1);
    internal::constCast(column).block3x1(rbd::AX,0).setZero();
    internal::constCast(column).block3x1(rbd::LX,0) = jointAxis;
}
///@}

#undef block3x1

}
}


#endif
