#ifndef IIT_RBD_COMPACT_TRANSFORM_H_
#define IIT_RBD_COMPACT_TRANSFORM_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/StateDependentBase.h>
#include <iit/rbd/internals.h>
//#include <iostream>

namespace iit {
namespace rbd {


// // Just a dummy type to investigate what happens with temporary vectors with
// // the code below. Using this type should confirm that no useless copies of
// // vectors are performed, even without optimizations (g++ 5.4 (onwards?)).
// // See instead what happens when deliberately compiling with -fno-elide-constructors
//template<typename Scalar>
//struct Vec6Probe : public Vec6<Scalar>
//{
//    Vec6Probe() {
//        std::cout << "CTor" << std::endl;
//    }
//    Vec6Probe(const Vec6Probe<Scalar>& rhs) {
//        std::cout << "Copy CTor" << std::endl;
//    }
//};

/** A compact type that can act as different representations of a coordinate
 * transform.
 *
 * An instance contains the minimum amount of data to encode a coordinate
 * transform for homogeneous coordinates and spatial vectors (both motion and
 * force): a rotation matrix and a translation vector.
 *
 * Different member functions implement the actual coordinate transformation for
 * the different representations.
 */
template<typename Scalar>
struct CTransformCore
{
    using vector6 = Vec6<Scalar>;
    //using vector6 = Vec6Probe<Scalar>; // use this to investigate about temporaries
    using matrix4 = PlainMatrix<Scalar, 4, 4>;
    using matrix6 = PlainMatrix<Scalar, 6, 6>;

    CTransformCore() {}
    explicit CTransformCore(int) : a_R_b(Mat33<Scalar>::Zero()), r_ab_a(0,0,0) {}

    template <typename Derived>
    vector6 A_XM_B(const MatrixBase<Derived>& v_b) const
    {
        using Scalar2 = typename Derived::Scalar;
        vector6 v_a;
        angularPart(v_a) = a_R_b * angularPart<Scalar2>(v_b);
        linearPart(v_a)  = internal::cross(r_ab_a, angularPart(v_a)) + a_R_b *linearPart<Scalar2>(v_b);
        return v_a;
    }

    template <typename Derived>
    vector6  B_XM_A(const MatrixBase<Derived>& v_a) const
    {
        using Scalar2 = typename Derived::Scalar;
        vector6 v_b;
        angularPart(v_b) = a_R_b.transpose() * angularPart<Scalar2>(v_a);
        linearPart(v_b)  = a_R_b.transpose() * (internal::cross(-r_ab_a, angularPart<Scalar2>(v_a)) + linearPart<Scalar2>(v_a));
        return v_b;
    }

    template <typename Derived>
    vector6  A_XF_B(const MatrixBase<Derived>& v_B) const
    {
        using Scalar2 = typename Derived::Scalar;
        vector6 v_A;
        linearPart(v_A) = a_R_b * linearPart<Scalar2>(v_B);
        angularPart(v_A)=  internal::cross(r_ab_a, linearPart(v_A)) + a_R_b * angularPart<Scalar2>(v_B);
        return v_A;
    }

    template <typename Derived>
    vector6  B_XF_A(const MatrixBase<Derived>& v_A) const
    {
        using Scalar2 = typename Derived::Scalar;
        vector6 v_B;
        linearPart(v_B) = a_R_b.transpose() * linearPart<Scalar2>(v_A);
        angularPart(v_B)= a_R_b.transpose() * (internal::cross(-r_ab_a, linearPart<Scalar2>(v_A)) + angularPart<Scalar2>(v_A));
        return v_B;
    }

    template <typename Derived>
    Vec3<Scalar> A_XH_B(const MatrixBase<Derived>& v_b) const
    {
        return a_R_b * v_b + r_ab_a;
    }

    template <typename Derived>
    Vec3<Scalar> B_XH_A(const MatrixBase<Derived>& v_a) const
    {
        return a_R_b.transpose() * (v_a - r_ab_a);
    }


#define block31 template block<3,1>
#define block33 template block<3,3>
    // there might be better ways to do the following functions in Eigen
    // (e.g. with nullary expressions), but for now I don't care

    matrix6 A_XM_B_matrix() const
    {
        matrix6 ret;
        ret.block33(AX,AX) = ret.block33(LX,LX) = a_R_b;
        ret.block33(LX,AX) = rx() * a_R_b;
        ret.block33(AX,LX).setZero();
        return ret;
    }
    matrix6 B_XM_A_matrix() const
    {
        matrix6 ret;
        ret.block33(AX,AX) = ret.block33(LX,LX) = a_R_b.transpose();
        ret.block33(LX,AX) = a_R_b.transpose() * rx().transpose();
        ret.block33(AX,LX).setZero();
        return ret;
    }

    matrix6 A_XF_B_matrix() const
    {
        matrix6 ret;
        ret.block33(AX,AX) = ret.block33(LX,LX) = a_R_b;
        ret.block33(AX,LX) = rx() * a_R_b;
        ret.block33(LX,AX).setZero();
        return ret;
    }
    matrix6 B_XF_A_matrix() const
    {
        matrix6 ret;
        ret.block33(AX,AX) = ret.block33(LX,LX) = a_R_b.transpose();
        ret.block33(AX,LX) = a_R_b.transpose() * rx().transpose();
        ret.block33(LX,AX).setZero();
        return ret;
    }

    matrix4 A_XH_B_matrix() const
    {
        matrix4 ret;
        ret.block33(0,0) = a_R_b;
        ret.block31(0,3) = r_ab_a;
        ret.row(3) << 0,0,0,1;
        return ret;
    }
    matrix4 B_XH_A_matrix() const
    {
        matrix4 ret;
        ret.block33(0,0) = a_R_b.transpose();
        ret.block31(0,3) = - a_R_b.transpose() * r_ab_a;
        ret.row(3) << 0,0,0,1;
        return ret;
    }

#undef block31
#undef block33

    /** The rotation matrix from 'b' to 'a' coordinates **/
    Mat33<Scalar> a_R_b;

    /** The position vector from the origin of 'a' to the origin of 'b', in
        'a' coordinates **/
    Vec3 <Scalar> r_ab_a;

private:
    Mat33<Scalar> rx() const {
        Mat33<Scalar> ret;
        ret <<  0        , -r_ab_a(Z),  r_ab_a(Y),
                r_ab_a(Z),   0       , -r_ab_a(X),
               -r_ab_a(Y),  r_ab_a(X),   0;
        return ret;
    }
};


// Define a type for each of the product functions defined above; this type
// can then overload the product operator and implement it with its
// corresponding function.
// For example, for the type 'A_XM_B', we have that:
//
//  A_XM_B::operator*()  <---->   CTransformCore::A_XM_B()
//
// This will enable us to make a TransformBase type (see below) which can use
// the nicer looking operator*.
//
// We use a parametrized macro, and then instantiate it a number of times:

#define STRUCT_DEFINING_OPERATOR_STAR(KEY)                         \
    template<typename Scalar>                                               \
    struct KEY                              \
    {                                                                       \
        const CTransformCore<Scalar>& data;\
        template <typename Derived>                                         \
        auto operator*(const MatrixBase<Derived>& v_in) const -> decltype( std::declval< CTransformCore<Scalar> >().KEY(v_in) ) \
        {                                                                     \
            return data.KEY(v_in);                         \
        }                                                                     \
        auto matrix() const -> decltype(std::declval< CTransformCore<Scalar> >().KEY ## _matrix() ) \
        {                                                                     \
            return data.KEY ## _matrix();                  \
        }                                                                     \
    };

// The 'decltype' above make sure to use the same return type as the actual
// product function that we call, in CTransformCore. This ensures consistency
// if we change the typedef(s) inside CTransformCore.


STRUCT_DEFINING_OPERATOR_STAR(A_XM_B)
STRUCT_DEFINING_OPERATOR_STAR(B_XM_A)
STRUCT_DEFINING_OPERATOR_STAR(A_XF_B)
STRUCT_DEFINING_OPERATOR_STAR(B_XF_A)
STRUCT_DEFINING_OPERATOR_STAR(A_XH_B)
STRUCT_DEFINING_OPERATOR_STAR(B_XH_A)


template<typename STATE, typename Actual>
struct TransformBase : public StateDependentBase<STATE, Actual>
{
    TransformBase() {}
    explicit TransformBase(int foo) : ct(foo) {}

    CTransformCore<typename STATE::Scalar> ct;

    template<typename REPR>
    REPR as() const {
        return REPR{ct};
    }
};

}
}



#endif
