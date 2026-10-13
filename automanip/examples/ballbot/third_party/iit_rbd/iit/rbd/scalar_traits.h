#ifndef IIT_RBD_TPL_SCALAR_TRAITS_H
#define IIT_RBD_TPL_SCALAR_TRAITS_H

#include "rbd.h"

namespace iit {
namespace rbd {


template<typename T>
struct BasicTraits
{
    typedef T ValueType;
    typedef T Scalar;
};

template struct BasicTraits<double>;
template struct BasicTraits<float>;


template<typename BasicTrait>
struct ScalarTraitsCommons
{
    typedef typename BasicTrait::Scalar    Scalar;
    typedef typename BasicTrait::ValueType ValueType;

    typedef typename rbd::Core<Scalar> CoreTypes;
    template<int R, int C> using PlainMatrix = rbd::PlainMatrix<Scalar, R, C>;

    template <int Dims>
    inline static PlainMatrix<Dims, 1> solve(
            const PlainMatrix<Dims, Dims>& A,
            const PlainMatrix<Dims, 1>& b)
    {
        return A.inverse()*b;
    }
};


namespace internal {

template<typename Float_t>
struct FloatPointFuncs
{
    // I believe 'constexpr' is automatically "dropped" should the template
    // argument be a non-literal type (or a "non-constexpr-constructible" type,
    // whichever is more correct). So it is fine to have constexpr, for those
    // types that would allow it.
    // I cannot find a reference for this topic, though.
    inline static constexpr Float_t sin (const Float_t& x) { return std::sin(x);  }
    inline static constexpr Float_t cos (const Float_t& x) { return std::cos(x);  }
    inline static constexpr Float_t tan (const Float_t& x) { return std::tan(x);  }
    inline static constexpr Float_t sinh(const Float_t& x) { return std::sinh(x); }
    inline static constexpr Float_t cosh(const Float_t& x) { return std::cosh(x); }
    inline static constexpr Float_t tanh(const Float_t& x) { return std::tanh(x); }
    inline static constexpr Float_t exp (const Float_t& x) { return std::exp(x);  }
    inline static constexpr Float_t abs (const Float_t& x) { return std::abs(x);  }
    inline static constexpr Float_t fabs(const Float_t& x) { return std::fabs(x); }
    inline static constexpr Float_t sqrt(const Float_t& x) { return std::sqrt(x); }
};



} // namespace internal



template<typename SCALAR>
struct ScalarTraits : public ScalarTraitsCommons< BasicTraits<SCALAR> >,
                      public internal::FloatPointFuncs<SCALAR>
{
};

// Explicit _instantiation_ of the traits for double and float
template struct ScalarTraits<double>;
template struct ScalarTraits<float>;

typedef ScalarTraits<double> DoubleTraits;
typedef ScalarTraits<float>  FloatTraits;



}
}


#endif

