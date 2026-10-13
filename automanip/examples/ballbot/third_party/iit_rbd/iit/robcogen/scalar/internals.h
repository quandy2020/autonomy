/*
 * internals.h
 *
 *  Created on: Jun 6, 2018
 *      Author: marco
 */

#ifndef ROOT_IIT_ROBCOGEN_SCALAR_INTERNALS_H_
#define ROOT_IIT_ROBCOGEN_SCALAR_INTERNALS_H_

#include <type_traits>
#include "../../rbd/scalar_traits.h"

namespace iit {
namespace robcogen {

namespace internal {

// ScalarTraitsSelector is a metafunction that extracts the ScalarTraits
// from a given a robot-traits. If the robot-traits does not define a
// ScalarTraits field, the metafunction resolves to the default traits for the
// double numeric type.
// This selector is provided purely for backwards compatibility with older
// versions of RobCoGen, which generate a robot-traits struct that does not
// include the ScalarTraits field.

// Usage example:
// typedef typename internal::ScalarTraitsSelector< ROBOT_TRAITS >::trait RobotScalarTrait;

// For some explanation of the mechanism, see Wikipedia's page about SFINAE,


// This alias always resolves to 'void'
template<typename ... > using void_t = void;

// This alias resolves to void if the template argument defines a 'ScalarTraits'
// field; otherwise the template argument substitution would fail.
template<typename ROBOT_TRAITS> using VoidIfDef = void_t<typename ROBOT_TRAITS::ScalarTraits>;


// #1: Primary template, "returns" the default traits
template <typename ROBOT_TRAITS, typename = void>
struct ScalarTraitsSelector
{
    typedef typename iit::rbd::DoubleTraits trait; // "default value"
};

// #2: A _partial specialization_ that resolves to ...<ROBOT_TRAITS, void> when
// ROBOT_TRAITS::ScalarTraits exists.
template <typename ROBOT_TRAITS>
struct ScalarTraitsSelector<ROBOT_TRAITS, VoidIfDef<ROBOT_TRAITS>  >
{
    typedef typename ROBOT_TRAITS::ScalarTraits trait;
};

// In my understanding, the trick works because an explicit template argument
// is preferred over a default argument.
// This client code:
//
//   ScalarTraitsSelector<RT>
//
// results in the primary template match with <RT, =void>
// If the robot-traits RT does define 'ScalarTraits', the partial specialization
// is <RT, void>, therefore it also matches and it is selected because preferred
// over <RT, =void>
// If RT does _not_ define 'ScalarTraits', then matching #2 fails in the first
// place, and the only candidate is the primary template #1.


}
}
}



#endif /* ROOT_IIT_ROBCOGEN_SCALAR_INTERNALS_H_ */
