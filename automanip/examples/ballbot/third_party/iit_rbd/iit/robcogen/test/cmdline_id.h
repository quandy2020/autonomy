#ifndef ROBCOGEN_TEST_ID_H
#define ROBCOGEN_TEST_ID_H

#include <cmath>
#include <iostream>
#include <fstream>

#include <sstream>
#include <string>

#include "fext.h"

#include <iit/rbd/rbd.h>
#include <iit/robcogen/utils.h>
#include <iit/robcogen/scalar/internals.h>


namespace iit {
namespace robcogen {
namespace test {

/**
 *
 */
template<class ROB>
void cmdline_id(int argc, char** argv, typename ROB::InvDynEngine& id)
{
    //
    typename ROB::JointState q, qd, qdd, tau;
    robcogen::utils::cmdlineargs_jstate<ROB>(argc-1, &(argv[1]), q, qd, qdd);

    typedef typename internal::ScalarTraitsSelector< ROB >::trait::Scalar Scalar;
    //
    int c = ROB::joints_count * 3 + 1;
    typename ROB::InvDynEngine::ExtForces fext(rbd::Force<Scalar>::Zero());
    if(argc > c) {
        std::string extForcesFile(argv[c]);
        robcogen::test::readExtForces<ROB>(extForcesFile, fext);
    }

    id.id(tau, q, qd, qdd, fext);
	std::cout << tau << std::endl;
}

template<class ROB>
void cmdline_id(int argc, char** argv)
{
    typename ROB::MotionTransforms  xm;
    typename ROB::InertiaProperties ip;
    typename ROB::InvDynEngine      id(ip, xm);
    cmdline_id<ROB> (argc, argv, id);
}

/**
 *
 */
template<class ROB>
void cmdline_id_fb(int argc, char** argv, typename ROB::InvDynEngine& id)
{
    //
    typename ROB::JointState q, qd, qdd, tau, tau2;
    robcogen::utils::cmdlineargs_jstate<ROB>(argc-1, &(argv[1]), q, qd, qdd);

    //
    rbd::VelocityVector v0, a0, grav;
    a0.setZero();
    v0.setZero();
    grav.setZero();
    grav(rbd::LZ) = -iit::rbd::g;

    // the number of "consumed" arguments so far
    int arg = ROB::joints_count * 3 + 1;

    v0(rbd::AX) = std::atof(argv[arg++]);
    v0(rbd::AY) = std::atof(argv[arg++]);
    v0(rbd::AZ) = std::atof(argv[arg++]);
    v0(rbd::LX) = std::atof(argv[arg++]);
    v0(rbd::LY) = std::atof(argv[arg++]);
    v0(rbd::LZ) = std::atof(argv[arg++]);

    //
    typename ROB::InvDynEngine::ExtForces fext(rbd::ForceVector::Zero());
    if(argc > arg) {
        std::string extForcesFile(argv[arg]);
        robcogen::test::readExtForces<ROB>(extForcesFile, fext);
    }

    id.id(tau, a0, grav, v0, q, qd, qdd, fext);
    std::cout << a0 << std::endl;
    std::cout << tau << std::endl;
}

template<class ROB>
void cmdline_id_fb(int argc, char** argv)
{
    typename ROB::MotionTransforms  xm;
    typename ROB::InertiaProperties ip;
    typename ROB::InvDynEngine      id(ip, xm);
    cmdline_id_fb<ROB> (argc, argv, id);
}

}
}
}

#endif
