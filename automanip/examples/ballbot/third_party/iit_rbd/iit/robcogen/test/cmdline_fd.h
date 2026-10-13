#ifndef ROBCOGEN_TEST_FD_H
#define ROBCOGEN_TEST_FD_H

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
void cmdline_fd(int argc, char** argv, typename ROB::FwdDynEngine& fd)
{
    //
    typename ROB::JointState q, qd, qdd, tau;
    robcogen::utils::cmdlineargs_jstate<ROB>(argc-1, &(argv[1]), q, qd, tau);

    typedef typename internal::ScalarTraitsSelector< ROB >::trait::Scalar Scalar;
    //
    int c = ROB::joints_count * 3 + 1;
    typename ROB::FwdDynEngine::ExtForces fext(rbd::Force<Scalar>::Zero());
    if(argc > c) {
        std::string extForcesFile(argv[c]);
        robcogen::test::readExtForces<ROB>(extForcesFile, fext);
    }
	
    fd.fd(qdd, q, qd, tau, fext);
	std::cout << qdd << std::endl;
}

template<class ROB>
void cmdline_fd(int argc, char** argv)
{
    typename ROB::MotionTransforms  xm;
    typename ROB::InertiaProperties ip;
    typename ROB::FwdDynEngine      fd(ip, xm);
    cmdline_fd<ROB>(argc, argv, fd);
}

/**
 *
 */
template<class ROB>
void cmdline_fd_fb(int argc, char** argv, typename ROB::FwdDynEngine& fd)
{
    //
    typename ROB::JointState q, qd, qdd, tau;
    robcogen::utils::cmdlineargs_jstate<ROB>(argc-1, &(argv[1]), q, qd, tau);

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
    typename ROB::FwdDynEngine::ExtForces fext(rbd::ForceVector::Zero());
    if(argc > arg) {
        std::string extForcesFile(argv[arg]);
        robcogen::test::readExtForces<ROB>(extForcesFile, fext);
    }

    fd.fd(qdd, a0, v0, grav, q, qd, tau, fext);
    std::cout << a0 << std::endl;
    std::cout << qdd << std::endl;
}

template<class ROB>
void cmdline_fd_fb(int argc, char** argv)
{
    typename ROB::MotionTransforms  xm;
    typename ROB::InertiaProperties ip;
    typename ROB::FwdDynEngine      fd(ip, xm);
    cmdline_fd_fb<ROB>(argc, argv, fd);
}

}
}
}

#endif
