#ifndef IIT_ROBCOGEN_TEST_JSIM_FB_H
#define IIT_ROBCOGEN_TEST_JSIM_FB_H

#include <cmath>
#include <iostream>
#include <fstream>

#include <sstream>
#include <string>

#include "fext.h"

#include <iit/rbd/rbd.h>
#include <iit/robcogen/utils.h>


namespace iit {
namespace robcogen {
namespace test {

template<class ROB>
void cmdline_jsim(int argc, char** argv, typename ROB::JSIM& jsim)
{
    //
    typename ROB::JointState q;
    robcogen::utils::cmdlineargs_jstate<ROB>(argc-1, &(argv[1]), q);

    // a couple of "void" calls to make sure the result stays consistent
    jsim(q);
    jsim(q);
    std::cout << jsim(q) << std::endl;
}

template<class ROB>
void cmdline_jsim(int argc, char** argv)
{
    typename ROB::ForceTransforms   xf;
    typename ROB::InertiaProperties ip;
    typename ROB::JSIM              jsim(ip, xf);
    cmdline_jsim<ROB>(argc, argv, jsim);
}

}
}
}

#endif
