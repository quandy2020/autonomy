This the C++ "iit_rbd" library. It contains some basic types and functions
related to spatial vector algebra and Rigid Body Dynamics (RBD).

This library was developed as part of the [RobCoGen project](https://robcogenteam.bitbucket.io/index.html),
and it was designed as the common ground of the C++ code that RobCoGen
generates.

Although it is not meant to be a user library, it can perfectly serve user
code that needs e.g. a type for spatial vectors or for the spatial 6x6
inertia tensor.

# Documentation

See the documentation comments in the source code. Your starting point should
be the [`iit/rbd/rbd.h`](iit/rbd/rbd.h) header file.

# Installation
This is a header only library. Installing is optional and just means copying the
headers in a location of your choice.

On Linux, you can install in the system include path with `./install.sh`

# Dependencies

  - [Eigen](https://eigen.tuxfamily.org)

# License
Copyright (c) 2015-2024, Marco Frigerio

Distributed under the BSD 2-clause license. See the `LICENSE` file for more
details.
