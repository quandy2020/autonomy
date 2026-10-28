# Third-party notices

Atlas contains algorithms migrated from ORB-SLAM3 and retains the upstream
GPLv3 license in `COPYING`. The ORB extractor is derived from OpenCV ORB code;
the internal MLPnP solver is derived from OpenGV. Their upstream notices remain
in the corresponding `.hpp` and `.cpp` sources.

Atlas does not vendor a Lie-group implementation. SO(3) and SE(3) come from
`autonomy/common/math`, while Atlas provides its small Sim(3) adapter in
`atlas/common/math/sim3.hpp`.

Atlas uses the **g2o** graph-optimization library and the **FBow** binary
bag-of-words library supplied as system CONFIG packages under `/usr/local` in
the SpaceHero dependency image. Their source code and public headers are not
vendored in the Atlas source tree.

Atlas additionally links OpenCV (Apache-2.0), Eigen (MPL2), Boost, OpenSSL,
AutoMsgs and Autolink as provided by the autonomy build environment.
