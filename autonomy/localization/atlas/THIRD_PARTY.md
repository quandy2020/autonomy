# Third-party notices

Atlas contains algorithms migrated from ORB-SLAM3 and retains the upstream
GPLv3 license in `COPYING`. The ORB extractor is derived from OpenCV ORB code;
the internal MLPnP solver is derived from OpenGV. Their upstream notices remain
in the corresponding `.hpp` and `.cpp` sources.

The `thirdparty/` directory contains:

- **FBow** — MIT-licensed binary bag-of-words implementation. Atlas links the
  FBow library supplied by the autonomy dependency image and keeps matching
  public headers under `thirdparty/FBow/include/fbow`.
- **g2o** — BSD-licensed graph optimization code retained with upstream file
  names and namespaces.
- **Sophus** — MIT-licensed Lie-group implementation retained with upstream
  file names and namespace.

Atlas additionally links OpenCV (Apache-2.0), Eigen (MPL2), Boost, OpenSSL,
AutoMsgs and Autolink as provided by the autonomy build environment.
