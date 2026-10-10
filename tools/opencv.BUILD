# @autonomy_opencv — system OpenCV from install_deps (install_opencv.sh).
#
# BCR opencv 4.13+/5.x forces eigen 5 + protobuf ≥33, which breaks the root
# grpc@1.70 / protobuf graph. Prefer the CMake-installed tree instead.
#
# Only opencv{5,4,2} header trees are symlinked (see repositories.bzl) — never
# the whole $PREFIX/include (that also ships autonomy/ headers).

load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "opencv",
    hdrs = glob(
        [
            "include/opencv5/**",
            "include/opencv4/**",
            "include/opencv2/**",
        ],
        allow_empty = True,
    ),
    includes = [
        "include/opencv5",
        "include/opencv4",
    ],
    linkopts = [
        "-Lprefix/lib",
        "-Wl,-rpath,prefix/lib",
        "-lopencv_core",
        "-lopencv_imgproc",
        "-lopencv_imgcodecs",
        "-lopencv_highgui",
        "-lopencv_calib3d",
        "-lopencv_features2d",
        "-lopencv_flann",
        "-lopencv_video",
        "-lopencv_videoio",
        "-lopencv_photo",
        "-lopencv_ml",
        "-lopencv_objdetect",
    ],
)
