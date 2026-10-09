"""Qt6 helpers: moc + rcc genrules wired to the system Qt toolchain.

Uses `/usr/lib/qt6/libexec/{moc,rcc}` (Ubuntu / docker SDK layout). Headers
without `Q_OBJECT` produce an empty stub so callers may pass a broad glob.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

_MOC = "/usr/lib/qt6/libexec/moc"
_RCC = "/usr/lib/qt6/libexec/rcc"

def _safe_name(path):
    return path.replace("/", "_").replace(".", "_").replace("-", "_")

def qt_moc_srcs(name, hdrs):
    """Generate one `moc_*.cpp` per header; returns the list of outs."""
    outs = []
    for hdr in hdrs:
        safe = _safe_name(hdr)
        out = "moc_%s.cpp" % safe
        outs.append(out)
        native.genrule(
            name = "gen_moc_%s" % safe,
            srcs = [hdr],
            outs = [out],
            # Separate moc TUs (unlike CMake AUTOMOC include-into-.cpp) need
            # complete types for std::unique_ptr members in the header.
            cmd = """
              tmp="$@.tmp"
              if %s \
                  -I. \
                  -I/usr/include/x86_64-linux-gnu/qt6 \
                  -I/usr/include/x86_64-linux-gnu/qt6/QtCore \
                  -I/usr/include/qt6 \
                  "$(location %s)" -o "$$tmp" 2>/dev/null; then
                {
                  echo '#include <google/protobuf/message.h>'
                  echo '#include <memory>'
                  cat "$$tmp"
                } > "$@"
                rm -f "$$tmp"
              else
                printf '// no Q_OBJECT\\n' > "$@"
              fi
            """ % (_MOC, hdr),
        )
    native.filegroup(
        name = name,
        srcs = outs,
        visibility = ["//visibility:public"],
    )
    return outs

def qt_rcc_srcs(name, qrcs, data):
    """Generate one `qrc_*.cpp` per `.qrc`; returns the list of outs.

    `data` must list every file referenced by the qrc inputs so the sandbox
    preserves the tree next to each `.qrc`.
    """
    outs = []
    for qrc in qrcs:
        safe = _safe_name(qrc)
        out = "qrc_%s.cpp" % safe
        outs.append(out)
        native.genrule(
            name = "gen_rcc_%s" % safe,
            srcs = [qrc] + data,
            outs = [out],
            cmd = '%s "$(location %s)" -name %s -o "$@"' % (
                _RCC,
                qrc,
                safe,
            ),
        )
    native.filegroup(
        name = name,
        srcs = outs,
        visibility = ["//visibility:public"],
    )
    return outs

def qt_cc_library(
        name,
        srcs,
        hdrs,
        moc_hdrs,
        qrcs = [],
        qrc_data = [],
        deps = [],
        **kwargs):
    """`cc_library` that compiles `srcs` + generated moc/rcc together."""
    moc_outs = qt_moc_srcs(name + "_moc", moc_hdrs)
    rcc_outs = qt_rcc_srcs(name + "_rcc", qrcs, qrc_data) if qrcs else []
    cc_library(
        name = name,
        srcs = srcs + moc_outs + rcc_outs,
        hdrs = hdrs,
        deps = deps,
        **kwargs
    )
