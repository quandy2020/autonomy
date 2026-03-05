#! /usr/bin/env bash
TOP_DIR="$(cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd -P)"
source ${TOP_DIR}/scripts/autolink.bashrc

export AUTOLINK_BAZEL_DIST_DIR="${AUTOLINK_CACHE_DIR}/distdir"
export AUTOLINK_PATH="${AUTOLINK_ROOT_DIR}/autolink"

bazel_bin_path="${AUTOLINK_ROOT_DIR}/bazel-bin"
mainboard_path="${bazel_bin_path}/autolink/mainboard"
autolink_tool_path="${bazel_bin_path}/autolink/tools"
performance_path="${autolink_tool_path}/autolink_performance"
recorder_path="${autolink_tool_path}/autolink_recorder"
launch_path="${autolink_tool_path}/autolink_launch"
channel_path="${autolink_tool_path}/autolink_channel"
node_path="${autolink_tool_path}/autolink_node"
service_path="${autolink_tool_path}/autolink_service"
monitor_path="${autolink_tool_path}/autolink_monitor"
visualizer_path="${bazel_bin_path}/modules/tools/visualizer"

# TODO(all): place all these in one place and pathprepend
for entry in "${mainboard_path}" \
    "${recorder_path}" "${monitor_path}"  \
    "${channel_path}" "${node_path}" \
    "${service_path}" "${performance_path}" \
    "${launch_path}" \
    "${visualizer_path}" ; do
    pathprepend "${entry}"
done

pathprepend ${bazel_bin_path}/autolink/python/internal PYTHONPATH
pathprepend "${PYTHON_INSTALL_PATH}/lib/python${PYTHON_VERSION}/site-packages" PYTHONPATH
pathprepend "${PYTHON_INSTALL_PATH}/bin/" PATH

export AUTOLINK_DOMAIN_ID=80
export AUTOLINK_IP=127.0.0.1

export GLOG_log_dir="${AUTOLINK_ROOT_DIR}/data/log"
export GLOG_alsologtostderr=0
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0

export sysmo_start=0

# for DEBUG log
#export GLOG_v=4

source ${AUTOLINK_PATH}/tools/autolink_tools_auto_complete.bash
