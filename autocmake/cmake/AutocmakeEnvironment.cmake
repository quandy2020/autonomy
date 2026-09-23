# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

include_guard(GLOBAL)

# @brief Install a relocatable environment hook for this package.
#
# Files land in share/<package>/environment/. library_path.dsv uses the ament
# dsv prefixes (PATH, LD_LIBRARY_PATH, DYLD_LIBRARY_PATH). library_path.sh can
# be sourced directly; it finds the prefix from its own path.
function(_autocmake_env)
  set(_dir "${CMAKE_CURRENT_BINARY_DIR}/autocmake_environment")
  file(MAKE_DIRECTORY "${_dir}")
  set(_bin "${CMAKE_INSTALL_BINDIR}")
  set(_lib "${CMAKE_INSTALL_LIBDIR}")
  file(WRITE "${_dir}/library_path.dsv"
"prepend-non-duplicate;PATH;${_bin}
prepend-non-duplicate;LD_LIBRARY_PATH;${_lib}
prepend-non-duplicate;DYLD_LIBRARY_PATH;${_lib}
prepend-non-duplicate;PYTHONPATH;${_lib}/python
")
  file(WRITE "${_dir}/library_path.sh"
"# Prepend this install prefix. Source from bash.
_autocmake_here=\"\$(cd \"\$(dirname \"\${BASH_SOURCE[0]}\")\" && pwd)\"
_autocmake_prefix=\"\$(cd \"\${_autocmake_here}/../../..\" && pwd)\"
export PATH=\"\${_autocmake_prefix}/${_bin}\${PATH:+:\${PATH}}\"
export PYTHONPATH=\"\${_autocmake_prefix}/${_lib}/python\${PYTHONPATH:+:\${PYTHONPATH}}\"
if [ \"\$(uname)\" = \"Darwin\" ]; then
  export DYLD_LIBRARY_PATH=\"\${_autocmake_prefix}/${_lib}\${DYLD_LIBRARY_PATH:+:\${DYLD_LIBRARY_PATH}}\"
else
  export LD_LIBRARY_PATH=\"\${_autocmake_prefix}/${_lib}\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}\"
fi
unset _autocmake_here
unset _autocmake_prefix
")
  install(
    FILES "${_dir}/library_path.dsv" "${_dir}/library_path.sh"
    DESTINATION "share/${PROJECT_NAME}/environment")
endfunction()
