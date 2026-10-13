# Prepend this install prefix. Source from bash.
_autocmake_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_autocmake_prefix="$(cd "${_autocmake_here}/../../.." && pwd)"
export PATH="${_autocmake_prefix}/bin${PATH:+:${PATH}}"
export PYTHONPATH="${_autocmake_prefix}/lib/python${PYTHONPATH:+:${PYTHONPATH}}"
if [ "$(uname)" = "Darwin" ]; then
  export DYLD_LIBRARY_PATH="${_autocmake_prefix}/lib${DYLD_LIBRARY_PATH:+:${DYLD_LIBRARY_PATH}}"
else
  export LD_LIBRARY_PATH="${_autocmake_prefix}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
fi
unset _autocmake_here
unset _autocmake_prefix
