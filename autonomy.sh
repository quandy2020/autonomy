#!/usr/bin/env bash
#
# autonomy.sh — modular CLI for the autonomy monorepo (Bazel + CMake domains).
#
# Quick start:
#   ./autonomy.sh help
#   ./autonomy.sh deps                               # third-party inventory
#   ./autonomy.sh modules                            # libs + binaries
#   ./autonomy.sh build
#   ./autonomy.sh build -m planning,control          # lib + autonomy.<d> binary
#   ./autonomy.sh build planning control             # same (domain names)
#   ./autonomy.sh build //autonomy/bridge:autonomy.bridge
#   ./autonomy.sh build autodriver:autodriver_bin    # package:target
#   ./autonomy.sh build --cmake -m planning          # CMake AUTONOMY_BUILD_*
#   ./autonomy.sh test
#   ./autonomy.sh install --prefix ./install
#   ./autonomy.sh install --cmake --prefix /opt/autonomy
#   ./autonomy.sh clean
#   ./autonomy.sh status
#
# Root Bazel: //:automsgs from sibling @automsgs (BCR protobuf);
# //:autolink.so + //:autonomy_headers from @autonomy_prefix (CMake).
#   export AUTONOMY_PREFIX=$PWD/build   # or install  (still needed for autolink)
#
# Naming conventions (this file):
#   list_* / is_* / require_*     discovery / predicates / guards
#   domain_*                      autonomy/<domain> helpers
#   bazel_package_*               sibling MODULE.bazel packages
#   build_* / test_* / run_*      actions
#   usage_* / cmd_*               CLI surface
#   parse_* / dispatch_* / main   argument plumbing
# Locals: snake_case; prefer full words (domain, package, label).
#
set -euo pipefail

readonly SCRIPT_NAME="$(basename "${BASH_SOURCE[0]}")"
readonly ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "${ROOT}"

# Workspace-local bazelisk (survives docker recreate when ROOT is bind-mounted).
readonly BAZELISK_CACHE_DIR="${ROOT}/.cache/bin"
readonly BAZELISK_VERSION="${BAZELISK_VERSION:-v1.25.0}"

# Prefer explicit BAZEL=, else bazel / bazelisk on PATH, else cached / common paths.
resolve_bazel() {
  if [[ -n "${BAZEL:-}" && "${BAZEL}" != "bazel" && "${BAZEL}" != "bazelisk" ]]; then
    printf '%s\n' "${BAZEL}"
    return
  fi
  if [[ -n "${BAZEL:-}" ]] && command -v "${BAZEL}" >/dev/null 2>&1; then
    command -v "${BAZEL}"
    return
  fi
  local candidate
  for candidate in \
      "${BAZELISK_CACHE_DIR}/bazel" \
      "${BAZELISK_CACHE_DIR}/bazelisk" \
      bazel bazelisk \
      /usr/local/bin/bazel /usr/local/bin/bazelisk \
      /usr/bin/bazel "${HOME}/.local/bin/bazel" "${HOME}/bin/bazel"; do
    if [[ "${candidate}" == */* ]]; then
      [[ -x "${candidate}" ]] && { printf '%s\n' "${candidate}"; return; }
    elif command -v "${candidate}" >/dev/null 2>&1; then
      command -v "${candidate}"
      return
    fi
  done
  printf '%s\n' "${BAZELISK_CACHE_DIR}/bazel"
}

# Download bazelisk into .cache/bin when no bazel is available (needs network once).
install_bazelisk() {
  local dest_dir="${BAZELISK_CACHE_DIR}"
  local dest="${dest_dir}/bazelisk"
  local arch asset url
  arch="$(uname -m)"
  case "${arch}" in
    x86_64|amd64) asset="bazelisk-linux-amd64" ;;
    aarch64|arm64) asset="bazelisk-linux-arm64" ;;
    *) die "unsupported arch for bazelisk: ${arch}" ;;
  esac
  url="https://github.com/bazelbuild/bazelisk/releases/download/${BAZELISK_VERSION}/${asset}"
  mkdir -p "${dest_dir}"
  info "install bazelisk ${BAZELISK_VERSION} → $(short_path "${dest}")"
  if command -v curl >/dev/null 2>&1; then
    curl -fsSL -o "${dest}" "${url}"
  elif command -v wget >/dev/null 2>&1; then
    wget -qO "${dest}" "${url}"
  else
    die "need curl or wget to download bazelisk"
  fi
  chmod +x "${dest}"
  ln -sfn "${dest}" "${dest_dir}/bazel"
  ok "bazelisk installed (${dest_dir}/bazel)"
}

BAZEL="$(resolve_bazel)"
JOBS="${JOBS:-$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 8)}"
PREFIX="${PREFIX:-${ROOT}/install}"
VERBOSE="${VERBOSE:-0}"
# auto | always | never  (also: NO_COLOR=1, FORCE_COLOR=1, CLICOLOR_FORCE=1)
COLOR_MODE="${COLOR_MODE:-auto}"

# Resolve AUTONOMY_PREFIX when unset (install → build → monorepo peers).
if [[ -z "${AUTONOMY_PREFIX:-}" ]]; then
  for candidate in \
    "${ROOT}/install" \
    "${ROOT}/build" \
    "${ROOT}/../../install/autonomy" \
    "${ROOT}/../../build/autonomy"
  do
    if [[ -f "${candidate}/lib/libautomsgs.so" ]]; then
      AUTONOMY_PREFIX="$(cd "${candidate}" && pwd)"
      break
    fi
  done
fi
export AUTONOMY_PREFIX="${AUTONOMY_PREFIX:-}"

# Middleware / third-party smoke labels (used by --smoke and as deps).
readonly DEFAULT_SMOKE_TARGETS=(
  //:automsgs
  //:autolink
  //:cpp_third_party
  //autonomy/common:autonomy_common
)

# Backward-compatible alias (package default when explicitly building "tools").
readonly DEFAULT_BUILD_TARGETS=("${DEFAULT_SMOKE_TARGETS[@]}")

# Bazel packages with their own MODULE.bazel ("tools" = repo root).
readonly BAZEL_PACKAGE_NAMES=(tools automsgs autodriver autoviz)

# CMake / autocmake packages (package.xml peers).
readonly CMAKE_PACKAGE_NAMES=(autonomy automsgs autolink autodriver autoviz)

# ---------------------------------------------------------------------------
# Color & layout
# ---------------------------------------------------------------------------

# Enable / disable ANSI styles from COLOR_MODE + env + TTY.
setup_color() {
  local enable=0
  case "${COLOR_MODE}" in
    always|on|yes|1) enable=1 ;;
    never|off|no|0)  enable=0 ;;
    auto|*)
      if [[ -n "${NO_COLOR:-}" ]]; then
        enable=0
      elif [[ -n "${FORCE_COLOR:-}" || -n "${CLICOLOR_FORCE:-}" ]]; then
        enable=1
      elif [[ -t 1 ]]; then
        enable=1
      else
        enable=0
      fi
      ;;
  esac

  if [[ "${enable}" -eq 1 ]]; then
    C_RESET=$'\033[0m'
    C_BOLD=$'\033[1m'
    C_DIM=$'\033[2m'
    C_RED=$'\033[31m'
    C_GREEN=$'\033[32m'
    C_YELLOW=$'\033[33m'
    C_BLUE=$'\033[34m'
    C_CYAN=$'\033[36m'
    USE_COLOR=1
  else
    C_RESET= C_BOLD= C_DIM= C_RED= C_GREEN= C_YELLOW= C_BLUE= C_CYAN=
    USE_COLOR=0
  fi
}

setup_color

# Wrap text in a style sequence (no-op when color off).
paint() {
  local style="$1"
  shift
  printf '%s%s%s' "${style}" "$*" "${C_RESET}"
}

# Fatal error to stderr, then exit 1.
die() {
  printf '%s: %s%s%s\n' \
    "${SCRIPT_NAME}" "${C_RED}${C_BOLD}error${C_RESET}: " "$*" >&2
  exit 1
}

# High-level progress line.
info() {
  printf '%s %s\n' "$(paint "${C_CYAN}${C_BOLD}" "==>")" "$*"
}

# Success line.
ok() {
  printf '%s %s\n' "$(paint "${C_GREEN}${C_BOLD}" " ok")" "$*"
}

# Indented detail under an info line.
note() {
  printf '     %s%s%s\n' "${C_DIM}" "$*" "${C_RESET}"
}

# Section header for inventory-style output (deps / modules / status).
section() {
  if [[ "${_SECTION_OPEN:-0}" == "1" ]]; then
    echo
  fi
  _SECTION_OPEN=1
  printf '%s%s%s\n' "${C_BOLD}${C_CYAN}" "$1" "${C_RESET}"
  if [[ "${USE_COLOR}" -eq 1 ]]; then
    printf '%s────────────────────────────────────────%s\n' "${C_DIM}" "${C_RESET}"
  fi
}

# Help / usage section title (no underline; tighter).
heading() {
  if [[ "${_HELP_OPEN:-0}" == "1" ]]; then
    echo
  fi
  _HELP_OPEN=1
  printf '%s%s%s\n' "${C_BOLD}${C_CYAN}" "$1" "${C_RESET}"
}

# Key / value row:  key········ value
kv() {
  local key="$1"
  local value="$2"
  printf '  %s%-22s%s %s\n' "${C_DIM}" "${key}" "${C_RESET}" "${value}"
}

# Table column header row (dim + bold).
th() {
  printf '  %s' "${C_DIM}${C_BOLD}"
  # shellcheck disable=SC2059
  printf "$@"
  printf '%s\n' "${C_RESET}"
}

# Dim separator line matching a printf format (e.g. dashes).
trule() {
  printf '  %s' "${C_DIM}"
  # shellcheck disable=SC2059
  printf "$@"
  printf '%s\n' "${C_RESET}"
}

# Colored ok / missing badges for status tables.
badge() {
  case "$1" in
    ok|present|yes)
      paint "${C_GREEN}" "ok"
      ;;
    missing|no|fail)
      paint "${C_RED}" "missing"
      ;;
    warn|partial)
      paint "${C_YELLOW}" "$1"
      ;;
    *)
      printf '%s' "$1"
      ;;
  esac
}

# Help: command name + description.
help_cmd() {
  printf '  %s%-10s%s %s\n' "${C_GREEN}" "$1" "${C_RESET}" "$2"
}

# Help: option line (flag already padded by caller in $1).
help_opt() {
  printf '  %s%-20s%s %s\n' "${C_YELLOW}" "$1" "${C_RESET}" "$2"
}

# Help: example command line.
help_ex() {
  printf '  %s%s%s\n' "${C_BLUE}" "$*" "${C_RESET}"
}

# Shorten absolute paths under HOME or ROOT for compact help / status.
short_path() {
  local path="$1"
  if [[ -z "${path}" ]]; then
    printf '\n'
  elif [[ -n "${ROOT}" && "${path}" == "${ROOT}" ]]; then
    printf '.\n'
  elif [[ -n "${ROOT}" && "${path}" == "${ROOT}"/* ]]; then
    printf '%s\n' "${path#"${ROOT}"/}"
  elif [[ -n "${HOME:-}" && "${path}" == "${HOME}"/* ]]; then
    printf '~/%s\n' "${path#"${HOME}"/}"
  else
    printf '%s\n' "${path}"
  fi
}

# ---------------------------------------------------------------------------
# Module discovery
# ---------------------------------------------------------------------------

# Print autonomy domain names, one per line.
# Prefers tools/package.bzl (AUTONOMY_DOMAIN_NAMES); else scans CMakeLists.txt.
list_domains() {
  local package_bzl="${ROOT}/tools/package.bzl"
  if [[ -f "${package_bzl}" ]]; then
    sed -n '/^AUTONOMY_DOMAIN_NAMES = \[/,/^\]/p' "${package_bzl}" \
      | sed -n 's/^[[:space:]]*"\([^"]*\)".*/\1/p'
    return 0
  fi
  local cmake_lists domain
  for cmake_lists in "${ROOT}"/autonomy/*/CMakeLists.txt; do
    [[ -f "${cmake_lists}" ]] || continue
    domain="$(basename "$(dirname "${cmake_lists}")")"
    if [[ -f "${ROOT}/autonomy/${domain}/AUTOCMAKE_IGNORE" ]] \
      || [[ -f "${ROOT}/autonomy/${domain}/COLCON_IGNORE" ]]; then
      continue
    fi
    printf '%s\n' "${domain}"
  done | sort
}

# True if autonomy/<domain>/BUILD.bazel exists.
domain_has_build() {
  [[ -f "${ROOT}/autonomy/$1/BUILD.bazel" ]]
}

# Map domain name → CMake cache flag (planning → AUTONOMY_BUILD_PLANNING).
domain_cmake_flag() {
  local upper
  upper="$(printf '%s' "$1" | tr '[:lower:]' '[:upper:]')"
  printf 'AUTONOMY_BUILD_%s' "${upper}"
}

# True if $1 is a known autonomy domain.
is_domain() {
  local want="$1" domain
  while IFS= read -r domain; do
    [[ "${domain}" == "${want}" ]] && return 0
  done < <(list_domains)
  return 1
}

# True if $1 is a known Bazel package name (tools|automsgs|...).
is_bazel_package() {
  local package
  for package in "${BAZEL_PACKAGE_NAMES[@]}"; do
    [[ "${package}" == "$1" ]] && return 0
  done
  return 1
}

# True if $1 is a known CMake / autocmake package.
is_cmake_package() {
  local package
  for package in "${CMAKE_PACKAGE_NAMES[@]}"; do
    [[ "${package}" == "$1" ]] && return 0
  done
  return 1
}

# Absolute directory of a Bazel package workspace.
bazel_package_dir() {
  case "$1" in
    tools|root|.) printf '%s\n' "${ROOT}" ;;
    automsgs)     printf '%s\n' "${ROOT}/automsgs" ;;
    autodriver)   printf '%s\n' "${ROOT}/autodriver" ;;
    autoviz)      printf '%s\n' "${ROOT}/autoviz" ;;
    *) die "unknown bazel package: $1" ;;
  esac
}

# True if the package links @autonomy_prefix (needs AUTONOMY_PREFIX).
bazel_package_needs_prefix() {
  case "$1" in
    root|.|tools|autodriver) return 0 ;;
    *) return 1 ;;
  esac
}

# Default bazel labels for a package (one per line).
bazel_package_default_targets() {
  case "$1" in
    tools|root|.) printf '%s\n' "${DEFAULT_BUILD_TARGETS[@]}" ;;
    *)            printf '%s\n' '//...' ;;
  esac
}

# ---------------------------------------------------------------------------
# Help
# ---------------------------------------------------------------------------

# Top-level usage summary.
usage_summary() {
  local bazel_short prefix_short autonomy_short
  bazel_short="$(short_path "${BAZEL}")"
  prefix_short="$(short_path "${PREFIX}")"
  autonomy_short="$(short_path "${AUTONOMY_PREFIX:-}")"
  [[ -n "${autonomy_short}" ]] || autonomy_short="(unset)"
  _HELP_OPEN=0

  printf '%s %s\n' \
    "$(paint "${C_BOLD}" "Usage:")" \
    "${SCRIPT_NAME} [options] <command> [args]"
  echo

  heading "Commands"
  help_cmd build   "Build packages / domains / labels"
  help_cmd install "Install build outputs to a prefix"
  help_cmd test    "Build, then bazel test"
  help_cmd clean   "bazel clean [--soft] [package…]"
  help_cmd modules "List packages and domain targets"
  help_cmd deps    "Third-party inventory (BCR + prefix)"
  help_cmd query   "bazel query  (default: //…)"
  help_cmd status  "Workspace / toolchain summary"
  help_cmd help    "This help, or: help <command>"

  heading "Options"
  help_opt "-j, --jobs N"     "Parallel jobs                 [${JOBS}]"
  help_opt "    --prefix DIR" "Install / stage prefix        [${prefix_short}]"
  help_opt "    --bazel PATH" "Bazel binary                  [${bazel_short}]"
  help_opt "    --color MODE" "auto | always | never         [${COLOR_MODE}]"
  help_opt "-v, --verbose"    "Print bazel / cmake commands"
  help_opt "-h, --help"       "Show help"

  heading "Environment"
  kv "AUTONOMY_PREFIX" "CMake prefix (libautomsgs.so)  [${autonomy_short}]"
  kv "COLOR_MODE"      "auto | always | never          [${COLOR_MODE}]"
  kv "also"            "JOBS  BAZEL  VERBOSE  PREFIX  NO_COLOR  FORCE_COLOR"

  heading "Examples"
  help_ex "${SCRIPT_NAME} status"
  help_ex "${SCRIPT_NAME} modules"
  help_ex "${SCRIPT_NAME} build"
  help_ex "${SCRIPT_NAME} build planning bridge"
  help_ex "${SCRIPT_NAME} build -m planning,control"
  help_ex "${SCRIPT_NAME} build //autonomy/bridge:autonomy.bridge"
  help_ex "${SCRIPT_NAME} build autodriver:autodriver_bin"
  help_ex "${SCRIPT_NAME} build --cmake -m perception,map"
  help_ex "${SCRIPT_NAME} install --prefix ./install"
  help_ex "${SCRIPT_NAME} -j 16 test autodriver"

  echo
  printf '%s %s\n' \
    "$(paint "${C_DIM}" "More:")" \
    "${SCRIPT_NAME} help build | install | test | …"
}

# Usage for the build command.
usage_build() {
  _HELP_OPEN=0
  printf '%s %s\n' \
    "$(paint "${C_BOLD}" "Usage:")" \
    "${SCRIPT_NAME} build [options] [target…]"
  echo

  heading "Targets (mixable)"
  kv "(none)"           "All domains + middleware (full default)"
  kv "--smoke"          "Quick check: automsgs autolink common only"
  kv "<domain>"         "//autonomy/<d>:autonomy_<d>  [+ autonomy.<d>]"
  kv "<package>"        "Entire MODULE workspace (//…)"
  kv "<package>:<name>" "Target inside a package"
  kv "//label …"        "Root-workspace Bazel labels"
  kv "all"              "All bazel packages (tools automsgs …)"
  kv "domains"          "All domain libs (+ binaries)"

  heading "Options"
  help_opt "-m, --module LIST" "Comma-separated domains"
  help_opt "--smoke"           "Only DEFAULT_SMOKE_TARGETS (fast)"
  help_opt "--cmake"           "Build via autocmake + AUTONOMY_BUILD_*"
  help_opt "--up-to"           "autocmake --packages-up-to"

  heading "Known"
  kv "Packages" "${BAZEL_PACKAGE_NAMES[*]}"
  kv "CMake"    "${CMAKE_PACKAGE_NAMES[*]}"
  kv "Domains"  "${SCRIPT_NAME} modules"

  heading "Examples"
  help_ex "${SCRIPT_NAME} build"
  help_ex "${SCRIPT_NAME} build --smoke"
  help_ex "${SCRIPT_NAME} build planning control"
  help_ex "${SCRIPT_NAME} build -m planning,bridge"
  help_ex "${SCRIPT_NAME} build domains"
  help_ex "${SCRIPT_NAME} build //autonomy/bridge:autonomy.bridge"
  help_ex "${SCRIPT_NAME} build autodriver:autodriver_bin"
  help_ex "${SCRIPT_NAME} build --cmake -m planning,control"
  help_ex "${SCRIPT_NAME} install --prefix ./install   # build ≠ install"
}

# Usage for the test command.
usage_test() {
  _HELP_OPEN=0
  printf '%s %s\n' \
    "$(paint "${C_BOLD}" "Usage:")" \
    "${SCRIPT_NAME} test [options] [target…]"
  echo
  printf '  %s\n' "Runs build resolution (same as build), then bazel test."

  heading "Options"
  help_opt "-m, --module LIST" "Domain list (same as build)"

  heading "Examples"
  help_ex "${SCRIPT_NAME} test"
  help_ex "${SCRIPT_NAME} test autodriver"
  help_ex "${SCRIPT_NAME} -j 16 test //autonomy/common/..."
}

# Usage for the clean command.
usage_clean() {
  _HELP_OPEN=0
  printf '%s %s\n' \
    "$(paint "${C_BOLD}" "Usage:")" \
    "${SCRIPT_NAME} clean [--soft] [package…]"

  heading "Modes"
  kv "(none)"    "Root workspace, bazel clean --expunge"
  kv "--soft"    "Keep repository cache (bazel clean)"
  kv "<package>" "tools | automsgs | autodriver | autoviz"
  kv "all"       "Every known bazel package"

  heading "Examples"
  help_ex "${SCRIPT_NAME} clean"
  help_ex "${SCRIPT_NAME} clean --soft tools"
  help_ex "${SCRIPT_NAME} clean all"
}

# Usage for the modules command.
usage_modules() {
  _HELP_OPEN=0
  printf '%s %s\n' \
    "$(paint "${C_BOLD}" "Usage:")" \
    "${SCRIPT_NAME} modules"
  echo
  printf '  %s\n' "• Bazel packages (MODULE.bazel)"
  printf '  %s\n' "• CMake / autocmake packages"
  printf '  %s\n' "• Domains → library, binaries, -DAUTONOMY_BUILD_<D>"
  echo
  printf '  %s %s\n' "$(paint "${C_DIM}" "Tip:")" "${SCRIPT_NAME} build -m planning"
}

# Usage for the deps command.
usage_deps() {
  _HELP_OPEN=0
  printf '%s %s\n' \
    "$(paint "${C_BOLD}" "Usage:")" \
    "${SCRIPT_NAME} deps"
  echo
  printf '  %s\n' "• BCR bazel_dep (MODULE.bazel / lock)"
  printf '  %s\n' "• @automsgs         (source + BCR protobuf)"
  printf '  %s\n' "• @autonomy_prefix  (autolink.so / autonomy headers)"
  printf '  %s\n' "• System labels     (//:pthread, //:opencv, …)"
  echo
  printf '  %s %s\n' \
    "$(paint "${C_DIM}" "Refresh:")" \
    "bazel mod deps --lockfile_mode=update"
}

# Usage for the query command.
usage_query() {
  _HELP_OPEN=0
  printf '%s %s\n' \
    "$(paint "${C_BOLD}" "Usage:")" \
    "${SCRIPT_NAME} query [-p package] [expression]"

  heading "Args"
  kv "(none)"             "//… in root workspace"
  kv "-p, --package NAME" "Query inside that package workspace"

  heading "Examples"
  help_ex "${SCRIPT_NAME} query"
  help_ex "${SCRIPT_NAME} query 'kind(\"cc_library\", //autonomy/...)'"
  help_ex "${SCRIPT_NAME} query -p autodriver //..."
}

# Usage for the status command.
usage_status() {
  _HELP_OPEN=0
  printf '%s %s\n' \
    "$(paint "${C_BOLD}" "Usage:")" \
    "${SCRIPT_NAME} status"
  echo
  printf '  %s\n' "Workspace path, Bazel version, AUTONOMY_PREFIX, jobs, packages, bazel-bin."
}

# Usage for the install command.
usage_install() {
  _HELP_OPEN=0
  printf '%s %s\n' \
    "$(paint "${C_BOLD}" "Usage:")" \
    "${SCRIPT_NAME} install [options] [DIR] [target…]"
  echo

  heading "Destination"
  kv "DIR / --prefix" "Install root (lib/ bin/ include/)  [default: ${PREFIX}]"
  kv "env PREFIX"     "Same as --prefix when DIR omitted"

  heading "Modes"
  kv "(default)" "Bazel: build then stage into prefix"
  kv "--cmake"   "autocmake build+install (AUTONOMY_INSTALL_BASE)"

  heading "Options"
  help_opt "--prefix DIR"      "Install destination"
  help_opt "--cmake"           "Install via CMake / autocmake"
  help_opt "--skip-build"      "Stage only (Bazel; skip rebuild)"
  help_opt "-m, --module LIST" "Domains to build/install (Bazel)"
  help_opt "--up-to"           "With --cmake: packages-up-to"

  heading "Layout"
  kv "lib/"     "libautonomy_*.a/.so  libautomsgs.*"
  kv "bin/"     "autonomy.* executables"
  kv "include/" "automsgs/ headers when present in bazel-bin"

  heading "Examples"
  help_ex "${SCRIPT_NAME} install"
  help_ex "${SCRIPT_NAME} install --prefix /opt/autonomy"
  help_ex "${SCRIPT_NAME} --prefix ./install install"
  help_ex "${SCRIPT_NAME} install /tmp/autonomy-prefix"
  help_ex "${SCRIPT_NAME} install -m planning,bridge --prefix ./install"
  help_ex "${SCRIPT_NAME} install --cmake --prefix ./install"
  help_ex "${SCRIPT_NAME} install --cmake --up-to autonomy --prefix ./install"
}

# Dispatch help for a named command (or summary when empty).
usage_command() {
  case "${1:-}" in
    build)   usage_build ;;
    install) usage_install ;;
    test)    usage_test ;;
    clean)   usage_clean ;;
    modules) usage_modules ;;
    deps)    usage_deps ;;
    query)   usage_query ;;
    status)  usage_status ;;
    help|"") usage_summary ;;
    *) usage_summary >&2; die "no help for unknown command: $1" ;;
  esac
}

# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

# Abort unless a Bazel binary is available (auto-install bazelisk into .cache/bin).
require_bazel() {
  if [[ -x "${BAZEL}" ]] || command -v "${BAZEL}" >/dev/null 2>&1; then
    return 0
  fi
  install_bazelisk
  BAZEL="$(resolve_bazel)"
  if [[ -x "${BAZEL}" ]] || command -v "${BAZEL}" >/dev/null 2>&1; then
    return 0
  fi
  die "bazel not found (install bazelisk, or set --bazel / BAZEL=)"
}

# Abort unless ROOT has MODULE.bazel.
require_root_module() {
  [[ -f "${ROOT}/MODULE.bazel" ]] \
    || die "not an autonomy Bazel workspace: ${ROOT}"
}

# Abort unless AUTONOMY_PREFIX contains libautomsgs.so.
require_prefix() {
  [[ -n "${AUTONOMY_PREFIX}" && -f "${AUTONOMY_PREFIX}/lib/libautomsgs.so" ]] \
    || die "AUTONOMY_PREFIX missing libautomsgs.so (export AUTONOMY_PREFIX=.../install)"
}

# Run bazel in workdir, forwarding AUTONOMY_PREFIX via --repo_env when set.
# --repo_env is a command option (after build|test|info|...), not a startup flag.
run_bazel_in() {
  local workdir="$1"
  shift
  local -a args=("$@")
  if [[ -n "${AUTONOMY_PREFIX}" && ${#args[@]} -gt 0 ]]; then
    args=("${args[0]}" "--repo_env=AUTONOMY_PREFIX=${AUTONOMY_PREFIX}" "${args[@]:1}")
  fi
  if [[ "${VERBOSE}" == "1" ]]; then
    info "+ (cd ${workdir}) ${BAZEL} ${args[*]}"
  fi
  (
    cd "${workdir}"
    "${BAZEL}" "${args[@]}"
  )
}

# Run bazel in the root workspace.
run_bazel() {
  run_bazel_in "${ROOT}" "$@"
}

# Print bazel-bin path for a workspace (cached symlink or `bazel info`).
bazel_bin_dir() {
  local workdir="${1:-${ROOT}}"
  if [[ -d "${workdir}/bazel-bin" ]]; then
    printf '%s\n' "${workdir}/bazel-bin"
  else
    run_bazel_in "${workdir}" info bazel-bin
  fi
}

# Split a comma-separated list; print one trimmed token per line.
split_csv() {
  local IFS=','
  # shellcheck disable=SC2086
  set -- $1
  local token
  for token in "$@"; do
    token="${token#"${token%%[![:space:]]*}"}"
    token="${token%"${token##*[![:space:]]}"}"
    [[ -n "${token}" ]] && printf '%s\n' "${token}"
  done
}

# Library label for a domain: //autonomy/<d>:autonomy_<d>.
domain_bazel_label() {
  printf '//autonomy/%s:autonomy_%s\n' "$1" "$1"
}

# Binary labels named autonomy.* declared in autonomy/<d>/BUILD.bazel.
domain_bazel_binaries() {
  local domain="$1"
  local build_file="${ROOT}/autonomy/${domain}/BUILD.bazel"
  [[ -f "${build_file}" ]] || return 0
  local target_name
  while IFS= read -r target_name; do
    [[ -n "${target_name}" ]] || continue
    printf '//autonomy/%s:%s\n' "${domain}" "${target_name}"
  done < <(sed -n 's/^[[:space:]]*name = "\(autonomy\.[^"]*\)".*/\1/p' "${build_file}")
}

# Library + binary labels for one domain (one label per line).
domain_bazel_labels() {
  local domain="$1"
  domain_bazel_label "${domain}"
  domain_bazel_binaries "${domain}"
}

# Print library + binary labels for many domains (one label per line).
# Bash 3.2-compatible (no nameref); callers append via `while read`.
domain_bazel_labels_for() {
  local domain
  for domain in "$@"; do
    domain_bazel_labels "${domain}"
  done
}

# Bazel-build domain libraries and any autonomy.* binaries.
build_bazel_domains() {
  local -a domains=("$@")
  local -a labels=()
  local domain label
  for domain in "${domains[@]}"; do
    is_domain "${domain}" || die "unknown domain module: ${domain} (see: ${SCRIPT_NAME} modules)"
    domain_has_build "${domain}" || die "domain '${domain}' has no autonomy/${domain}/BUILD.bazel"
  done
  while IFS= read -r label; do
    labels+=("${label}")
  done < <(domain_bazel_labels_for "${domains[@]}")
  require_root_module
  info "domains  ${domains[*]}"
  note "${labels[*]}"
  build_bazel_package tools "${labels[@]}"
}

# CMake-build selected domains via autocmake + AUTONOMY_BUILD_* flags.
build_cmake_domains() {
  local -a domains=("$@")
  local script="${ROOT}/scripts/build_autonomy_autocmake.sh"
  [[ -f "${script}" ]] || die "missing ${script}"

  local -a all_domains=()
  local domain
  while IFS= read -r domain; do
    all_domains+=("${domain}")
  done < <(list_domains)
  [[ ${#all_domains[@]} -gt 0 ]] || die "no autonomy/* domains found"

  local -a cmake_args=()
  for domain in "${all_domains[@]}"; do
    cmake_args+=("-D$(domain_cmake_flag "${domain}")=OFF")
  done
  for domain in "${domains[@]}"; do
    is_domain "${domain}" || die "unknown domain module: ${domain} (see: ${SCRIPT_NAME} modules)"
    cmake_args+=("-D$(domain_cmake_flag "${domain}")=ON")
  done

  info "cmake    domains  ${domains[*]}"
  note "${cmake_args[*]}"
  if [[ "${VERBOSE}" == "1" ]]; then
    info "+ ${script} select autonomy -- ${cmake_args[*]}"
  fi
  "${script}" select autonomy -- "${cmake_args[@]}"
}

# Resolve "package:target" and build inside that Bazel workspace.
build_bazel_package_target() {
  local spec="$1"
  local package="${spec%%:*}"
  local target="${spec#*:}"
  [[ -n "${package}" && -n "${target}" && "${package}" != "${spec}" ]] \
    || die "invalid package:target '${spec}'"

  local -a labels=()
  if [[ "${target}" == //* || "${target}" == @* ]]; then
    labels+=("${target}")
  elif [[ "${target}" == */* ]]; then
    labels+=("//${target}")
  else
    labels+=("//:${target}")
  fi

  if [[ "${package}" == "tools" || "${package}" == "root" \
     || "${package}" == "." || "${package}" == "autonomy" ]]; then
    require_root_module
    build_bazel_package tools "${labels[@]}"
  else
    is_bazel_package "${package}" || die "unknown bazel package '${package}' in '${spec}'"
    build_bazel_package "${package}" "${labels[@]}"
  fi
}

# Run autocmake for CMake packages (mode: select | up-to).
build_cmake_packages() {
  local mode="$1"
  shift
  local -a packages=("$@")
  local script="${ROOT}/scripts/build_autonomy_autocmake.sh"
  [[ -f "${script}" ]] || die "missing ${script}"
  [[ ${#packages[@]} -gt 0 ]] || die "cmake: need at least one package"

  info "cmake    ${mode}  ${packages[*]}"
  if [[ "${VERBOSE}" == "1" ]]; then
    info "+ ${script} ${mode} ${packages[*]}"
  fi
  "${script}" "${mode}" "${packages[@]}"
}

# bazel build inside a package workspace (default targets if none given).
build_bazel_package() {
  local package="$1"
  shift
  local -a targets=("$@")
  local package_dir
  package_dir="$(bazel_package_dir "${package}")"
  [[ -f "${package_dir}/MODULE.bazel" ]] || die "no MODULE.bazel in ${package_dir} (${package})"

  require_bazel
  if bazel_package_needs_prefix "${package}"; then
    require_prefix
  fi

  if [[ ${#targets[@]} -eq 0 ]]; then
    local -a defaults=()
    local target
    while IFS= read -r target; do
      defaults+=("${target}")
    done < <(bazel_package_default_targets "${package}")
    targets=("${defaults[@]}")
  fi

  info "build  ${package}  jobs=${JOBS}  $(short_path "${package_dir}")"
  note "${targets[*]}"
  if bazel_package_needs_prefix "${package}" && [[ -n "${AUTONOMY_PREFIX}" ]]; then
    export LD_LIBRARY_PATH="${AUTONOMY_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  fi
  run_bazel_in "${package_dir}" build --jobs="${JOBS}" "${targets[@]}"
  ok "build  ${package}  → $(short_path "$(bazel_bin_dir "${package_dir}")")"
}

# bazel test inside a package workspace (default //... if none given).
test_bazel_package() {
  local package="$1"
  shift
  local -a targets=("$@")
  local package_dir
  package_dir="$(bazel_package_dir "${package}")"
  [[ -f "${package_dir}/MODULE.bazel" ]] || die "no MODULE.bazel in ${package_dir} (${package})"

  require_bazel
  if bazel_package_needs_prefix "${package}"; then
    require_prefix
  fi

  if [[ ${#targets[@]} -eq 0 ]]; then
    targets=(//...)
  fi

  info "test   ${package}  jobs=${JOBS}  $(short_path "${package_dir}")"
  note "${targets[*]}"
  if bazel_package_needs_prefix "${package}" && [[ -n "${AUTONOMY_PREFIX}" ]]; then
    export LD_LIBRARY_PATH="${AUTONOMY_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  fi
  run_bazel_in "${package_dir}" test --jobs="${JOBS}" "${targets[@]}"
  ok "test   ${package}"
}

# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------

# Print third-party dependency inventory (BCR, prefix, domain BUILD map).
cmd_deps() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help) usage_deps; return 0 ;;
      *) die "deps: unexpected argument: $1 (try --help)" ;;
    esac
  done

  _SECTION_OPEN=0
  section "BCR  (bazel_dep · MODULE.bazel)"
  th '%-18s %-22s %s' "MODULE" "VERSION" "REPO"
  trule '%-18s %-22s %s' "------" "-------" "----"
  python3 - <<'PY' "${ROOT}/MODULE.bazel"
import re, sys
text = open(sys.argv[1], encoding="utf-8").read()
for m in re.finditer(
    r'bazel_dep\(\s*name\s*=\s*"([^"]+)"\s*,\s*version\s*=\s*"([^"]+)"'
    r'(?:\s*,\s*repo_name\s*=\s*"([^"]+)")?',
    text,
):
    name, ver = m.group(1), m.group(2)
    repo = m.group(3) or name
    print(f"  {name:<18} {ver:<22} @{repo}")
PY

  section "Middleware"
  th '%-22s %s' "LABEL" "ARTIFACT"
  trule '%-22s %s' "-----" "--------"
  kv "//:automsgs" "@automsgs (Bazel + BCR protobuf 30.2)"
  kv "//:autolink" "@autonomy_prefix libautolink.so"
  kv "//:autonomy_headers" "@autonomy_prefix include/autonomy/**"
  kv "//:opencv" "@autonomy_opencv"
  kv "AUTONOMY_PREFIX" \
    "$( [[ -n "${AUTONOMY_PREFIX}" ]] && short_path "${AUTONOMY_PREFIX}" || echo '(unset)' )"
  kv "OPENCV_ROOT" "${OPENCV_ROOT:-'(unset → /usr/local|/usr)'}"

  section "System"
  kv "//:pthread" "-lpthread"

  section "Domain BUILD"
  kv "style"     "autonomy_cc_library + explicit srcs/hdrs"
  kv "binary"    "autonomy.<name>   e.g. //autonomy/bridge:autonomy.bridge"
  kv "deps map"  "tools/dependencies.bzl → AUTONOMY_THIRD_PARTY_DEPS"
  kv "graph"     "tools/package.bzl → AUTONOMY_DOMAIN_*"
  kv "aggregate" "//:cpp_third_party"
  kv "bzlmod"    "MODULE.bazel + MODULE.bazel.lock"
  kv "refresh"   "bazel mod deps --lockfile_mode=update"
}

# List Bazel packages, CMake peers, and domain lib/binary/CMake rows.
cmd_modules() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help) usage_modules; return 0 ;;
      *) die "modules: unexpected argument: $1 (try --help)" ;;
    esac
  done

  _SECTION_OPEN=0
  section "Bazel packages"
  local package package_dir relative_path
  for package in "${BAZEL_PACKAGE_NAMES[@]}"; do
    package_dir="$(bazel_package_dir "${package}")"
    relative_path="."
    [[ "${package_dir}" != "${ROOT}" ]] && relative_path="${package_dir#"${ROOT}"/}"
    if [[ -f "${package_dir}/MODULE.bazel" ]]; then
      printf '  %s%-12s%s %s\n' "${C_GREEN}" "${package}" "${C_RESET}" "${relative_path}"
    else
      printf '  %s%-12s%s %s  %s\n' \
        "${C_RED}" "${package}" "${C_RESET}" "${relative_path}" \
        "$(paint "${C_RED}" "(missing MODULE.bazel)")"
    fi
  done

  section "CMake packages"
  for package in "${CMAKE_PACKAGE_NAMES[@]}"; do
    if [[ "${package}" == "autonomy" ]]; then
      if [[ -f "${ROOT}/package.xml" ]]; then
        printf '  %s%-12s%s .\n' "${C_GREEN}" "${package}" "${C_RESET}"
      else
        printf '  %s%-12s%s .  %s\n' \
          "${C_RED}" "${package}" "${C_RESET}" "$(paint "${C_RED}" "(missing package.xml)")"
      fi
    elif [[ -f "${ROOT}/${package}/package.xml" ]]; then
      printf '  %s%-12s%s %s\n' "${C_GREEN}" "${package}" "${C_RESET}" "${package}"
    else
      printf '  %s%-12s%s %s  %s\n' \
        "${C_RED}" "${package}" "${C_RESET}" "${package}" \
        "$(paint "${C_RED}" "(missing package.xml)")"
    fi
  done

  section "Domains"
  th '%-14s %-48s %-36s %s' "DOMAIN" "LIBRARY" "BINARIES" "CMAKE"
  trule '%-14s %-48s %-36s %s' "------" "-------" "--------" "-----"
  local domain binary_names binary_label
  while IFS= read -r domain; do
    binary_names=""
    while IFS= read -r binary_label; do
      binary_label="${binary_label##*:}"
      if [[ -z "${binary_names}" ]]; then
        binary_names="${binary_label}"
      else
        binary_names="${binary_names},${binary_label}"
      fi
    done < <(domain_bazel_binaries "${domain}")
    [[ -n "${binary_names}" ]] || binary_names="-"
    printf '  %s%-14s%s %-48s %-36s %s-D%s%s\n' \
      "${C_BOLD}" "${domain}" "${C_RESET}" \
      "$(domain_bazel_label "${domain}")" \
      "${binary_names}" \
      "${C_DIM}" "$(domain_cmake_flag "${domain}")" "${C_RESET}"
  done < <(list_domains)

  echo
  kv "prefix" \
    "$( [[ -n "${AUTONOMY_PREFIX}" ]] && short_path "${AUTONOMY_PREFIX}" || echo '(unset)' )"
  kv "tip" "${SCRIPT_NAME} build -m planning"
}

# Modular build: domains, packages, package:target, or // labels.
cmd_build() {
  local -a domains=()
  local -a bazel_packages=()
  local -a cmake_packages=()
  local -a labels=()
  local -a package_targets=()
  local cmake_mode=""   # select | up-to
  local force_cmake=0
  local build_all=0
  local build_all_domains=0
  local smoke_only=0
  local domain

  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help) usage_build; return 0 ;;
      -m|--module|--modules|--domain|--domains)
        [[ $# -ge 2 ]] || die "$1 needs a comma-separated domain list"
        while IFS= read -r domain; do
          domains+=("${domain}")
        done < <(split_csv "$2")
        shift 2
        ;;
      --module=*|--modules=*|--domain=*|--domains=*)
        while IFS= read -r domain; do
          domains+=("${domain}")
        done < <(split_csv "${1#*=}")
        shift
        ;;
      --smoke)
        smoke_only=1
        shift
        ;;
      --cmake)
        force_cmake=1
        cmake_mode="${cmake_mode:-select}"
        shift
        ;;
      --up-to)
        force_cmake=1
        cmake_mode="up-to"
        shift
        ;;
      -*)
        die "build: unexpected option: $1 (try --help)"
        ;;
      all)
        build_all=1
        shift
        ;;
      domains)
        build_all_domains=1
        shift
        ;;
      //*|@*)
        labels+=("$1")
        shift
        ;;
      *:*)
        package_targets+=("$1")
        shift
        ;;
      *)
        if is_bazel_package "$1" || [[ "$1" == "root" || "$1" == "." ]]; then
          if [[ "${force_cmake}" == "1" ]] && is_cmake_package "$1"; then
            cmake_packages+=("$1")
          else
            bazel_packages+=("$1")
          fi
        elif is_cmake_package "$1"; then
          cmake_packages+=("$1")
          force_cmake=1
          cmake_mode="${cmake_mode:-select}"
        elif is_domain "$1"; then
          domains+=("$1")
        else
          die "build: unknown module '${1}' (see: ${SCRIPT_NAME} modules)"
        fi
        shift
        ;;
    esac
  done

  if [[ "${build_all}" == "1" ]]; then
    bazel_packages=("${BAZEL_PACKAGE_NAMES[@]}")
  fi
  if [[ "${build_all_domains}" == "1" ]]; then
    while IFS= read -r domain; do
      domains+=("${domain}")
    done < <(list_domains)
  fi

  # Domains → Bazel lib + autonomy.* binaries (default), or CMake with --cmake.
  if [[ ${#domains[@]} -gt 0 ]]; then
    if [[ "${force_cmake}" == "1" ]]; then
      build_cmake_domains "${domains[@]}"
    else
      build_bazel_domains "${domains[@]}"
    fi
  fi

  if [[ ${#cmake_packages[@]} -gt 0 ]]; then
    build_cmake_packages "${cmake_mode:-select}" "${cmake_packages[@]}"
  fi

  if [[ ${#package_targets[@]} -gt 0 ]]; then
    local spec
    for spec in "${package_targets[@]}"; do
      build_bazel_package_target "${spec}"
    done
  fi

  if [[ ${#labels[@]} -gt 0 ]]; then
    require_root_module
    build_bazel_package tools "${labels[@]}"
  fi

  if [[ ${#bazel_packages[@]} -gt 0 ]]; then
    local package
    for package in "${bazel_packages[@]}"; do
      build_bazel_package "${package}"
    done
  fi

  # Default (no args): all domains + middleware. Use --smoke for the old 4-target check.
  if [[ ${#domains[@]} -eq 0 \
     && ${#cmake_packages[@]} -eq 0 \
     && ${#package_targets[@]} -eq 0 \
     && ${#labels[@]} -eq 0 \
     && ${#bazel_packages[@]} -eq 0 ]]; then
    require_root_module
    if [[ "${smoke_only}" == "1" ]]; then
      build_bazel_package tools "${DEFAULT_SMOKE_TARGETS[@]}"
    else
      local -a all_domains=()
      while IFS= read -r domain; do
        all_domains+=("${domain}")
      done < <(list_domains)
      [[ ${#all_domains[@]} -gt 0 ]] || die "no autonomy/* domains found"
      info "build  all domains (${#all_domains[@]}) + middleware"
      build_bazel_package tools //:automsgs //:autolink //:cpp_third_party
      build_bazel_domains "${all_domains[@]}"
    fi
  elif [[ "${smoke_only}" == "1" ]]; then
    die "build: --smoke cannot be combined with other targets (omit args or drop --smoke)"
  fi
}

# Build selected modules, then bazel test them (or //... by default).
cmd_test() {
  local -a domains=()
  local -a bazel_packages=()
  local -a labels=()
  local test_all=0
  local domain

  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help) usage_test; return 0 ;;
      -m|--module|--modules|--domain|--domains)
        [[ $# -ge 2 ]] || die "$1 needs a comma-separated domain list"
        while IFS= read -r domain; do
          domains+=("${domain}")
        done < <(split_csv "$2")
        shift 2
        ;;
      --module=*|--modules=*|--domain=*|--domains=*)
        while IFS= read -r domain; do
          domains+=("${domain}")
        done < <(split_csv "${1#*=}")
        shift
        ;;
      -*)
        die "test: unexpected option: $1 (try --help)"
        ;;
      all)
        test_all=1
        shift
        ;;
      //*|@*)
        labels+=("$1")
        shift
        ;;
      *)
        if is_bazel_package "$1" || [[ "$1" == "root" || "$1" == "." ]]; then
          bazel_packages+=("$1")
        elif is_domain "$1"; then
          domains+=("$1")
        else
          die "test: unknown module '${1}' (see: ${SCRIPT_NAME} modules)"
        fi
        shift
        ;;
    esac
  done

  if [[ ${#domains[@]} -gt 0 ]]; then
    build_bazel_domains "${domains[@]}"
    local -a domain_labels=()
    local label
    while IFS= read -r label; do
      domain_labels+=("${label}")
    done < <(domain_bazel_labels_for "${domains[@]}")
    test_bazel_package tools "${domain_labels[@]}"
  fi

  if [[ "${test_all}" == "1" ]]; then
    bazel_packages=("${BAZEL_PACKAGE_NAMES[@]}")
  fi

  if [[ ${#labels[@]} -gt 0 ]]; then
    require_root_module
    build_bazel_package tools "${labels[@]}"
    test_bazel_package tools "${labels[@]}"
  fi

  if [[ ${#bazel_packages[@]} -gt 0 ]]; then
    local package
    for package in "${bazel_packages[@]}"; do
      build_bazel_package "${package}"
      test_bazel_package "${package}"
    done
  fi

  if [[ ${#domains[@]} -eq 0 \
     && ${#labels[@]} -eq 0 \
     && ${#bazel_packages[@]} -eq 0 ]]; then
    require_root_module
    cmd_build
    info "test   tools  jobs=${JOBS}  //..."
    require_prefix
    export LD_LIBRARY_PATH="${AUTONOMY_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    run_bazel test --jobs="${JOBS}" //...
    ok "test   tools"
  fi
}

# bazel clean [--expunge] for selected package workspaces.
cmd_clean() {
  local soft=0
  local -a packages=()
  local clean_all=0

  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help) usage_clean; return 0 ;;
      --soft) soft=1; shift ;;
      all) clean_all=1; shift ;;
      *)
        if is_bazel_package "$1" || [[ "$1" == "root" || "$1" == "." ]]; then
          packages+=("$1")
        else
          die "clean: unexpected argument: $1 (try --help)"
        fi
        shift
        ;;
    esac
  done

  if [[ "${clean_all}" == "1" ]]; then
    packages=("${BAZEL_PACKAGE_NAMES[@]}")
  fi
  if [[ ${#packages[@]} -eq 0 ]]; then
    packages=(tools)
  fi

  require_bazel
  local package package_dir
  for package in "${packages[@]}"; do
    package_dir="$(bazel_package_dir "${package}")"
    [[ -f "${package_dir}/MODULE.bazel" ]] || die "no MODULE.bazel in ${package_dir}"
    if [[ "${soft}" == "1" ]]; then
      info "clean  ${package}  --soft  $(short_path "${package_dir}")"
      run_bazel_in "${package_dir}" clean
    else
      info "clean  ${package}  --expunge  $(short_path "${package_dir}")"
      run_bazel_in "${package_dir}" clean --expunge
    fi
  done
  ok "clean"
}

# Run bazel query in a package workspace (default //... in tools).
cmd_query() {
  local expression="//..."
  local package="tools"

  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help) usage_query; return 0 ;;
      -p|--package)
        [[ $# -ge 2 ]] || die "$1 needs a package name"
        package="$2"
        shift 2
        ;;
      --package=*)
        package="${1#--package=}"
        shift
        ;;
      *)
        expression="$1"
        shift
        break
        ;;
    esac
  done
  [[ $# -eq 0 ]] || die "query: unexpected extra arguments (try --help)"

  is_bazel_package "${package}" || [[ "${package}" == "root" || "${package}" == "." ]] \
    || die "query: unknown package '${package}'"

  local package_dir
  package_dir="$(bazel_package_dir "${package}")"
  require_bazel
  info "query  ${package}  ${expression}"
  (
    cd "${package_dir}"
    "${BAZEL}" query "${expression}"
  )
}

# Copy one file into DEST/subdir (create dirs; preserve mode for executables).
install_file_into() {
  local src="$1"
  local dest_dir="$2"
  local mode="${3:-644}"
  [[ -f "${src}" ]] || return 0
  mkdir -p "${dest_dir}"
  install -m "${mode}" "${src}" "${dest_dir}/"
  if [[ "${VERBOSE}" == "1" ]]; then
    note "$(short_path "${src}") → $(short_path "${dest_dir}")/"
  fi
}

# Stage Bazel outputs under bazel-bin into DEST/{lib,bin,include}.
install_bazel_stage() {
  local dest="$1"
  local bin_dir="$2"
  local n_lib=0 n_bin=0 n_hdr=0
  local path

  mkdir -p "${dest}/lib" "${dest}/bin" "${dest}/include"

  # Domain + package libraries.
  while IFS= read -r path; do
    [[ -n "${path}" && -f "${path}" ]] || continue
    install_file_into "${path}" "${dest}/lib" 644
    n_lib=$((n_lib + 1))
  done < <(
    find "${bin_dir}" -type f \( \
        -name 'libautonomy_*.a' -o -name 'libautonomy_*.so' -o -name 'libautonomy_*.so.*' \
        -o -name 'libautomsgs.a' -o -name 'libautomsgs.so' -o -name 'libautomsgs.so.*' \
        -o -name 'libasync_grpc.a' -o -name 'libasync_grpc.so' \
      \) 2>/dev/null | sort -u
  )

  # autonomy.* executables (skip runfiles / bazel metadata; keep ELF only).
  while IFS= read -r path; do
    [[ -n "${path}" && -f "${path}" && -x "${path}" ]] || continue
    [[ "${path}" == *.runfiles* || "${path}" == *.repo_mapping || "${path}" == *.params ]] && continue
    [[ "$(head -c 4 "${path}" 2>/dev/null)" == $'\x7fELF' ]] || continue
    install_file_into "${path}" "${dest}/bin" 755
    n_bin=$((n_bin + 1))
  done < <(
    find "${bin_dir}/autonomy" -type f -name 'autonomy.*' \
      ! -path '*/.runfiles/*' ! -name '*.repo_mapping' 2>/dev/null | sort -u
  )

  # Generated automsgs headers (Bazel @automsgs outs + core/include).
  local am_gen am_core
  am_gen="$(find "${bin_dir}" -type d -path '*/external/*/automsgs' 2>/dev/null | head -1 || true)"
  if [[ -n "${am_gen}" && -d "${am_gen}" ]]; then
    mkdir -p "${dest}/include/automsgs"
    cp -a "${am_gen}/." "${dest}/include/automsgs/" 2>/dev/null || true
    n_hdr=$((n_hdr + 1))
  fi
  am_core="$(find "${bin_dir}" -type d -path '*/external/*/core/include/automsgs' 2>/dev/null | head -1 || true)"
  if [[ -z "${am_core}" ]]; then
    am_core="$(find "${ROOT}/automsgs/core/include/automsgs" -maxdepth 0 -type d 2>/dev/null | head -1 || true)"
  fi
  if [[ -n "${am_core}" && -d "${am_core}" ]]; then
    mkdir -p "${dest}/include/automsgs"
    cp -a "${am_core}/." "${dest}/include/automsgs/" 2>/dev/null || true
    n_hdr=$((n_hdr + 1))
  fi

  note "staged  lib=${n_lib}  bin=${n_bin}  header_trees=${n_hdr}"
  [[ "${n_lib}" -gt 0 || "${n_bin}" -gt 0 ]] \
    || die "install: nothing to stage under $(short_path "${bin_dir}") (build first?)"
}

# Install via autocmake into DEST (sets AUTONOMY_INSTALL_BASE).
install_cmake_prefix() {
  local dest="$1"
  shift
  local mode="${1:-select}"
  shift || true
  local -a packages=("$@")
  local script="${ROOT}/scripts/build_autonomy_autocmake.sh"
  [[ -f "${script}" ]] || die "missing ${script}"

  mkdir -p "${dest}"
  export AUTONOMY_INSTALL_BASE="${dest}"
  # Keep build trees under workspace unless the user overrode them.
  export AUTONOMY_BUILD_BASE="${AUTONOMY_BUILD_BASE:-${ROOT}/build}"

  info "cmake    install  → $(short_path "${dest}")"
  if [[ ${#packages[@]} -eq 0 ]]; then
    packages=(automsgs autolink autonomy)
    mode=select
  fi
  if [[ "${VERBOSE}" == "1" ]]; then
    info "+ AUTONOMY_INSTALL_BASE=${dest} ${script} ${mode} ${packages[*]}"
  fi
  "${script}" "${mode}" "${packages[@]}"
}

# Install build outputs to a prefix directory.
cmd_install() {
  local dest="${PREFIX}"
  local force_cmake=0
  local skip_build=0
  local cmake_mode="select"
  local -a domains=()
  local -a cmake_packages=()
  local -a labels=()
  local -a package_targets=()
  local dest_set=0
  local domain

  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help) usage_install; return 0 ;;
      --prefix)
        [[ $# -ge 2 ]] || die "--prefix needs a directory"
        dest="$2"
        dest_set=1
        shift 2
        ;;
      --prefix=*)
        dest="${1#--prefix=}"
        dest_set=1
        shift
        ;;
      --cmake)
        force_cmake=1
        shift
        ;;
      --skip-build|--no-build)
        skip_build=1
        shift
        ;;
      --up-to)
        force_cmake=1
        cmake_mode="up-to"
        shift
        ;;
      -m|--module|--modules|--domain|--domains)
        [[ $# -ge 2 ]] || die "$1 needs a comma-separated domain list"
        while IFS= read -r domain; do
          domains+=("${domain}")
        done < <(split_csv "$2")
        shift 2
        ;;
      --module=*|--modules=*|--domain=*|--domains=*)
        while IFS= read -r domain; do
          domains+=("${domain}")
        done < <(split_csv "${1#*=}")
        shift
        ;;
      -*)
        die "install: unexpected option: $1 (try --help)"
        ;;
      //*|@*)
        labels+=("$1")
        shift
        ;;
      *:*)
        package_targets+=("$1")
        shift
        ;;
      *)
        # First bare path-looking arg is the install destination.
        if [[ "${dest_set}" == "0" ]] && {
             [[ "$1" == /* || "$1" == ./* || "$1" == ../* || "$1" == ~* ]] \
             || [[ "$1" == install || "$1" == */install || "$1" == */install/* ]]
           }; then
          dest="$1"
          dest_set=1
          shift
        elif is_cmake_package "$1"; then
          cmake_packages+=("$1")
          force_cmake=1
          shift
        elif is_domain "$1"; then
          domains+=("$1")
          shift
        else
          die "install: unexpected argument: $1 (try --help)"
        fi
        ;;
    esac
  done

  [[ -n "${dest}" ]] || die "install: need a destination (--prefix DIR)"
  # Expand relative dest against CWD (not ROOT) so `install ./out` is intuitive.
  case "${dest}" in
    /*) ;;
    ~*) dest="${dest/#\~/${HOME}}" ;;
    *) dest="$(pwd)/${dest}" ;;
  esac
  PREFIX="${dest}"

  if [[ "${force_cmake}" == "1" ]]; then
    if [[ ${#cmake_packages[@]} -eq 0 && ${#domains[@]} -eq 0 ]]; then
      install_cmake_prefix "${dest}" select automsgs autolink autonomy
    elif [[ ${#domains[@]} -gt 0 ]]; then
      export AUTONOMY_INSTALL_BASE="${dest}"
      export AUTONOMY_BUILD_BASE="${AUTONOMY_BUILD_BASE:-${ROOT}/build}"
      build_cmake_domains "${domains[@]}"
    else
      install_cmake_prefix "${dest}" "${cmake_mode}" "${cmake_packages[@]}"
    fi
    ok "install  cmake  → $(short_path "${dest}")"
    note "export AUTONOMY_PREFIX=$(short_path "${dest}")"
    return 0
  fi

  require_root_module
  require_bazel

  if [[ "${skip_build}" != "1" ]]; then
    if [[ ${#domains[@]} -gt 0 ]]; then
      build_bazel_domains "${domains[@]}"
    fi
    if [[ ${#package_targets[@]} -gt 0 ]]; then
      local spec
      for spec in "${package_targets[@]}"; do
        build_bazel_package_target "${spec}"
      done
    fi
    if [[ ${#labels[@]} -gt 0 ]]; then
      build_bazel_package tools "${labels[@]}"
    fi
    if [[ ${#domains[@]} -eq 0 \
       && ${#package_targets[@]} -eq 0 \
       && ${#labels[@]} -eq 0 ]]; then
      build_bazel_package tools
    fi
  fi

  local bin_dir
  bin_dir="$(bazel_bin_dir "${ROOT}")"
  [[ -d "${bin_dir}" ]] || die "install: missing bazel-bin (build first)"

  info "install  bazel  → $(short_path "${dest}")"
  note "from $(short_path "${bin_dir}")"
  install_bazel_stage "${dest}" "${bin_dir}"
  ok "install  → $(short_path "${dest}")"
  note "export AUTONOMY_PREFIX=$(short_path "${dest}")"
}

# Print workspace / toolchain / prefix / module summary.
cmd_status() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help) usage_status; return 0 ;;
      *) die "status: unexpected argument: $1 (try --help)" ;;
    esac
  done

  local bazel_state version_line
  if [[ -x "${BAZEL}" ]] || command -v "${BAZEL}" >/dev/null 2>&1; then
    bazel_state="$(short_path "${BAZEL}")"
    version_line="$(${BAZEL} --version 2>/dev/null | head -1 || echo unknown)"
  else
    bazel_state="$(short_path "${BAZEL}")  $(paint "${C_YELLOW}" "(missing → auto-install on build)")"
    version_line="—"
  fi

  _SECTION_OPEN=0
  section "Workspace"
  kv "root" "$(short_path "${ROOT}")"
  kv "module" "$( [[ -f "${ROOT}/MODULE.bazel" ]] && badge ok || badge missing )"
  kv "bazel-bin" \
    "$( [[ -d "${ROOT}/bazel-bin" ]] && short_path "${ROOT}/bazel-bin" || paint "${C_DIM}" "(not built)" )"

  section "Toolchain"
  kv "bazel" "${bazel_state}"
  kv "version" "${version_line}"
  kv "jobs" "${JOBS}"
  kv "verbose" "${VERBOSE}"
  kv "color" "${COLOR_MODE}$( [[ "${USE_COLOR}" -eq 1 ]] && echo " (on)" || echo " (off)" )"

  section "Prefix"
  kv "PREFIX" "$(short_path "${PREFIX}")"
  kv "AUTONOMY" \
    "$( [[ -n "${AUTONOMY_PREFIX}" ]] && short_path "${AUTONOMY_PREFIX}" || paint "${C_YELLOW}" "(unset)" )"
  if [[ -n "${AUTONOMY_PREFIX}" ]]; then
    kv "automsgs" \
      "$( [[ -f "${AUTONOMY_PREFIX}/lib/libautomsgs.so" ]] && badge ok || badge missing )"
    kv "autolink" \
      "$( [[ -f "${AUTONOMY_PREFIX}/lib/libautolink.so" ]] && badge ok || badge missing )"
  fi

  section "Packages"
  local package package_dir
  for package in "${BAZEL_PACKAGE_NAMES[@]}"; do
    package_dir="$(bazel_package_dir "${package}")"
    kv "${package}" \
      "$( [[ -f "${package_dir}/MODULE.bazel" ]] && badge ok || badge missing )"
  done

  local domain_count=0
  while IFS= read -r _; do
    domain_count=$((domain_count + 1))
  done < <(list_domains)
  kv "domains" "${domain_count}  ($(paint "${C_DIM}" "${SCRIPT_NAME} modules"))"
}

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

# Parse --prefix / --prefix=DIR. Returns consumed argc via exit status (1 or 2).
parse_prefix_arg() {
  case "$1" in
    --prefix)
      [[ $# -ge 2 ]] || die "--prefix needs a directory"
      PREFIX="$2"
      return 2
      ;;
    --prefix=*)
      PREFIX="${1#--prefix=}"
      return 1
      ;;
    *) return 0 ;;
  esac
}

# Parse one global flag. Sets PARSE_SHIFT / PARSE_HELP; return 1 if not a global.
parse_global_flag() {
  PARSE_SHIFT=0
  PARSE_HELP=0
  case "$1" in
    -j|--jobs)
      [[ $# -ge 2 ]] || die "$1 needs a value"
      JOBS="$2"
      PARSE_SHIFT=2
      ;;
    --jobs=*)
      JOBS="${1#--jobs=}"
      PARSE_SHIFT=1
      ;;
    --prefix|--prefix=*)
      local consumed=0
      parse_prefix_arg "$@" || consumed=$?
      [[ "${consumed}" -gt 0 ]] || return 1
      PARSE_SHIFT="${consumed}"
      ;;
    --bazel)
      [[ $# -ge 2 ]] || die "--bazel needs a path"
      BAZEL="$2"
      PARSE_SHIFT=2
      ;;
    --bazel=*)
      BAZEL="${1#--bazel=}"
      PARSE_SHIFT=1
      ;;
    -v|--verbose)
      VERBOSE=1
      PARSE_SHIFT=1
      ;;
    --color)
      [[ $# -ge 2 ]] || die "--color needs auto|always|never"
      COLOR_MODE="$2"
      setup_color
      PARSE_SHIFT=2
      ;;
    --color=*)
      COLOR_MODE="${1#--color=}"
      setup_color
      PARSE_SHIFT=1
      ;;
    --no-color)
      COLOR_MODE=never
      setup_color
      PARSE_SHIFT=1
      ;;
    -h|--help)
      PARSE_SHIFT=1
      PARSE_HELP=1
      ;;
    *) return 1 ;;
  esac
  return 0
}

# Route a command name to its cmd_* handler.
dispatch_command() {
  local command="$1"
  shift
  case "${command}" in
    build)   cmd_build "$@" ;;
    install) cmd_install "$@" ;;
    test)    cmd_test "$@" ;;
    clean)   cmd_clean "$@" ;;
    modules) cmd_modules "$@" ;;
    deps)    cmd_deps "$@" ;;
    query)   cmd_query "$@" ;;
    status)  cmd_status "$@" ;;
    help|-h|--help)
      if [[ $# -eq 0 ]]; then usage_summary; else usage_command "$1"; fi
      ;;
    *)
      usage_summary >&2
      die "unknown command: ${command}"
      ;;
  esac
}

# Entry: parse globals, then dispatch_command.
main() {
  local command=""
  local -a command_args=()

  while [[ $# -gt 0 ]]; do
    if parse_global_flag "$@"; then
      if [[ "${PARSE_HELP}" == "1" && $# -eq "${PARSE_SHIFT}" ]]; then
        usage_summary
        return 0
      fi
      shift "${PARSE_SHIFT}"
      continue
    fi
    break
  done

  command="${1:-help}"
  shift || true

  while [[ $# -gt 0 ]]; do
    if parse_global_flag "$@"; then
      if [[ "${PARSE_HELP}" == "1" ]]; then
        usage_command "${command}"
        return 0
      fi
      shift "${PARSE_SHIFT}"
      continue
    fi
    command_args+=("$1")
    shift
  done

  dispatch_command "${command}" "${command_args[@]+"${command_args[@]}"}"
}

main "$@"
