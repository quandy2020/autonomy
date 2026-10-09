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
#   ./autonomy.sh clean
#   ./autonomy.sh status
#
# Root Bazel links CMake-built automsgs/autolink via @autonomy_prefix:
#   export AUTONOMY_PREFIX=$PWD/build   # or install
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

BAZEL="${BAZEL:-bazel}"
JOBS="${JOBS:-$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 8)}"
PREFIX="${PREFIX:-${ROOT}/install}"
VERBOSE="${VERBOSE:-0}"

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

# Default smoke targets (need AUTONOMY_PREFIX for //:automsgs / //:autolink).
readonly DEFAULT_BUILD_TARGETS=(
  //:automsgs
  //:autolink
  //:cpp_third_party
  //autonomy/common:autonomy_common
)

# Bazel packages with their own MODULE.bazel ("tools" = repo root).
readonly BAZEL_PACKAGE_NAMES=(tools automsgs autodriver autoviz)

# CMake / autocmake packages (package.xml peers).
readonly CMAKE_PACKAGE_NAMES=(autonomy automsgs autolink autodriver autoviz)

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

# Fatal error to stderr, then exit 1.
die()  { echo "${SCRIPT_NAME}: error: $*" >&2; exit 1; }

# High-level progress line.
info() { echo "==> $*"; }

# Success line.
ok()   { echo "ok  $*"; }

# Indented detail under an info line.
note() { echo "    $*"; }

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
  cat <<EOF
Usage: ${SCRIPT_NAME} [global options] <command> [command options]

Commands:
  build       Modular build (packages, domains, package:target, // labels)
  test        Build, then run bazel test
  clean       Wipe Bazel cache (optionally per package)
  modules     List packages, domain libs/binaries, cmake peers
  deps        Third-party deps (BCR + prefix + domain BUILD map)
  query       Run bazel query (default: //... in root)
  status      Workspace / toolchain / prefix summary
  help        Show this help, or help for a command

Global options:
  -j, --jobs N         Bazel parallel jobs          [default: ${JOBS}]
      --prefix DIR     Local install hint           [default: ${PREFIX}]
      --bazel PATH     Bazel binary                 [default: ${BAZEL}]
  -v, --verbose        Print underlying bazel / cmake commands
  -h, --help           Same as: ${SCRIPT_NAME} help

Environment:
  AUTONOMY_PREFIX   CMake prefix with libautomsgs.so   [now: ${AUTONOMY_PREFIX:-'(unset)'}]
  PREFIX   JOBS   BAZEL   VERBOSE

Examples:
  ${SCRIPT_NAME} deps
  ${SCRIPT_NAME} modules
  ${SCRIPT_NAME} build
  ${SCRIPT_NAME} build -m planning,bridge
  ${SCRIPT_NAME} build //autonomy/planning:autonomy_planning
  ${SCRIPT_NAME} build //autonomy/bridge:autonomy.bridge
  ${SCRIPT_NAME} build --cmake -m perception,map
  ${SCRIPT_NAME} -j 16 test autodriver
EOF
}

# Usage for the build command.
usage_build() {
  cat <<EOF
Usage: ${SCRIPT_NAME} build [options] [module|label ...]

Build modes (mixable):
  (no args)              Root workspace default targets
  <bazel-package>        Build that MODULE.bazel package (//...)
  <package>:<target>     Target inside a bazel package (e.g. autodriver:autodriver)
  all                    All known bazel packages (tools automsgs autodriver autoviz)
  domains                All domain libs (+ binaries when declared)
  //label ...            Root-workspace Bazel labels
  -m, --module LIST      domains → //autonomy/<d>:autonomy_<d> [+ autonomy.<d>]
  --cmake                Use autocmake + AUTONOMY_BUILD_* for domains / packages
  --up-to pkg ...        autocmake --packages-up-to

Bazel packages:  ${BAZEL_PACKAGE_NAMES[*]}
CMake packages:  ${CMAKE_PACKAGE_NAMES[*]}
Domains:         \$ ${SCRIPT_NAME} modules

Examples:
  ${SCRIPT_NAME} build
  ${SCRIPT_NAME} build planning control          # lib + binary (if any)
  ${SCRIPT_NAME} build -m planning,bridge
  ${SCRIPT_NAME} build domains
  ${SCRIPT_NAME} build //autonomy/bridge:autonomy.bridge
  ${SCRIPT_NAME} build autodriver:autodriver_bin
  ${SCRIPT_NAME} build --cmake -m planning,control
EOF
}

# Usage for the test command.
usage_test() {
  cat <<EOF
Usage: ${SCRIPT_NAME} test [options] [module|label ...]

1. Build selected modules / labels (same resolution as \`build\`)
2. bazel test //... (or the labels / package defaults you pass)

Options:
  -m, --module LIST   Same as build (CMake domain select; test still runs bazel)
EOF
}

# Usage for the clean command.
usage_clean() {
  cat <<EOF
Usage: ${SCRIPT_NAME} clean [--soft] [package ...]

Default: bazel clean --expunge in each selected workspace
  --soft     bazel clean   (keep the repository cache)
  (no pkg)   root workspace only
  <package>  clean that bazel package dir (tools|automsgs|autodriver|autoviz)
  all        clean every known bazel package
EOF
}

# Usage for the modules command.
usage_modules() {
  cat <<EOF
Usage: ${SCRIPT_NAME} modules

Print:
  - Bazel packages (own MODULE.bazel)
  - CMake / autocmake packages
  - Domains: library //autonomy/<d>:autonomy_<d>, binaries autonomy.<name>,
    and CMake -DAUTONOMY_BUILD_<D>=ON|OFF
  Domain BUILD style: Apollo-like explicit srcs/hdrs (see autonomy/bridge/BUILD.bazel)
EOF
}

# Usage for the deps command.
usage_deps() {
  cat <<EOF
Usage: ${SCRIPT_NAME} deps

Print third-party dependencies at a glance:
  - BCR bazel_dep from MODULE.bazel
  - CMake @autonomy_prefix targets (automsgs / autolink / autonomy_headers)
  - System //:pthread

Also see: MODULE.bazel (header inventory)
EOF
}

# Usage for the query command.
usage_query() {
  cat <<EOF
Usage: ${SCRIPT_NAME} query [expression]
       ${SCRIPT_NAME} query -p <package> [expression]

Default expression: //...
  -p, --package NAME   Query inside that bazel package workspace
EOF
}

# Usage for the status command.
usage_status() {
  cat <<EOF
Usage: ${SCRIPT_NAME} status

Print workspace, Bazel version, AUTONOMY_PREFIX, JOBS, modules, and bazel-bin.
EOF
}

# Dispatch help for a named command (or summary when empty).
usage_command() {
  case "${1:-}" in
    build)   usage_build ;;
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

# Abort unless BAZEL is on PATH.
require_bazel() {
  command -v "${BAZEL}" >/dev/null 2>&1 \
    || die "'${BAZEL}' not found in PATH (set --bazel or BAZEL=)"
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
run_bazel_in() {
  local workdir="$1"
  shift
  if [[ "${VERBOSE}" == "1" ]]; then
    info "+ (cd ${workdir}) AUTONOMY_PREFIX=${AUTONOMY_PREFIX:-} ${BAZEL} $*"
  fi
  (
    cd "${workdir}"
    if [[ -n "${AUTONOMY_PREFIX}" ]]; then
      "${BAZEL}" --repo_env=AUTONOMY_PREFIX="${AUTONOMY_PREFIX}" "$@"
    else
      "${BAZEL}" "$@"
    fi
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
  info "bazel domains: ${domains[*]}"
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

  info "cmake domains: ${domains[*]}"
  note "flags: ${cmake_args[*]}"
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

  info "cmake ${mode}: ${packages[*]}"
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

  info "bazel [${package}] (${JOBS} jobs)  dir=${package_dir}"
  note "${targets[*]}"
  if bazel_package_needs_prefix "${package}" && [[ -n "${AUTONOMY_PREFIX}" ]]; then
    export LD_LIBRARY_PATH="${AUTONOMY_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  fi
  run_bazel_in "${package_dir}" build --jobs="${JOBS}" "${targets[@]}"
  ok "build [${package}] → $(bazel_bin_dir "${package_dir}")"
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

  info "bazel test [${package}] (${JOBS} jobs)"
  note "${targets[*]}"
  if bazel_package_needs_prefix "${package}" && [[ -n "${AUTONOMY_PREFIX}" ]]; then
    export LD_LIBRARY_PATH="${AUTONOMY_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  fi
  run_bazel_in "${package_dir}" test --jobs="${JOBS}" "${targets[@]}"
  ok "test [${package}] finished"
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

  echo "=== BCR (bazel_dep in MODULE.bazel) ==="
  printf '  %-18s %-22s %s\n' "MODULE" "VERSION" "REPO / NOTE"
  printf '  %-18s %-22s %s\n' "------" "-------" "----------"
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

  echo
  echo "=== CMake prefix (@autonomy_prefix → //:*) ==="
  printf '  %-22s %s\n' "LABEL" "ARTIFACT"
  printf '  %-22s %s\n' "-----" "--------"
  printf '  %-22s %s\n' "//:automsgs" "libautomsgs.so + include/automsgs"
  printf '  %-22s %s\n' "//:autolink" "libautolink.so + include/autolink"
  printf '  %-22s %s\n' "//:autonomy_headers" "include/autonomy/** (generated *.pb.h)"
  echo "  AUTONOMY_PREFIX=${AUTONOMY_PREFIX:-'(unset)'}"

  echo
  echo "=== System ==="
  printf '  %-22s %s\n' "//:pthread" "-lpthread"

  echo
  echo "=== Domain BUILD (Apollo-style) ==="
  echo "  each autonomy/<d>/BUILD.bazel: autonomy_cc_library + explicit srcs/hdrs"
  echo "  binaries: autonomy.<name> (e.g. //autonomy/bridge:autonomy.bridge)"
  echo "  third-party SSOT: tools/dependencies.bzl → AUTONOMY_THIRD_PARTY_DEPS"
  echo "  domain graph:     tools/package.bzl → AUTONOMY_DOMAIN_*"
  echo "  aggregate:        //:cpp_third_party"
  echo
  echo "Docs: MODULE.bazel header | BUILD.bazel | autonomy/bridge/BUILD.bazel"
}

# List Bazel packages, CMake peers, and domain lib/binary/CMake rows.
cmd_modules() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help) usage_modules; return 0 ;;
      *) die "modules: unexpected argument: $1 (try --help)" ;;
    esac
  done

  echo "Bazel packages (MODULE.bazel):"
  local package package_dir relative_path
  for package in "${BAZEL_PACKAGE_NAMES[@]}"; do
    package_dir="$(bazel_package_dir "${package}")"
    relative_path="."
    [[ "${package_dir}" != "${ROOT}" ]] && relative_path="${package_dir#"${ROOT}"/}"
    if [[ -f "${package_dir}/MODULE.bazel" ]]; then
      printf '  %-12s %s\n' "${package}" "${relative_path}"
    else
      printf '  %-12s %s  (missing MODULE.bazel)\n' "${package}" "${relative_path}"
    fi
  done

  echo
  echo "CMake / autocmake packages:"
  for package in "${CMAKE_PACKAGE_NAMES[@]}"; do
    if [[ "${package}" == "autonomy" ]]; then
      if [[ -f "${ROOT}/package.xml" ]]; then
        printf '  %-12s .\n' "${package}"
      else
        printf '  %-12s .  (missing package.xml)\n' "${package}"
      fi
    elif [[ -f "${ROOT}/${package}/package.xml" ]]; then
      printf '  %-12s %s\n' "${package}" "${package}"
    else
      printf '  %-12s %s  (missing package.xml)\n' "${package}" "${package}"
    fi
  done

  echo
  echo "Domains (lib / binaries / CMake flag):"
  printf '  %-14s %-50s %-32s %s\n' "DOMAIN" "LIBRARY" "BINARIES" "CMAKE"
  printf '  %-14s %-50s %-32s %s\n' "------" "-------" "--------" "-----"
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
    printf '  %-14s %-50s %-32s -D%s=ON|OFF\n' \
      "${domain}" \
      "$(domain_bazel_label "${domain}")" \
      "${binary_names}" \
      "$(domain_cmake_flag "${domain}")"
  done < <(list_domains)

  echo
  echo "Prefix: AUTONOMY_PREFIX=${AUTONOMY_PREFIX:-'(unset — export to CMake install/build)'}"
  echo "Tip:    ${SCRIPT_NAME} build -m planning   # builds lib + autonomy.planning"
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

  # Default: root tools targets when nothing else was selected.
  if [[ ${#domains[@]} -eq 0 \
     && ${#cmake_packages[@]} -eq 0 \
     && ${#package_targets[@]} -eq 0 \
     && ${#labels[@]} -eq 0 \
     && ${#bazel_packages[@]} -eq 0 ]]; then
    require_root_module
    build_bazel_package tools
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
    info "bazel test //... [tools]"
    require_prefix
    export LD_LIBRARY_PATH="${AUTONOMY_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    run_bazel test --jobs="${JOBS}" //...
    ok "test finished"
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
      info "bazel clean [${package}]"
      run_bazel_in "${package_dir}" clean
    else
      info "bazel clean --expunge [${package}]"
      run_bazel_in "${package_dir}" clean --expunge
    fi
  done
  ok "clean complete"
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
  info "bazel query [${package}] ${expression}"
  (
    cd "${package_dir}"
    "${BAZEL}" query "${expression}"
  )
}

# Print workspace / toolchain / prefix / module summary.
cmd_status() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -h|--help) usage_status; return 0 ;;
      *) die "status: unexpected argument: $1 (try --help)" ;;
    esac
  done

  echo "workspace : ${ROOT}"
  echo "bazel     : ${BAZEL} ($(command -v "${BAZEL}" 2>/dev/null || echo 'not found'))"
  if command -v "${BAZEL}" >/dev/null 2>&1; then
    echo "version   : $(${BAZEL} --version 2>/dev/null | head -1 || echo unknown)"
  fi
  echo "jobs      : ${JOBS}"
  echo "prefix    : ${PREFIX}"
  echo "autonomy  : ${AUTONOMY_PREFIX:-'(unset)'}"
  if [[ -n "${AUTONOMY_PREFIX}" ]]; then
    if [[ -f "${AUTONOMY_PREFIX}/lib/libautomsgs.so" ]]; then
      echo "  libautomsgs.so : present"
    else
      echo "  libautomsgs.so : missing"
    fi
    if [[ -f "${AUTONOMY_PREFIX}/lib/libautolink.so" ]]; then
      echo "  libautolink.so : present"
    else
      echo "  libautolink.so : missing"
    fi
  fi
  echo "verbose   : ${VERBOSE}"
  echo "module    : $( [[ -f "${ROOT}/MODULE.bazel" ]] && echo present || echo missing )"

  echo "packages  :"
  local package package_dir
  for package in "${BAZEL_PACKAGE_NAMES[@]}"; do
    package_dir="$(bazel_package_dir "${package}")"
    printf '  %-12s %s\n' "${package}" \
      "$( [[ -f "${package_dir}/MODULE.bazel" ]] && echo ok || echo missing )"
  done

  local domain_count=0
  while IFS= read -r _; do
    domain_count=$((domain_count + 1))
  done < <(list_domains)
  echo "domains   : ${domain_count}  (${SCRIPT_NAME} modules)"

  local bazel_bin=""
  [[ -d "${ROOT}/bazel-bin" ]] && bazel_bin="${ROOT}/bazel-bin"
  echo "bazel-bin : ${bazel_bin:-'(not built yet)'}"
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
