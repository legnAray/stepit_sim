#!/usr/bin/env bash
set -euo pipefail

MIN_VERSION="${STEPIT_SIM_CMAKE_MIN_VERSION:-3.23}"
CMAKE_VERSION="${STEPIT_SIM_CMAKE_VERSION:-3.31.12}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROOT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
TOOLS_DIR="${ROOT_DIR}/tools"
CMAKE_LINK="${TOOLS_DIR}/cmake"

if [[ -t 1 ]]; then
  GREEN=$'\033[0;32m'
  YELLOW=$'\033[0;33m'
  RED=$'\033[0;31m'
  CLEAR=$'\033[0m'
else
  GREEN=""
  YELLOW=""
  RED=""
  CLEAR=""
fi

log() {
  printf "%b\n" "$*"
}

die() {
  log "${RED}ERROR:${CLEAR} $*" >&2
  exit 1
}

run() {
  log "${GREEN}>>${CLEAR} $(printf '%q ' "$@")"
  "$@"
}

version_ge() {
  local have="$1"
  local need="$2"
  local lowest
  lowest="$(printf '%s\n%s\n' "${need}" "${have}" | sort -V | head -n 1)"
  [[ "${lowest}" == "${need}" ]]
}

cmake_version() {
  local cmake_bin="$1"
  local version_line
  version_line="$("${cmake_bin}" --version 2>/dev/null | head -n 1 || true)"
  [[ "${version_line}" =~ ([0-9]+(\.[0-9]+)+) ]] || return 1
  printf '%s' "${BASH_REMATCH[1]}"
}

download_file() {
  local url="$1"
  local output="$2"
  if command -v curl >/dev/null 2>&1; then
    run curl -L --fail --show-error -o "${output}" "${url}"
  elif command -v wget >/dev/null 2>&1; then
    run wget -O "${output}" "${url}"
  else
    die "Missing curl or wget; install one to download CMake."
  fi
}

detect_cmake_arch() {
  local kernel
  local machine
  kernel="$(uname -s)"
  machine="$(uname -m)"

  [[ "${kernel}" == "Linux" ]] || die "Workspace-local CMake bootstrap currently supports Linux only."
  case "${machine}" in
    x86_64|amd64) printf '%s' "x86_64" ;;
    aarch64|arm64) printf '%s' "aarch64" ;;
    *) die "Unsupported CPU architecture for CMake bootstrap: ${machine}" ;;
  esac
}

if [[ -x "${CMAKE_LINK}/bin/cmake" ]]; then
  detected_version="$(cmake_version "${CMAKE_LINK}/bin/cmake" || true)"
  if [[ -n "${detected_version}" ]] && version_ge "${detected_version}" "${MIN_VERSION}"; then
    log "CMake: ${CMAKE_LINK}/bin/cmake (${detected_version}, workspace-local)"
    log
    log "Use it in this shell with:"
    log "  export PATH=\"${CMAKE_LINK}/bin:\$PATH\""
    exit 0
  fi
fi

if [[ -e "${CMAKE_LINK}" && ! -L "${CMAKE_LINK}" ]]; then
  die "${CMAKE_LINK} exists and is not a symlink. Move it aside before reinstalling workspace-local CMake."
fi

ARCH="$(detect_cmake_arch)"
PACKAGE_NAME="cmake-${CMAKE_VERSION}-linux-${ARCH}"
ARCHIVE="${TOOLS_DIR}/${PACKAGE_NAME}.tar.gz"
SHA_FILE="${TOOLS_DIR}/cmake-${CMAKE_VERSION}-SHA-256.txt"
BASE_URL="https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}"

run mkdir -p "${TOOLS_DIR}"

if [[ ! -f "${ARCHIVE}" ]]; then
  download_file "${BASE_URL}/${PACKAGE_NAME}.tar.gz" "${ARCHIVE}"
else
  log "Using existing archive: ${ARCHIVE}"
fi

if [[ ! -f "${SHA_FILE}" ]]; then
  download_file "${BASE_URL}/cmake-${CMAKE_VERSION}-SHA-256.txt" "${SHA_FILE}"
else
  log "Using existing checksum file: ${SHA_FILE}"
fi

command -v awk >/dev/null 2>&1 || die "Missing awk."
command -v sha256sum >/dev/null 2>&1 || die "Missing sha256sum."
command -v tar >/dev/null 2>&1 || die "Missing tar."

EXPECTED_SHA="$(awk -v file="$(basename "${ARCHIVE}")" '
  index($0, file) {
    for (i = 1; i <= NF; ++i) {
      if (length($i) == 64 && $i ~ /^[0-9a-fA-F]+$/) {
        print tolower($i)
        exit
      }
    }
  }
' "${SHA_FILE}")"
[[ -n "${EXPECTED_SHA}" ]] || die "Could not find checksum for $(basename "${ARCHIVE}") in ${SHA_FILE}."

ACTUAL_SHA="$(sha256sum "${ARCHIVE}" | awk '{print tolower($1)}')"
[[ "${ACTUAL_SHA}" == "${EXPECTED_SHA}" ]] || die "Checksum mismatch for ${ARCHIVE}."

run rm -rf "${TOOLS_DIR:?}/${PACKAGE_NAME}"
run tar -xzf "${ARCHIVE}" -C "${TOOLS_DIR}"
run ln -sfn "${PACKAGE_NAME}" "${CMAKE_LINK}"

DETECTED_VERSION="$(cmake_version "${CMAKE_LINK}/bin/cmake" || true)"
[[ -n "${DETECTED_VERSION}" ]] && version_ge "${DETECTED_VERSION}" "${MIN_VERSION}" \
  || die "Installed CMake does not satisfy >= ${MIN_VERSION}: ${CMAKE_LINK}/bin/cmake"

log
log "CMake: ${CMAKE_LINK}/bin/cmake (${DETECTED_VERSION}, workspace-local)"
log
log "Use it in this shell with:"
log "  export PATH=\"${CMAKE_LINK}/bin:\$PATH\""
