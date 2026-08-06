#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
WS_DIR="${ROOT_DIR}/reBotArmController_ROS2-main"
WEB_DIR="${ROOT_DIR}/reBotArm_simulator-DM"
VENV_DIR="${WS_DIR}/.venv"
SDK_DEFAULT_DIR="${WS_DIR}/third_party/reBotArm_control_py"
SDK_URL="${REBOTARM_SDK_URL:-https://github.com/vectorBH6/reBotArm_control_py.git}"
SDK_REF="${REBOTARM_SDK_REF:-0fce1f5acd61bd125b99caa111d932fdc2dca60c}"
CHECK_ONLY=0
ASSUME_YES=0

INSTALLED=()
SKIPPED=()
MISMATCH=()
FAILED=()

usage() {
  cat <<'EOF'
Usage: ./setup.sh [--check] [--yes]

  --check  Inspect only; do not install, clone, copy, or build anything.
  --yes    Run non-interactively. sudo may still ask for the user's password.
  -h       Show this help.

Existing files, SDK checkouts, virtual environments, and user configuration
are preserved. Missing components are installed; incompatible versions are
reported in the final summary.
EOF
}

while (($#)); do
  case "$1" in
    --check) CHECK_ONLY=1 ;;
    --yes) ASSUME_YES=1 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown option: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

log() { printf '\n[rebotarm-setup] %s\n' "$*"; }
have() { command -v "$1" >/dev/null 2>&1; }
record_installed() { INSTALLED+=("$1"); }
record_skipped() { SKIPPED+=("$1"); }
record_mismatch() { MISMATCH+=("$1"); }
record_failed() { FAILED+=("$1"); }

run_sudo() {
  if [[ "${EUID}" -eq 0 ]]; then "$@"; else sudo "$@"; fi
}

print_group() {
  local title="$1"
  shift
  local -a values=("$@")
  printf '\n%s (%d)\n' "${title}" "${#values[@]}"
  if ((${#values[@]} == 0)); then
    printf '  - none\n'
  else
    printf '  - %s\n' "${values[@]}"
  fi
}

finish() {
  local status=$?
  if ((status != 0)); then
    record_failed "setup exited before completion with status ${status}"
  fi
  print_group 'Installed/updated' "${INSTALLED[@]}"
  print_group 'Already usable; skipped' "${SKIPPED[@]}"
  print_group 'Version/platform mismatches' "${MISMATCH[@]}"
  print_group 'Failed or still missing' "${FAILED[@]}"
  if ((CHECK_ONLY)); then
    printf '\nCheck-only mode made no changes.\n'
  elif ((${#FAILED[@]} == 0)); then
    printf '\nSetup complete. Next:\n'
    printf '  ./rebotarm doctor\n'
    printf '  ./rebotarm start web\n'
    printf '  ./rebotarm start dm\n'
  else
    printf '\nSetup finished with missing items. Fix the failures above, then rerun ./setup.sh.\n'
  fi
}
trap finish EXIT

if [[ ! -d "${WS_DIR}/src" || ! -d "${WEB_DIR}/public" ]]; then
  record_failed "repository layout is incomplete under ${ROOT_DIR}"
  exit 1
fi

log 'Checking supported platform'
if [[ -r /etc/os-release ]]; then
  # shellcheck source=/dev/null
  source /etc/os-release
  if [[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 24.04 ]]; then
    record_skipped "Ubuntu ${VERSION_ID} supported"
  else
    record_mismatch "expected Ubuntu 24.04; found ${PRETTY_NAME:-unknown}"
  fi
else
  record_mismatch 'cannot identify operating system'
fi

if [[ "$(uname -m)" != x86_64 && "$(uname -m)" != aarch64 ]]; then
  record_mismatch "untested architecture: $(uname -m)"
fi

APT_PACKAGES=(
  git curl ca-certificates build-essential
  python3 python3-venv python3-pip
  nodejs npm
  ros-dev-tools
  ros-jazzy-desktop
  ros-jazzy-rosbridge-suite
  ros-jazzy-moveit
  ros-jazzy-tf-transformations
)
MISSING_APT=()
apt_capability_available() {
  case "$1" in
    nodejs) have node ;;
    npm) have npm ;;
    ros-dev-tools) have colcon && have rosdep ;;
    ros-jazzy-rosbridge-suite) [[ -d /opt/ros/jazzy/share/rosbridge_server ]] ;;
    ros-jazzy-moveit) [[ -d /opt/ros/jazzy/share/moveit_ros_move_group ]] ;;
    ros-jazzy-tf-transformations)
      [[ -d /opt/ros/jazzy/lib/python3.12/site-packages/tf_transformations || \
         -d "${VENV_DIR}/lib/python3.12/site-packages/tf_transformations" ]]
      ;;
    *) return 1 ;;
  esac
}
for package in "${APT_PACKAGES[@]}"; do
  if dpkg-query -W -f='${Status}' "${package}" 2>/dev/null | grep -q 'install ok installed'; then
    version="$(dpkg-query -W -f='${Version}' "${package}" 2>/dev/null || true)"
    record_skipped "apt ${package} ${version}"
  elif apt_capability_available "${package}"; then
    record_skipped "${package} capability already available without the meta-package"
  else
    MISSING_APT+=("${package}")
  fi
done

if ((${#MISSING_APT[@]})); then
  if ((CHECK_ONLY)); then
    for package in "${MISSING_APT[@]}"; do record_failed "missing apt package ${package}"; done
  else
    log "Installing missing system packages: ${MISSING_APT[*]}"
    if ! apt-cache show ros-jazzy-desktop >/dev/null 2>&1; then
      log 'ROS apt repository is missing; installing the official ros2-apt-source package'
      if ! run_sudo apt-get update; then
        record_mismatch 'apt update reported an error; preserving user-configured sources and continuing with available package indexes'
      fi
      run_sudo apt-get install -y software-properties-common curl
      run_sudo add-apt-repository -y universe
      ros_apt_version="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | sed -n 's/.*"tag_name": "\([^"]*\)".*/\1/p' | head -n1)"
      codename="${UBUNTU_CODENAME:-${VERSION_CODENAME:-noble}}"
      ros_apt_deb="$(mktemp /tmp/ros2-apt-source.XXXXXX.deb)"
      curl -fL -o "${ros_apt_deb}" "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_version}/ros2-apt-source_${ros_apt_version}.${codename}_all.deb"
      run_sudo dpkg -i "${ros_apt_deb}"
      record_installed "official ROS 2 apt source ${ros_apt_version}"
    fi
    if ! run_sudo apt-get update; then
      record_mismatch 'apt update reported an error (often a third-party source); no source was removed, continuing with available package indexes'
    fi
    if run_sudo apt-get install -y "${MISSING_APT[@]}"; then
      for package in "${MISSING_APT[@]}"; do record_installed "apt ${package}"; done
    else
      for package in "${MISSING_APT[@]}"; do
        if ! dpkg-query -W -f='${Status}' "${package}" 2>/dev/null | grep -q 'install ok installed'; then
          record_failed "apt ${package}"
        fi
      done
    fi
  fi
fi

log 'Checking runtime versions'
if have python3; then
  py_version="$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:3])))')"
  if python3 -c 'import sys; raise SystemExit(0 if sys.version_info[:2] == (3, 12) else 1)'; then
    record_skipped "Python ${py_version} compatible"
  else
    record_mismatch "expected Python 3.12; found ${py_version}"
  fi
else
  record_failed 'python3 command missing'
fi

if have node; then
  node_version="$(node -p 'process.versions.node')"
  node_major="${node_version%%.*}"
  if ((node_major >= 18)); then
    record_skipped "Node.js ${node_version} compatible"
  else
    record_mismatch "Node.js >=18 required; found ${node_version}"
  fi
else
  record_failed 'node command missing'
fi

log 'Checking reBotArm_control_py SDK'
SDK_DIR=''
for candidate in \
  "${WS_DIR}/third_party/reBotArm_control_py" \
  "${WS_DIR}/sdk/reBotArm_control_py" \
  "${HOME}/reBotArm_control_py"; do
  if [[ -d "${candidate}/reBotArm_control_py" ]]; then SDK_DIR="${candidate}"; break; fi
done

if [[ -z "${SDK_DIR}" ]]; then
  if ((CHECK_ONLY)); then
    record_failed "SDK missing; expected ${SDK_DEFAULT_DIR} or ~/reBotArm_control_py"
  else
    mkdir -p "$(dirname -- "${SDK_DEFAULT_DIR}")"
    if git clone "${SDK_URL}" "${SDK_DEFAULT_DIR}" && git -C "${SDK_DEFAULT_DIR}" checkout "${SDK_REF}"; then
      SDK_DIR="${SDK_DEFAULT_DIR}"
      record_installed "SDK ${SDK_URL} @ ${SDK_REF}"
    else
      record_failed "unable to clone SDK ${SDK_URL}"
    fi
  fi
else
  record_skipped "existing SDK ${SDK_DIR} preserved"
fi

if [[ -n "${SDK_DIR}" && -d "${SDK_DIR}/.git" ]]; then
  sdk_head="$(git -C "${SDK_DIR}" rev-parse HEAD 2>/dev/null || true)"
  if [[ "${sdk_head}" == "${SDK_REF}" ]]; then
    record_skipped "SDK revision ${sdk_head} compatible"
  else
    record_mismatch "SDK revision ${sdk_head:-unknown}; validated revision is ${SDK_REF} (existing checkout not changed)"
  fi
fi

log 'Checking project virtual environment'
if [[ -x "${VENV_DIR}/bin/python" ]]; then
  record_skipped "existing virtual environment ${VENV_DIR} preserved"
elif ((CHECK_ONLY)); then
  record_failed "missing virtual environment ${VENV_DIR}"
else
  python3 -m venv "${VENV_DIR}" --system-site-packages
  record_installed "virtual environment ${VENV_DIR}"
fi

if [[ -x "${VENV_DIR}/bin/python" ]]; then
  if ((CHECK_ONLY)); then
    :
  else
    log 'Installing only missing or incompatible Python packages into the project venv'
    "${VENV_DIR}/bin/python" -m pip install --upgrade pip
    "${VENV_DIR}/bin/python" -m pip install -r "${ROOT_DIR}/requirements.txt"
    record_installed 'Python requirements checked/updated in project venv'
  fi

  mapfile -t PY_REPORT < <("${VENV_DIR}/bin/python" - "${ROOT_DIR}/requirements.txt" <<'PY'
import importlib.metadata as md
import sys
from packaging.requirements import Requirement

for raw in open(sys.argv[1], encoding='utf-8'):
    raw = raw.strip()
    if not raw or raw.startswith('#'):
        continue
    req = Requirement(raw)
    try:
        version = md.version(req.name)
    except md.PackageNotFoundError:
        print(f'MISSING|pip {req.name} {req.specifier}')
        continue
    state = 'OK' if version in req.specifier else 'MISMATCH'
    print(f'{state}|pip {req.name} {version} expected {req.specifier}')
PY
  )
  for item in "${PY_REPORT[@]}"; do
    state="${item%%|*}"
    detail="${item#*|}"
    case "${state}" in
      OK) record_skipped "${detail}" ;;
      MISMATCH) record_mismatch "${detail}" ;;
      MISSING) record_failed "${detail}" ;;
    esac
  done
fi

log 'Checking web configuration'
if [[ -f "${WEB_DIR}/.env" ]]; then
  record_skipped "existing ${WEB_DIR}/.env preserved"
elif ((CHECK_ONLY)); then
  record_failed "missing ${WEB_DIR}/.env"
else
  cp "${WEB_DIR}/.env.example" "${WEB_DIR}/.env"
  record_installed "created ${WEB_DIR}/.env from example"
fi

if [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
  if ((CHECK_ONLY)); then
    [[ -f "${WS_DIR}/install/setup.bash" ]] \
      && record_skipped 'ROS workspace is built' \
      || record_failed 'ROS workspace has not been built'
  else
    log 'Resolving ROS dependencies and building the workspace'
    # shellcheck source=/dev/null
    source /opt/ros/jazzy/setup.bash
    if have rosdep; then
      if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        if ! run_sudo rosdep init; then
          record_mismatch 'rosdep init failed; preserving the system state and continuing to the build'
        fi
      fi
      if [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
        if ! rosdep update; then
          record_mismatch 'rosdep update failed; continuing with explicitly installed dependencies'
        elif ! rosdep install --from-paths "${WS_DIR}/src" --ignore-src -r -y; then
          record_mismatch 'rosdep could not resolve every package; colcon build will perform the final dependency check'
        fi
      else
        record_mismatch 'rosdep is not initialized; colcon build will perform the final dependency check'
      fi
    fi
    (
      cd "${WS_DIR}"
      source "${VENV_DIR}/bin/activate"
      colcon build --symlink-install
    )
    record_installed 'ROS workspace built with colcon'
  fi
else
  record_failed '/opt/ros/jazzy/setup.bash missing'
fi

log 'Final import checks'
if [[ -x "${VENV_DIR}/bin/python" && -n "${SDK_DIR}" ]]; then
  if PYTHONPATH="${SDK_DIR}:${VENV_DIR}/lib/python3.12/site-packages:${PYTHONPATH:-}" \
    "${VENV_DIR}/bin/python" -c 'import mujoco, pinocchio, motorbridge, transforms3d, tf_transformations, fastmcp, openai, reBotArm_control_py'; then
    record_skipped 'critical Python and SDK imports pass'
  else
    record_failed 'one or more critical Python/SDK imports fail'
  fi
fi

if [[ -e /dev/ttyACM0 ]]; then
  record_skipped '/dev/ttyACM0 detected'
  if [[ ! -r /dev/ttyACM0 || ! -w /dev/ttyACM0 ]]; then
    record_mismatch '/dev/ttyACM0 exists but current user lacks read/write permission'
  fi
else
  record_mismatch '/dev/ttyACM0 not connected (installation can still complete)'
fi
