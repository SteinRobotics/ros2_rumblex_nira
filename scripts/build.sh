#!/usr/bin/env bash
# Build from the robot's colcon workspace, with a shared resource budget.
set -eo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
workspace=$(cd "$repo/../.." && pwd -P)
profile_file="${XDG_CONFIG_HOME:-$HOME/.config}/nira/build-profile"
if [[ ${1:-} == --profile ]]; then
    profile=${2:-}
    [[ $# -ge 2 ]] || { echo 'Expected --profile robot or desktop' >&2; exit 2; }
    shift 2
elif [[ -n ${NIRA_BUILD_PROFILE:-} ]]; then
    profile=$NIRA_BUILD_PROFILE
elif [[ -f $profile_file ]]; then
    profile=$(cat "$profile_file")
else
    profile=desktop
fi
case "$profile" in
    robot|desktop) ;;
    *) echo "Unknown build profile: $profile (expected robot or desktop)" >&2; exit 2 ;;
esac
packages=()
for package in "$@"; do
    [[ $package =~ ^[a-z][a-z0-9_]*$ && -f "$repo/$package/package.xml" ]] || {
        echo "Unknown local package: $package" >&2; exit 2;
    }
    packages+=("$package")
done
selection=()
if (( ${#packages[@]} )); then selection=(--packages-up-to "${packages[@]}"); fi
source "$repo/scripts/environment.sh" --underlay-only
cd "$workspace"
discovery=(--base-paths "$workspace/src")
cmake_options=(-DBUILD_TESTING=ON)
if [[ $profile == robot ]]; then
    source "$repo/scripts/robot_packages.sh"
    robot_package_paths "$workspace/src"
    discovery=(--paths "${robot_paths[@]}")
    cmake_options=(-DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release)
fi
workers=2
memory_high=6G
memory_max=8G
memory_swap=1G
if [[ $(uname -m) == aarch64 ]]; then
    workers=1
    memory_high=40%
    memory_max=50%
    memory_swap=512M
fi
if pgrep -x 'cc1plus|colcon|ninja|ctest|pytest|FreeCAD|gz' >/dev/null; then
    echo 'Another build/test/simulation process is active. Inspect it before starting this build.' >&2
    exit 1
fi
export MAKEFLAGS="-j$workers -l$workers" CMAKE_BUILD_PARALLEL_LEVEL="$workers" CTEST_PARALLEL_LEVEL=1
flock --nonblock "/tmp/rumblex-heavy-$(id -u).lock" \
  systemd-run --user --scope -p "MemoryHigh=$memory_high" -p "MemoryMax=$memory_max" \
    -p "MemorySwapMax=$memory_swap" \
  timeout --kill-after=10s 300s bash -c '
    cg=$(sed -n "s/^0:://p" /proc/self/cgroup)
    for key in memory.high memory.max memory.swap.max; do
      value=$(cat "/sys/fs/cgroup$cg/$key") || exit 1
      [[ $value =~ ^[0-9]+$ ]] || exit 1
    done
    exec "$@"
  ' bash python -m colcon build --symlink-install --executor sequential "${discovery[@]}" "${selection[@]}" \
    --cmake-args "${cmake_options[@]}" -DPython3_EXECUTABLE="$(command -v python)" -DPYTHON_EXECUTABLE="$(command -v python)"
