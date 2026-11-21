# Populate robot_paths with the physical bringup package and its source dependencies.
robot_package_paths() {
    local paths
    paths=$(python -m colcon list --base-paths "$1" --packages-up-to nira_bringup --paths-only) || return 1
    [[ -n $paths ]] || { echo 'Empty robot package selection' >&2; return 1; }
    mapfile -t robot_paths <<< "$paths"
}
