#!/usr/bin/env bash
set -e
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
exec "$repo/scripts/build.sh" --profile desktop "$@"
