#!/bin/bash
trap ctrl_c INT

function ctrl_c() {
    echo "Requested to stop."
    exit 1
}

set -euo pipefail

PREFIX="local"
VARIANTS="cpu gpu"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

while getopts ":a:p:" opt; do
    case $opt in
    a)
        VARIANTS="$OPTARG"
        ;;
    p)
        PREFIX="$OPTARG"
        ;;
    \?)
        echo "Invalid option -$OPTARG" >&2
        exit 1
        ;;
    esac
done

echo "push.sh is deprecated; publishing transparent manifests for [$VARIANTS] instead."
exec "$DIR/publish-manifests.sh" -a "$VARIANTS" -p "$PREFIX"
