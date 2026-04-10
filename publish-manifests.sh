#!/bin/bash
trap ctrl_c INT

function ctrl_c() {
    echo "Requested to stop."
    exit 1
}

set -euo pipefail

PREFIX="local"
VARIANTS="cpu gpu"

function usage() {
    cat <<EOF
Usage: $0 [-a "cpu gpu"] [-p prefix]
  -a, --variant   Space-separated variants to publish (default: "cpu gpu")
  -p, --prefix    Image prefix / repository namespace
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
    -a|--variant)
        VARIANTS="$2"
        shift 2
        ;;
    -p|--prefix)
        PREFIX="$2"
        shift 2
        ;;
    -h|--help)
        usage
        exit 0
        ;;
    *)
        echo "Unknown option $1" >&2
        usage
        exit 1
        ;;
    esac
done

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
VERSION=$(jq -r '.simapp' "$DIR/VERSION")

echo "Publishing transparent docker manifests for [${VARIANTS}]"

for variant in $VARIANTS; do
    case "$variant" in
    cpu)
        docker buildx imagetools create \
            -t "${PREFIX}/deepracer-simapp:${VERSION}-cpu" \
            "${PREFIX}/deepracer-simapp:${VERSION}-cpu-amd64" \
            "${PREFIX}/deepracer-simapp:${VERSION}-cpu-arm64"
        docker buildx imagetools inspect "${PREFIX}/deepracer-simapp:${VERSION}-cpu"
        ;;
    gpu)
        docker buildx imagetools create \
            -t "${PREFIX}/deepracer-simapp:${VERSION}-gpu" \
            "${PREFIX}/deepracer-simapp:${VERSION}-gpu-amd64"
        docker buildx imagetools inspect "${PREFIX}/deepracer-simapp:${VERSION}-gpu"
        ;;
    *)
        echo "Unknown variant ${variant}" >&2
        exit 1
        ;;
    esac
done
