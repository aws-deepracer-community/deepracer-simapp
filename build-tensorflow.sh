#!/bin/bash
#
# Build TensorFlow 2.19.0 with CUDA/GPU support for AWS EC2 g5g instances (ARM64 + NVIDIA T4G)
#
# Uses docker/Dockerfile.tensorflow to build inside a container.
# CUDA and cuDNN are downloaded automatically by Bazel (hermetic build).
# No GPU or NVIDIA driver is required on the build machine.
#
# Target:
#   - Architecture: aarch64 (ARM64, AWS Graviton)
#   - GPU: NVIDIA T4G (compute capability 7.5)
#   - CUDA: 12.5 | cuDNN: 9.3 | Clang 18 | Bazel 6.5.0 | Python 3.9-3.12
#
# Usage:
#   ./build-tensorflow.sh              # Build with defaults
#   ./build-tensorflow.sh -p 3.11      # Build with Python 3.11
#   ./build-tensorflow.sh -j 32        # Build with 32 parallel jobs
#   ./build-tensorflow.sh -r 48000     # Limit RAM usage to ~48GB
#   ./build-tensorflow.sh -o /output   # Custom output directory for .whl
#
# Prerequisites:
#   - Docker with buildx support
#   - For cross-compilation from x86: QEMU user-static + binfmt-support
#

set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"

trap ctrl_c INT
function ctrl_c() {
    echo ""
    echo "Build interrupted by user."
    exit 1
}

# ──────────────────────────────────────────────────────────────────────────────
# Configuration defaults
# ──────────────────────────────────────────────────────────────────────────────
TF_VERSION="2.19.0"
TF_GIT_TAG="v${TF_VERSION}"
PYTHON_VERSION="3.12"
CLANG_VERSION="18"
CUDA_COMPUTE_CAPABILITIES="7.5"   # T4G GPU
PARALLEL_JOBS=""                   # Empty = auto (Bazel default)
RAM_RESOURCES=""                   # Empty = auto; set to e.g. 48000 for 48GB
OUTPUT_DIR="$(pwd)/tensorflow-wheels"
DOCKER_IMAGE_NAME="tf-builder-arm64-cuda"

# ──────────────────────────────────────────────────────────────────────────────
# Parse arguments
# ──────────────────────────────────────────────────────────────────────────────
while getopts ":p:j:r:o:h" opt; do
    case $opt in
        p) PYTHON_VERSION="$OPTARG" ;;
        j) PARALLEL_JOBS="$OPTARG" ;;
        r) RAM_RESOURCES="$OPTARG" ;;
        o) OUTPUT_DIR="$OPTARG" ;;
        h)
            echo "Usage: $0 [-p python_version] [-j parallel_jobs] [-r ram_mb] [-o output_dir]"
            echo ""
            echo "  -p  Python version (default: ${PYTHON_VERSION}). Supported: 3.9, 3.10, 3.11, 3.12"
            echo "  -j  Number of parallel Bazel jobs (default: auto)"
            echo "  -r  RAM limit in MB for Bazel (default: auto). E.g. 48000 for ~48GB"
            echo "  -o  Output directory for the .whl file (default: ./tensorflow-wheels)"
            echo "  -h  Show this help message"
            exit 0
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            exit 1
            ;;
    esac
done

# ──────────────────────────────────────────────────────────────────────────────
# Validate Python version
# ──────────────────────────────────────────────────────────────────────────────
case "$PYTHON_VERSION" in
    3.9|3.10|3.11|3.12) ;;
    *)
        echo "Error: Unsupported Python version '${PYTHON_VERSION}'. TF 2.19 supports 3.9-3.12."
        exit 1
        ;;
esac

echo "================================================================"
echo "  TensorFlow ${TF_VERSION} Build Configuration"
echo "================================================================"
echo "  Target arch:       aarch64 (ARM64)"
echo "  GPU:               NVIDIA T4G (sm_75)"
echo "  CUDA:              12.5 (hermetic)"
echo "  cuDNN:             9.3 (hermetic)"
echo "  Python:            ${PYTHON_VERSION}"
echo "  Clang:             ${CLANG_VERSION}"
echo "  Compute cap:       ${CUDA_COMPUTE_CAPABILITIES}"
echo "  Output dir:        ${OUTPUT_DIR}"
echo "================================================================"
echo ""

# ──────────────────────────────────────────────────────────────────────────────
# Build via Docker
# ──────────────────────────────────────────────────────────────────────────────
echo "Building TensorFlow inside Docker container..."
mkdir -p "${OUTPUT_DIR}"

DOCKERFILE="${DIR}/docker/Dockerfile.tensorflow"
if [ ! -f "${DOCKERFILE}" ]; then
    echo "Error: Dockerfile not found at ${DOCKERFILE}"
    exit 1
fi

docker buildx build \
    --platform linux/arm64 \
    -f "${DOCKERFILE}" \
    --build-arg BASE_IMG="ubuntu:24.04" \
    --build-arg PYTHON_VERSION="${PYTHON_VERSION}" \
    --build-arg CLANG_VERSION="${CLANG_VERSION}" \
    --build-arg TF_GIT_TAG="${TF_GIT_TAG}" \
    --build-arg CUDA_COMPUTE_CAPABILITIES="${CUDA_COMPUTE_CAPABILITIES}" \
    --build-arg PARALLEL_JOBS="${PARALLEL_JOBS}" \
    --build-arg RAM_RESOURCES="${RAM_RESOURCES}" \
    -t "${DOCKER_IMAGE_NAME}:${TF_VERSION}" \
    --load \
    .

# Extract the wheel from the container
CONTAINER_ID=$(docker create "${DOCKER_IMAGE_NAME}:${TF_VERSION}")
docker cp "${CONTAINER_ID}:/output/." "${OUTPUT_DIR}/"
docker rm "${CONTAINER_ID}"

echo ""
echo "================================================================"
echo "  Build complete!"
echo "  Finished at: $(date)"
echo "================================================================"
echo ""
echo "  Wheel(s) saved to: ${OUTPUT_DIR}"
ls -lah "${OUTPUT_DIR}"/tensorflow*.whl 2>/dev/null || echo "Warning: No wheel files found."
echo ""
echo "  Install with:"
echo "    pip install ${OUTPUT_DIR}/tensorflow-*.whl"
echo ""
echo "  Verify GPU support:"
echo "    python -c \"import tensorflow as tf; print('GPUs:', tf.config.list_physical_devices('GPU'))\""
echo ""
