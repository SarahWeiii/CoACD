#!/usr/bin/env bash
set -euo pipefail

PACKAGE_VERSION="0.0.5"
PY_VERSION=311
BIN="/opt/python/cp${PY_VERSION}-cp${PY_VERSION}/bin/python"

# Detect the system architecture and set variables accordingly
ARCH=$(uname -m)
if [ "$ARCH" = "x86_64" ]; then
  WHEEL_ARCH="x86_64"
  AUDITWHEEL_PLAT="manylinux_2_28_x86_64"
elif [ "$ARCH" = "aarch64" ]; then
  WHEEL_ARCH="aarch64"
  AUDITWHEEL_PLAT="manylinux_2_28_aarch64"
else
  echo "Unsupported architecture: $ARCH"
  exit 1
fi

echo "Building CoACD version ${PACKAGE_VERSION} for ${ARCH}"

# Build the wheel using setup.py
COMMAND="${BIN} setup.py bdist_wheel"
echo "Running: ${COMMAND}"
eval ${COMMAND}

# Construct the expected wheel filename (from bdist_wheel) based on architecture.
WHEEL_NAME="./dist/coacd-${PACKAGE_VERSION}-cp${PY_VERSION}-cp${PY_VERSION}-linux_${WHEEL_ARCH}.whl"
echo "Repairing wheel: ${WHEEL_NAME}"

# Run auditwheel repair, specifying the target manylinux platform.
auditwheel repair "${WHEEL_NAME}" --plat "${AUDITWHEEL_PLAT}"

# Move into the wheelhouse directory where the repaired wheel is output.
cd wheelhouse

# The repaired wheel should be named according to our package version,
# python version, and the manylinux tag provided.
WHEEL_REPAIRED="coacd-${PACKAGE_VERSION}-cp${PY_VERSION}-cp${PY_VERSION}-${AUDITWHEEL_PLAT}.whl"
echo "Final wheel tags for: ${WHEEL_REPAIRED}"

# Optionally, re-tag the wheel using the wheel tool if needed.
${BIN} -m wheel tags --python-tag=py3 --abi-tag=none "${WHEEL_REPAIRED}"
