#!/usr/bin/env bash

while [[ "$#" -gt 0 ]]; do
    case $1 in
        35) VERSION="35";;
        36) VERSION="36";;
        37) VERSION="37";;
        38) VERSION="38";;
        39) VERSION="39";;
        310) VERSION="310";;
        311) VERSION="311";;
    esac
    shift
done

# rm -rf build dist wheelhouse
# echo "deleting build, dist, wheelhouse"

[ -z $VERSION ] && echo "Version not specified" && exit || echo "Compile for Python ${VERSION}"

function build_manylinux14_wheel() {
  PY_VERSION=$1
  if [ "$PY_VERSION" -eq 35 ]; then
      PY_DOT=3.5
      EXT="m"
  elif [ "$PY_VERSION" -eq 36 ]; then
      PY_DOT=3.6
      EXT="m"
  elif [ "$PY_VERSION" -eq 37 ]; then
      PY_DOT=3.7
      EXT="m"
  elif [ "$PY_VERSION" -eq 38 ]; then
      PY_DOT=3.8
      EXT=""
  elif [ "$PY_VERSION" -eq 39 ]; then
      PY_DOT=3.9
      EXT=""
  elif [ "$PY_VERSION" -eq 310 ]; then
      PY_DOT=3.10
      EXT=""
  elif [ "$PY_VERSION" -eq 311 ]; then
      PY_DOT=3.11
      EXT=""
  else
    echo "Error, python version not found!"
  fi

  INCLUDE_PATH=/opt/python/cp${PY_VERSION}-cp${PY_VERSION}${EXT}/include/python${PY_DOT}${EXT}
  BIN=/opt/python/cp${PY_VERSION}-cp${PY_VERSION}${EXT}/bin/python
  echo "Using bin path ${BIN}"
  echo "Using include path ${INCLUDE_PATH}"

  export CPLUS_INCLUDE_PATH=$INCLUDE_PATH
  COMMAND="${BIN} setup.py bdist_wheel"
  echo "Running command ${COMMAND}"
  eval "$COMMAND"

  PACKAGE_VERSION="0.0.1"
  echo "CoACD version ${PACKAGE_VERSION}"
  WHEEL_NAME="./dist/coacd-${PACKAGE_VERSION}-cp${PY_VERSION}-cp${PY_VERSION}${EXT}-linux_x86_64.whl"
  if test -f "$WHEEL_NAME"; then
    echo "$FILE exist, begin audit and repair"
  fi
  auditwheel repair ${WHEEL_NAME}
}

build_manylinux14_wheel $VERSION
