#!/usr/bin/env bash

PACKAGE_VERSION="0.0.5"
PY_VERSION=311
BIN=/opt/python/cp${PY_VERSION}-cp${PY_VERSION}/bin/python
COMMAND="${BIN} setup.py bdist_wheel"

echo "CoACD version ${PACKAGE_VERSION}"
echo ${COMMAND}
eval ${COMMAND}

WHEEL_NAME="./dist/coacd-${PACKAGE_VERSION}-cp${PY_VERSION}-cp${PY_VERSION}-linux_x86_64.whl"
auditwheel repair ${WHEEL_NAME}

cd wheelhouse
${BIN} -m wheel tags --python-tag=py3 --abi-tag=none coacd-0.0.5-cp311-cp311-manylinux2014_x86_64.whl
