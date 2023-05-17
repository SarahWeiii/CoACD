#!/usr/bin/env bash

PACKAGE_VERSION="0.0.3"
PY_VERSION=311
BIN=/opt/python/cp${PY_VERSION}-cp${PY_VERSION}/bin/python
COMMAND="${BIN} setup.py bdist_wheel"

echo "CoACD version ${PACKAGE_VERSION}"
echo ${COMMAND}
eval ${COMMAND}

WHEEL_NAME="./dist/coacd-${PACKAGE_VERSION}-cp${PY_VERSION}-cp${PY_VERSION}-linux_x86_64.whl"
auditwheel repair ${WHEEL_NAME}

FIXED_WHEEL_NAME="coacd-${PACKAGE_VERSION}-cp${PY_VERSION}-cp${PY_VERSION}-manylinux2014_x86_64.whl"
NEW_WHEEL_NAME="coacd-${PACKAGE_VERSION}-py3-none-manylinux2014_x86_64.whl"

cd wheelhouse
unzip ${FIXED_WHEEL_NAME}
WHEEL_INFO_FILE="coacd-${PACKAGE_VERSION}.dist-info/WHEEL"
echo "Wheel-Version: 1.0" > ${WHEEL_INFO_FILE}
echo "Generator: bdist_wheel (0.38.4)" >> ${WHEEL_INFO_FILE}
echo "Root-Is-Purelib: false" >> ${WHEEL_INFO_FILE}
echo "Tag: py3-none-manylinux2014_x86_64" >> ${WHEEL_INFO_FILE}
zip -r ${NEW_WHEEL_NAME} coacd coacd-${PACKAGE_VERSION}.data coacd-${PACKAGE_VERSION}.dist-info coacd.libs && \
  rm -rf coacd coacd-${PACKAGE_VERSION}.data coacd-${PACKAGE_VERSION}.dist-info coacd.libs ${FIXED_WHEEL_NAME}
