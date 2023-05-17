FROM quay.io/pypa/manylinux2014_x86_64:2022-11-14-1226cfc
RUN yum update -y && yum install -y zip

ENV CC=/opt/rh/devtoolset-10/root/usr/bin/cc CXX=/opt/rh/devtoolset-10/root/usr/bin/c++
ENV LD_LIBRARY_PATH=/opt/rh/devtoolset-10/root/usr/lib64:/opt/rh/devtoolset-10/root/usr/lib:/opt/rh/devtoolset-10/root/usr/lib64/dyninst:/opt/rh/devtoolset-10/root/usr/lib/dyninst:/opt/rh/devtoolset-10/root/usr/lib64:/opt/rh/devtoolset-10/root/usr/lib PCP_DIR=/opt/rh/devtoolset-10/root DEVTOOLSET_ROOTPATH=/opt/rh/devtoolset-10/root MANPATH=/opt/rh/devtoolset-10/root/usr/share/man: PATH=/opt/rh/devtoolset-10/root/usr/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

RUN /opt/python/cp38-cp38/bin/pip install git+https://github.com/fbxiang/auditwheel.git@088b034ae497b0add794da13f8e0f3b0a3b2d651
RUN echo "#!/opt/python/cp38-cp38/bin/python" > /usr/local/bin/auditwheel && \
    echo "import re, sys" >> /usr/local/bin/auditwheel && \
    echo "from auditwheel.main import main" >> /usr/local/bin/auditwheel && \
    echo "if __name__ == '__main__':" >> /usr/local/bin/auditwheel && \
    echo "    sys.argv[0] = re.sub(r'(-script\.pyw|\.exe)?$', '', sys.argv[0])" >> /usr/local/bin/auditwheel && \
    echo "    sys.exit(main())" >> /usr/local/bin/auditwheel
