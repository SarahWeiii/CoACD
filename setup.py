# -*- coding: utf-8 -*-
import os
import sys
import platform
import subprocess

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from wheel.bdist_wheel import bdist_wheel


class bdist_wheel_abi3(bdist_wheel):
    def get_tag(self):
        python, abi, plat = super().get_tag()

        if python.startswith("cp"):
            # on CPython, our wheels are abi3 and compatible back to 3.6
            return python, "abi3", plat

        return python, abi, plat


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        extdir = os.path.join(extdir, self.distribution.get_name())

        cfg = "Debug" if self.debug else "Release"

        cmake_generator = os.environ.get("CMAKE_GENERATOR", "")
        system = platform.system().lower()
        # Set Python_EXECUTABLE instead if you use PYBIND11_FINDPYTHON
        cmake_args = [
            "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={}".format(extdir),
            "-DCMAKE_RUNTIME_OUTPUT_DIRECTORY=$<1:{}>".format(extdir),
            "-DCMAKE_OSX_ARCHITECTURES=arm64;x86_64",
            # "-DPYTHON_EXECUTABLE={}".format(sys.executable),
            "-DCMAKE_BUILD_TYPE={}".format(cfg),  # not used on MSVC, but no harm
            "-DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded",
            "-DOPENVDB_CORE_SHARED=OFF",
            "-DTBB_TEST=OFF",
            f"-DCMAKE_CXX_FLAGS=-fPIC {'-static-libgcc -static-libstdc++' if system == 'linux' else '/MT /EHsc' if system == 'windows' else ''}"
        ]
        
        build_args = []

        # if not cmake_generator:
        #    cmake_args += ["-GNinja"]

        self.parallel = 4
        if "CMAKE_BUILD_PARALLEL_LEVEL" not in os.environ and system != 'windows':
            if hasattr(self, "parallel") and self.parallel:
                build_args += ["-j{}".format(self.parallel)]

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        print(cmake_args, build_args)

        subprocess.check_call(
            ["cmake", ext.sourcedir] + cmake_args, cwd=self.build_temp
        )
        subprocess.check_call(
            ["cmake", "--build", ".", "--target", "_coacd" , '--config', cfg] + build_args,
            cwd=self.build_temp,
        )


setup(
    name="coacd",
    version="1.0.2",
    author_email="xiwei@ucsd.edu",
    keywords="collision convex decomposition",
    description="Approximate Convex Decomposition for 3D Meshes with Collision-Aware Concavity and Tree Search",
    classifiers=[
        "Operating System :: POSIX :: Linux",
        "Intended Audience :: Developers",
        "Intended Audience :: Education",
        "Intended Audience :: Other Audience",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Natural Language :: English",
        "Framework :: Robot Framework :: Tool",
        "Programming Language :: C++",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Topic :: Education",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: Utilities",
    ],
    license="MIT",
    url="https://colin97.github.io/CoACD/",
    packages=["coacd"],
    python_requires=">=3.6",
    install_requires=["numpy"],
    ext_modules=[CMakeExtension("coacd")],
    cmdclass={"build_ext": CMakeBuild, "bdist_wheel": bdist_wheel_abi3},
    zip_safe=False,
    package_dir={"coacd": os.path.join("python/package")},
    scripts=["python/package/bin/coacd"]
)
