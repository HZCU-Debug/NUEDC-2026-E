"""Build the optional in-place C++ acceleration module."""

import os
import sys

import numpy
from setuptools import Extension, setup


using_mingw = any("mingw" in argument.lower() for argument in sys.argv)
if os.name == "nt" and not using_mingw:
    compile_args = ["/O2", "/std:c++14"]
    link_args = []
else:
    compile_args = [
        "-O3",
        "-std=c++14",
        "-fopenmp",
        "-march=native",
    ]
    link_args = ["-fopenmp"]

if using_mingw:
    # Modern MinGW links against the universal CRT.  Python 3.7's legacy
    # distutils still asks for an obsolete ``libmsvcr140.a`` import library
    # that is not shipped by current toolchains.
    import distutils.cygwinccompiler

    distutils.cygwinccompiler.get_msvcr = lambda: []


setup(
    name="vision-fast",
    version="0.1.0",
    ext_modules=[
        Extension(
            "vision_fast",
            sources=["native/vision_fast.cpp"],
            include_dirs=[numpy.get_include()],
            language="c++",
            extra_compile_args=compile_args,
            extra_link_args=link_args,
        )
    ],
)
