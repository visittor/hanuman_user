from distutils.core import setup
from Cython.Build import cythonize
from distutils.extension import Extension
import numpy

# TO RUN: python setup.py build_ext --inplace
# ext_modules = [Extension('func1', ['util/func1_pc.py', 'util/funct2_pc.py'],)]
sourcefiles = ['cython_function/find_pattern.pyx']

extensions = [Extension("example", sourcefiles)]

setup(
    ext_modules = cythonize("cython_function/*.pyx"),
    # ext_modules = cythonize(extensions),
    include_dirs=[numpy.get_include()]
)
