mkdir -p build
cd build
cmake .. \
  -DPYTHON_INCLUDE_DIR=$(python3 -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())")  \
  -DPYTHON_LIBRARY_DIR="$HOME/.local/lib/python3.8/site-packages"
  -DPYTHON_EXECUTABLE=$(which python3)
make clean
make
./tests
make install
cd ..
