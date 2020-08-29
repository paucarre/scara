cd build
cmake ..  -DPYTHON_EXECUTABLE=$CONDA_PYTHON_EXE -DPYTHON_LIBRARY_DIR=${CONDA_PREFIX}/lib/python3.8/site-packages
make clean
make
./test
make install
cd ..